#include "ble_publisher.h"
#include "audio_dsp.h"
#include "calibration.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "heap_diag.h"

static const char* TAG = "ble_publisher";

#ifdef CONFIG_BT_ENABLED

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "constants.h"
#include "esp_app_desc.h"
#include "event_group.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "pb_encode.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "noise.pb.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// --- Custom service / characteristic UUIDs -----------------------------------
//
// Base UUID for noise service: 7ed2f2c4-69e8-4f7c-9c93-7a3b1e5d0a00
// The single `record` characteristic uses the same base with last byte 0x01.
// Bytes are little-endian in the BLE_UUID128_INIT macro.
#define NOISE_UUID_BASE(last) BLE_UUID128_INIT( \
    (last), 0x0a, 0x5d, 0x1e, 0x3b, 0x7a, 0x93, 0x9c,    \
    0x7c, 0x4f, 0xe8, 0x69, 0xc4, 0xf2, 0xd2, 0x7e)

static const ble_uuid128_t noise_svc_uuid  = NOISE_UUID_BASE(0x00);
static const ble_uuid128_t chr_record_uuid = NOISE_UUID_BASE(0x01);
static const ble_uuid128_t chr_cal_uuid    = NOISE_UUID_BASE(0x02);

// Value handles filled by the stack at GATT registration.
static uint16_t h_record = 0;
static uint16_t h_cal    = 0;

// Connection + subscription state.
static volatile uint16_t curr_conn       = 0xffff;
static volatile bool     record_subscribed = false;

// Cached encoding of the latest record. Used by the READ callback (NimBLE
// host task) and refreshed by the publisher task after each notify. A mutex
// guards the buffer because the encoder and the reader run on different
// tasks. The notify path itself doesn't need the mutex — it encodes into a
// local buffer and hands the mbuf to NimBLE.
//
// 128 bytes comfortably exceeds the encoded NoiseRecording-with-one-Record
// size (Record_size = 69, plus a few bytes of wrapper overhead).
#define ENCODED_BUF_SZ 128
static uint8_t            cached_buf[ENCODED_BUF_SZ];
static size_t             cached_len = 0;
static SemaphoreHandle_t  cached_mtx = NULL;

// --- DIS strings -------------------------------------------------------------
static const char manufacturer_name[] = "Kulturspektakel";
static char firmware_revision[16] = "unknown";
static char serial_number[DEVICE_ID_LENGTH + 1];

// --- Access callbacks --------------------------------------------------------
static int read_record(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg);
static int read_dis_string(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg);
static int access_cal(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg);

// --- GATT service definition -------------------------------------------------
static const struct ble_gatt_svc_def gatt_svcs[] = {
  {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = BLE_UUID16_DECLARE(0x180A),  // Device Information Service
    .characteristics = (struct ble_gatt_chr_def[]) {
      { .uuid = BLE_UUID16_DECLARE(0x2A29), .access_cb = read_dis_string,
        .arg = (void*)manufacturer_name, .flags = BLE_GATT_CHR_F_READ },
      { .uuid = BLE_UUID16_DECLARE(0x2A26), .access_cb = read_dis_string,
        .arg = firmware_revision, .flags = BLE_GATT_CHR_F_READ },
      { .uuid = BLE_UUID16_DECLARE(0x2A25), .access_cb = read_dis_string,
        .arg = serial_number, .flags = BLE_GATT_CHR_F_READ },
      { 0 },
    },
  },
  {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &noise_svc_uuid.u,
    .characteristics = (struct ble_gatt_chr_def[]) {
      { .uuid = &chr_record_uuid.u, .access_cb = read_record,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &h_record },
      // Calibration offset, signed 32-bit hundredths of dB, little-endian.
      // Read returns current offset; write persists to NVS and sets the
      // CALIBRATED event bit (so the status LED turns green).
      { .uuid = &chr_cal_uuid.u, .access_cb = access_cal,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE, .val_handle = &h_cal },
      { 0 },
    },
  },
  { 0 },
};

// --- Read helpers ------------------------------------------------------------
static int read_dis_string(
    uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg
) {
  const char* s = (const char*)arg;
  return os_mbuf_append(ctxt->om, s, strlen(s)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int read_record(
    uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg
) {
  int rc = 0;
  xSemaphoreTake(cached_mtx, portMAX_DELAY);
  if (cached_len > 0) {
    if (os_mbuf_append(ctxt->om, cached_buf, cached_len) != 0) {
      rc = BLE_ATT_ERR_INSUFFICIENT_RES;
    }
  }
  xSemaphoreGive(cached_mtx);
  return rc;
}

static int access_cal(
    uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg
) {
  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    int32_t v = calibration_offset_x100();
    return os_mbuf_append(ctxt->om, &v, sizeof(v)) == 0
        ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }
  if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
    if (OS_MBUF_PKTLEN(ctxt->om) != sizeof(int32_t)) {
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    int32_t v = 0;
    uint16_t copied = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, &v, sizeof(v), &copied) != 0 || copied != sizeof(v)) {
      return BLE_ATT_ERR_UNLIKELY;
    }
    esp_err_t err = calibration_set(v);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "calibration_set(%ld) failed: %s", (long)v, esp_err_to_name(err));
      return BLE_ATT_ERR_UNLIKELY;
    }
    ESP_LOGI(TAG, "calibration offset set via BLE: %+.2f dB", v / 100.0f);
    return 0;
  }
  return BLE_ATT_ERR_UNLIKELY;
}

// --- GAP --------------------------------------------------------------------
static uint8_t own_addr_type;
static char    adv_name[24];

static int gap_event(struct ble_gap_event* ev, void* arg);

static void start_advertising(void) {
  // Primary advertisement: flags + the 128-bit service UUID. Web Bluetooth's
  // `filters: [{services: [...]}]` only matches against primary-adv data, so
  // the UUID has to live here for Chrome's device picker to find us. The
  // full name goes in the scan response (most scanners combine both for
  // display). Layout: flags (3 B) + UUID (16 + 2 = 18 B) = 21 B / 31 B max.
  struct ble_hs_adv_fields adv_fields;
  memset(&adv_fields, 0, sizeof(adv_fields));
  adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  adv_fields.uuids128 = (ble_uuid128_t*)&noise_svc_uuid;
  adv_fields.num_uuids128 = 1;
  adv_fields.uuids128_is_complete = 1;
  int rc = ble_gap_adv_set_fields(&adv_fields);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_adv_set_fields: %d", rc);
    return;
  }

  // Scan response: the device name. Up to 29 bytes available here.
  struct ble_hs_adv_fields rsp_fields;
  memset(&rsp_fields, 0, sizeof(rsp_fields));
  rsp_fields.name = (uint8_t*)adv_name;
  rsp_fields.name_len = strlen(adv_name);
  rsp_fields.name_is_complete = 1;
  rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields: %d", rc);
    return;
  }

  // 250 ms advertising interval — fast enough that a phone scanner can both
  // see the device AND complete a connect request within its typical 10 s
  // scan window. The 5 s interval we had before saved a tiny bit of power
  // but made connection establishment unreliable; phones could see the name
  // (cached adv) but couldn't actually pair. Advertising automatically
  // stops while connected, so the higher rate only applies when idle.
  struct ble_gap_adv_params adv_params = {
      .conn_mode = BLE_GAP_CONN_MODE_UND,
      .disc_mode = BLE_GAP_DISC_MODE_GEN,
      .itvl_min = BLE_GAP_ADV_ITVL_MS(250),
      .itvl_max = BLE_GAP_ADV_ITVL_MS(250),
  };
  rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event, NULL);
  if (rc != 0) ESP_LOGE(TAG, "ble_gap_adv_start: %d", rc);
}

static int gap_event(struct ble_gap_event* ev, void* arg) {
  switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
      if (ev->connect.status == 0) {
        curr_conn = ev->connect.conn_handle;
        ESP_LOGI(TAG, "BLE client connected");
      } else {
        start_advertising();
      }
      break;
    case BLE_GAP_EVENT_DISCONNECT:
      ESP_LOGI(TAG, "BLE client disconnected (reason %d)", ev->disconnect.reason);
      curr_conn = 0xffff;
      record_subscribed = false;
      start_advertising();
      break;
    case BLE_GAP_EVENT_SUBSCRIBE:
      if (ev->subscribe.attr_handle == h_record) {
        record_subscribed = ev->subscribe.cur_notify != 0;
      }
      break;
    case BLE_GAP_EVENT_MTU:
      ESP_LOGI(TAG, "MTU updated to %d", ev->mtu.value);
      break;
    default:
      break;
  }
  return 0;
}

// --- Host sync / host task ---------------------------------------------------
static void on_sync(void) {
  int rc = ble_hs_id_infer_auto(0, &own_addr_type);
  if (rc != 0) { ESP_LOGE(TAG, "ble_hs_id_infer_auto: %d", rc); return; }
  start_advertising();
}

static void on_reset(int reason) {
  ESP_LOGI(TAG, "BLE host reset (reason %d)", reason);
}

static void host_task(void* param) {
  nimble_port_run();   // blocks until nimble_port_stop()
  nimble_port_freertos_deinit();
}

// --- Main task --------------------------------------------------------------
static void compose_adv_name(void) {
  // Use the eFuse-loaded device name directly (e.g. "Wischnevsky"). Fall
  // back to a static placeholder when device_id hasn't been provisioned.
  if (strlen(DEVICE_ID) > 0) {
    strlcpy(adv_name, DEVICE_ID, sizeof(adv_name));
  } else {
    strlcpy(adv_name, "NoiseMonitor", sizeof(adv_name));
  }
}


void ble_publisher(void* params) {
  // Wait for device_id so the advertised name + DIS Serial Number reflect it.
  xEventGroupWaitBits(event_group, DEVICE_ID_LOADED, false, true, portMAX_DELAY);

  strlcpy(serial_number, DEVICE_ID, sizeof(serial_number));
  const esp_app_desc_t* d = esp_app_get_description();
  if (d) strlcpy(firmware_revision, d->version, sizeof(firmware_revision));
  compose_adv_name();

  cached_mtx = xSemaphoreCreateMutex();
  if (!cached_mtx) {
    ESP_LOGE(TAG, "failed to create cached_mtx");
    vTaskDelete(NULL);
    return;
  }

  heap_diag("before nimble_port_init");
  if (nimble_port_init() != ESP_OK) {
    ESP_LOGE(TAG, "nimble_port_init failed");
    vTaskDelete(NULL);
    return;
  }

  ble_hs_cfg.sync_cb = on_sync;
  ble_hs_cfg.reset_cb = on_reset;

  ble_svc_gap_init();
  ble_svc_gatt_init();

  int rc = ble_gatts_count_cfg(gatt_svcs);
  if (rc != 0) { ESP_LOGE(TAG, "ble_gatts_count_cfg: %d", rc); vTaskDelete(NULL); return; }
  rc = ble_gatts_add_svcs(gatt_svcs);
  if (rc != 0) { ESP_LOGE(TAG, "ble_gatts_add_svcs: %d", rc); vTaskDelete(NULL); return; }

  ble_svc_gap_device_name_set(adv_name);
  nimble_port_freertos_init(host_task);

  ESP_LOGI(TAG, "BLE peripheral up; advertising as %s", adv_name);

  // Drain loop: encode each per-second record as the same protobuf bytes the
  // MQTT publisher sends, notify if a subscriber is attached, and refresh the
  // cached copy for on-demand READs.
  uint8_t local_buf[ENCODED_BUF_SZ];
  while (true) {
    record_t r;
    if (xQueueReceive(ble_publisher_queue, &r, portMAX_DELAY) != pdTRUE) continue;

    size_t n = record_encode_single(&r, local_buf, sizeof(local_buf));
    if (n == 0) continue;

    // Refresh cached bytes for cold READs.
    xSemaphoreTake(cached_mtx, portMAX_DELAY);
    memcpy(cached_buf, local_buf, n);
    cached_len = n;
    xSemaphoreGive(cached_mtx);

    if (curr_conn != 0xffff && record_subscribed) {
      struct os_mbuf* om = ble_hs_mbuf_from_flat(local_buf, n);
      if (om) ble_gatts_notify_custom(curr_conn, h_record, om);
    }
  }
}

#else  // !CONFIG_BT_ENABLED — drain the queue so DSP non-blocking sends keep succeeding.

void ble_publisher(void* params) {
  ESP_LOGW(TAG, "CONFIG_BT_ENABLED=n; BLE peripheral not started. Draining queue.");
  record_t r;
  while (true) {
    xQueueReceive(ble_publisher_queue, &r, portMAX_DELAY);
    // discard
  }
}

#endif  // CONFIG_BT_ENABLED
