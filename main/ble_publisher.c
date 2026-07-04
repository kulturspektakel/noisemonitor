#include "ble_publisher.h"
#include "audio_dsp.h"
#include "calibration.h"
#include "esp_log.h"
#include "log_uploader.h"
#include "power_management.h"
#include "wifi_connect.h"
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
static const ble_uuid128_t chr_wifi_uuid   = NOISE_UUID_BASE(0x03);
static const ble_uuid128_t chr_uploads_uuid = NOISE_UUID_BASE(0x04);
static const ble_uuid128_t chr_wifi_status_uuid = NOISE_UUID_BASE(0x05);

// Value handles filled by the stack at GATT registration.
static uint16_t h_record = 0;
static uint16_t h_cal    = 0;
static uint16_t h_wifi   = 0;
static uint16_t h_batt   = 0;
static uint16_t h_uploads = 0;
static uint16_t h_wifi_status = 0;

// Connection + subscription state.
static volatile uint16_t curr_conn       = 0xffff;
static volatile bool     record_subscribed = false;
static volatile bool     batt_subscribed   = false;
static volatile bool     uploads_subscribed = false;
static volatile bool     wifi_status_subscribed = false;
// Last battery percentage pushed via notify. 0xff = "none yet" sentinel (real
// values are 0..100), forcing a fresh notify on (re)subscribe.
static volatile uint8_t  batt_last_notified = 0xff;
// Last pending-uploads count pushed via notify. -1 = "none yet" sentinel,
// forcing a fresh notify on (re)subscribe.
static volatile int      uploads_last_notified = -1;
// Last wifi_status pushed via notify. 0xff = "none yet" sentinel (real values
// are 0..2), forcing a fresh notify on (re)subscribe.
static volatile uint8_t  wifi_status_last_notified = 0xff;

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
static int write_wifi(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg);
static int read_battery(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg);
static int read_uploads(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg);
static int read_wifi_status(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg);

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
    .uuid = BLE_UUID16_DECLARE(0x180F),  // Battery Service (SIG standard)
    .characteristics = (struct ble_gatt_chr_def[]) {
      // Battery Level (0x2A19): single uint8 percentage, 0..100. Read returns
      // the current battery_percentage(); notify pushes it when it changes.
      { .uuid = BLE_UUID16_DECLARE(0x2A19), .access_cb = read_battery,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &h_batt },
      { 0 },
    },
  },
  {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &noise_svc_uuid.u,
    .characteristics = (struct ble_gatt_chr_def[]) {
      { .uuid = &chr_record_uuid.u, .access_cb = read_record,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &h_record },
      // Per-band calibration: 31 signed bytes, one per 1/3-octave band (low to
      // high), in 0.5 dB steps (range ±63.5 dB; the realistic trim is well
      // within ±16 dB now that the SPL anchor handles the bulk). 31 bytes goes
      // in a single Write Request once the client negotiates a larger MTU —
      // automatic on iOS/Android, and NimBLE's preferred MTU is 256. (If a
      // client never raises MTU past the 23-byte default it falls back to a
      // Long Write, which NimBLE reassembles into one mbuf for this callback.)
      // Read returns the current 31 bytes; write persists to NVS.
      { .uuid = &chr_cal_uuid.u, .access_cb = access_cal,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE, .val_handle = &h_cal },
      // WiFi credentials, write-only. Payload format:
      //   [u8 ssid_len][ssid bytes][u8 pw_len][pw bytes]
      // ssid_len in 1..32, pw_len in 0..63 (0 = open network).
      { .uuid = &chr_wifi_uuid.u, .access_cb = write_wifi,
        .flags = BLE_GATT_CHR_F_WRITE, .val_handle = &h_wifi },
      // Pending log uploads: read-only uint16 (little-endian) count of log
      // files still waiting to be uploaded. Notifies when the count changes.
      { .uuid = &chr_uploads_uuid.u, .access_cb = read_uploads,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &h_uploads },
      // WiFi connection status: read-only uint8. 0 = disconnected,
      // 1 = connecting, 2 = connected. Notifies when the state changes.
      { .uuid = &chr_wifi_status_uuid.u, .access_cb = read_wifi_status,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &h_wifi_status },
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
    int8_t bands[CALIBRATION_BANDS];
    calibration_get_bands(bands, CALIBRATION_BANDS);
    return os_mbuf_append(ctxt->om, bands, sizeof(bands)) == 0
        ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }
  if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
    if (OS_MBUF_PKTLEN(ctxt->om) != CALIBRATION_BANDS) {
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    int8_t bands[CALIBRATION_BANDS];
    uint16_t copied = 0;
    if (ble_hs_mbuf_to_flat(ctxt->om, bands, sizeof(bands), &copied) != 0
        || copied != sizeof(bands)) {
      return BLE_ATT_ERR_UNLIKELY;
    }
    esp_err_t err = calibration_set_bands(bands, CALIBRATION_BANDS);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "calibration_set_bands failed: %s", esp_err_to_name(err));
      return BLE_ATT_ERR_UNLIKELY;
    }
    ESP_LOGI(TAG, "per-band calibration set via BLE (%d bands)", CALIBRATION_BANDS);
    return 0;
  }
  return BLE_ATT_ERR_UNLIKELY;
}

static int write_wifi(
    uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg
) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;

  // Pull the payload into a flat buffer. Max plausible size: 1 + 32 + 1 + 63 = 97.
  uint8_t buf[128];
  uint16_t total = OS_MBUF_PKTLEN(ctxt->om);
  if (total < 2 || total > sizeof(buf)) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
  uint16_t copied = 0;
  if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &copied) != 0 || copied != total) {
    return BLE_ATT_ERR_UNLIKELY;
  }

  // Parse: [u8 ssid_len][ssid][u8 pw_len][pw]
  uint8_t ssid_len = buf[0];
  if (ssid_len < 1 || ssid_len > 32 || (size_t)(1 + ssid_len + 1) > total) {
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
  }
  uint8_t pw_len = buf[1 + ssid_len];
  if (pw_len > 63 || (size_t)(1 + ssid_len + 1 + pw_len) != total) {
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
  }

  char ssid[33];
  char password[64];
  memcpy(ssid, buf + 1, ssid_len);
  ssid[ssid_len] = '\0';
  memcpy(password, buf + 1 + ssid_len + 1, pw_len);
  password[pw_len] = '\0';

  esp_err_t err = wifi_connect_set_credentials(ssid, password);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "wifi_connect_set_credentials: %s", esp_err_to_name(err));
    return BLE_ATT_ERR_UNLIKELY;
  }
  // Log the SSID but not the password — keeping it out of the serial log
  // matches the "never expose the password" goal of the write-only characteristic.
  ESP_LOGI(TAG, "WiFi credentials set via BLE: ssid=%s (pw %u bytes)", ssid, pw_len);
  return 0;
}

// Battery percentage to report over BLE. On USB the discharge-curve percentage
// is meaningless (the pack reads high while charging), so report 100% — same
// on-battery gate as record_encode_single's battery_mv (usb_voltage <= 1000).
static uint8_t battery_pct_ble(void) {
  if (usb_voltage > 1000) return 100;
  return (uint8_t)battery_percentage();  // 0..100
}

static int read_battery(
    uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg
) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
  uint8_t pct = battery_pct_ble();
  return os_mbuf_append(ctxt->om, &pct, sizeof(pct)) == 0
      ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// Clamp the pending-uploads count into the uint16 wire range (negative shouldn't
// happen — the uploader floors at 0 — but guard anyway).
static uint16_t uploads_count_u16(void) {
  int n = log_files_to_upload;
  if (n < 0) n = 0;
  if (n > 0xffff) n = 0xffff;
  return (uint16_t)n;
}

static int read_uploads(
    uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg
) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
  uint16_t n = uploads_count_u16();  // little-endian on the ESP32
  return os_mbuf_append(ctxt->om, &n, sizeof(n)) == 0
      ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int read_wifi_status(
    uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg
) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
  uint8_t s = (uint8_t)wifi_status;  // 0=disconnected, 1=connecting, 2=connected
  return os_mbuf_append(ctxt->om, &s, sizeof(s)) == 0
      ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
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

  // 1 s advertising interval — common "low-power-but-still-responsive"
  // value (Eddystone beacons, most accessories that aren't trying to be
  // discovered instantly). Web Bluetooth picker sees the device within
  // 1–2 bursts; connection setup adds maybe a second. The 250 ms we used
  // earlier was tuned for fast pairing while debugging; now that the
  // device is provisioned rarely, the radio duty cycle wins.
  // (5 s was too long — phones cached the name but couldn't pair reliably.)
  struct ble_gap_adv_params adv_params = {
      .conn_mode = BLE_GAP_CONN_MODE_UND,
      .disc_mode = BLE_GAP_DISC_MODE_GEN,
      .itvl_min = BLE_GAP_ADV_ITVL_MS(1000),
      .itvl_max = BLE_GAP_ADV_ITVL_MS(1000),
  };
  rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event, NULL);
  if (rc != 0) ESP_LOGE(TAG, "ble_gap_adv_start: %d", rc);
}

// Log the live connection parameters for `conn`. Supervision timeout is the
// one to watch for "drops after a few minutes": if it's short and we miss a
// run of connection events (e.g. CPU at 40 MHz under DFS), the link times out.
static void log_conn_params(uint16_t conn, const char* what) {
  struct ble_gap_conn_desc desc;
  if (ble_gap_conn_find(conn, &desc) != 0) {
    ESP_LOGW(TAG, "%s: ble_gap_conn_find failed", what);
    return;
  }
  ESP_LOGI(TAG,
      "%s: itvl=%u (%.2f ms) latency=%u sup_timeout=%u (%u ms)",
      what, desc.conn_itvl, desc.conn_itvl * 1.25,
      desc.conn_latency, desc.supervision_timeout, desc.supervision_timeout * 10);
}

static int gap_event(struct ble_gap_event* ev, void* arg) {
  switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
      if (ev->connect.status == 0) {
        curr_conn = ev->connect.conn_handle;
        ESP_LOGI(TAG, "BLE client connected");
        log_conn_params(curr_conn, "connect params");
        // A client attaching is a good moment to try WiFi (e.g. someone is
        // provisioning or checking on the device). No-op if already connected.
        wifi_connect_trigger();
      } else {
        ESP_LOGW(TAG, "connect failed (status=%d); re-advertising", ev->connect.status);
        start_advertising();
      }
      break;
    case BLE_GAP_EVENT_DISCONNECT:
      // reason is an HCI error code. Common ones: 0x08 supervision timeout,
      // 0x13 remote user terminated, 0x16 local host terminated,
      // 0x3d connection failed to establish / LMP response timeout.
      ESP_LOGW(TAG, "BLE client disconnected: reason=%d (0x%02x)",
               ev->disconnect.reason, ev->disconnect.reason);
      curr_conn = 0xffff;
      record_subscribed = false;
      batt_subscribed = false;
      uploads_subscribed = false;
      wifi_status_subscribed = false;
      start_advertising();
      break;
    case BLE_GAP_EVENT_CONN_UPDATE:
      // The central (Chrome/OS) often renegotiates interval/timeout shortly
      // after connecting; log the new values so we see what we actually got.
      ESP_LOGI(TAG, "conn update (status=%d)", ev->conn_update.status);
      log_conn_params(ev->conn_update.conn_handle, "updated params");
      break;
    case BLE_GAP_EVENT_SUBSCRIBE:
      if (ev->subscribe.attr_handle == h_record) {
        record_subscribed = ev->subscribe.cur_notify != 0;
        ESP_LOGI(TAG, "record notifications %s", record_subscribed ? "ENABLED" : "disabled");
      } else if (ev->subscribe.attr_handle == h_batt) {
        batt_subscribed = ev->subscribe.cur_notify != 0;
        // Force the next drain iteration to push the current level.
        batt_last_notified = 0xff;
        ESP_LOGI(TAG, "battery notifications %s", batt_subscribed ? "ENABLED" : "disabled");
      } else if (ev->subscribe.attr_handle == h_uploads) {
        uploads_subscribed = ev->subscribe.cur_notify != 0;
        // Force the next drain iteration to push the current count.
        uploads_last_notified = -1;
        ESP_LOGI(TAG, "uploads notifications %s", uploads_subscribed ? "ENABLED" : "disabled");
      } else if (ev->subscribe.attr_handle == h_wifi_status) {
        wifi_status_subscribed = ev->subscribe.cur_notify != 0;
        // Force the next drain iteration to push the current state.
        wifi_status_last_notified = 0xff;
        ESP_LOGI(TAG, "wifi-status notifications %s", wifi_status_subscribed ? "ENABLED" : "disabled");
      }
      break;
    case BLE_GAP_EVENT_MTU:
      ESP_LOGI(TAG, "MTU updated to %d", ev->mtu.value);
      break;
    default:
      ESP_LOGD(TAG, "unhandled GAP event type=%d", ev->type);
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
  nimble_port_run();  // blocks until nimble_port_stop()
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
      if (!om) {
        // mbuf pool exhausted — notifies aren't draining. A sustained run of
        // this just before a disconnect points at the host running out of
        // buffers rather than a radio/timeout problem.
        ESP_LOGW(TAG, "notify skipped: mbuf alloc failed (seq %lu)", (unsigned long)r.seq_no);
      } else {
        int rc = ble_gatts_notify_custom(curr_conn, h_record, om);
        if (rc != 0) ESP_LOGW(TAG, "notify failed: rc=%d (seq %lu)", rc, (unsigned long)r.seq_no);
      }
    }

    // Battery level: notify only when the percentage changes (it moves slowly,
    // so this is near-silent). Piggybacks on the 1 s record cadence rather than
    // running its own timer.
    if (curr_conn != 0xffff && batt_subscribed) {
      uint8_t pct = battery_pct_ble();
      if (pct != batt_last_notified) {
        struct os_mbuf* om = ble_hs_mbuf_from_flat(&pct, sizeof(pct));
        if (om && ble_gatts_notify_custom(curr_conn, h_batt, om) == 0) {
          batt_last_notified = pct;
        }
      }
    }

    // Pending uploads: notify when the count changes (uploads drain and new log
    // files rotate in slowly relative to the 1 s cadence, so this is quiet).
    if (curr_conn != 0xffff && uploads_subscribed) {
      int n = log_files_to_upload;
      if (n != uploads_last_notified) {
        uint16_t v = uploads_count_u16();
        struct os_mbuf* om = ble_hs_mbuf_from_flat(&v, sizeof(v));
        if (om && ble_gatts_notify_custom(curr_conn, h_uploads, om) == 0) {
          uploads_last_notified = n;
        }
      }
    }

    // WiFi connection status: notify when the state changes.
    if (curr_conn != 0xffff && wifi_status_subscribed) {
      uint8_t s = (uint8_t)wifi_status;
      if (s != wifi_status_last_notified) {
        struct os_mbuf* om = ble_hs_mbuf_from_flat(&s, sizeof(s));
        if (om && ble_gatts_notify_custom(curr_conn, h_wifi_status, om) == 0) {
          wifi_status_last_notified = s;
        }
      }
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
