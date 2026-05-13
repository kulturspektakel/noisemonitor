#include "ble_publisher.h"
#include "audio_dsp.h"
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
#include "calibration.h"
#include "constants.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "event_group.h"
#include "freertos/event_groups.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "log_uploader.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "power_management.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// --- Custom service / characteristic UUIDs -----------------------------------
//
// Base UUID for noise service: 7ed2f2c4-69e8-4f7c-9c93-7a3b1e5d0a00
// Per-characteristic UUIDs increment the last byte. Bytes are little-endian
// in the BLE_UUID128_INIT macro.
#define NOISE_UUID_BASE(last) BLE_UUID128_INIT( \
    (last), 0x0a, 0x5d, 0x1e, 0x3b, 0x7a, 0x93, 0x9c,    \
    0x7c, 0x4f, 0xe8, 0x69, 0xc4, 0xf2, 0xd2, 0x7e)

static const ble_uuid128_t noise_svc_uuid          = NOISE_UUID_BASE(0x00);
static const ble_uuid128_t chr_status_uuid         = NOISE_UUID_BASE(0x01);
static const ble_uuid128_t chr_laeq_1s_uuid        = NOISE_UUID_BASE(0x02);
static const ble_uuid128_t chr_laeq_1m_uuid        = NOISE_UUID_BASE(0x03);
static const ble_uuid128_t chr_laeq_15m_uuid       = NOISE_UUID_BASE(0x04);
static const ble_uuid128_t chr_laeq_30m_uuid       = NOISE_UUID_BASE(0x05);
static const ble_uuid128_t chr_lceq_1m_uuid        = NOISE_UUID_BASE(0x06);
static const ble_uuid128_t chr_lceq_15m_uuid       = NOISE_UUID_BASE(0x07);
static const ble_uuid128_t chr_lafmax_1m_uuid      = NOISE_UUID_BASE(0x08);
static const ble_uuid128_t chr_lcpeak_1m_uuid      = NOISE_UUID_BASE(0x09);
static const ble_uuid128_t chr_battery_uuid        = NOISE_UUID_BASE(0x0a);
static const ble_uuid128_t chr_uptime_uuid         = NOISE_UUID_BASE(0x0b);
static const ble_uuid128_t chr_cal_offset_uuid     = NOISE_UUID_BASE(0x0c);

// Discriminator passed via NimBLE's `.arg` field (a void*) so a single
// `access_cb` knows which characteristic it's serving. Cast to void* via
// CHR_TAG so the compiler doesn't complain about the int→pointer conversion.
enum chr_tag {
  TAG_LAEQ_1S = 1,    // start at 1: NULL=0 would be ambiguous
  TAG_LAEQ_1M,
  TAG_LAEQ_15M,
  TAG_LAEQ_30M,
  TAG_LCEQ_1M,
  TAG_LCEQ_15M,
  TAG_LAFMAX_1M,
  TAG_LCPEAK_1M,
};
#define CHR_TAG(t) ((void*)(uintptr_t)(t))

// --- Characteristic value handles (filled by stack at GATT registration) -----
static uint16_t h_status        = 0;
static uint16_t h_laeq_1s       = 0;
static uint16_t h_laeq_1m       = 0;
static uint16_t h_laeq_15m      = 0;
static uint16_t h_laeq_30m      = 0;
static uint16_t h_lceq_1m       = 0;
static uint16_t h_lceq_15m      = 0;
static uint16_t h_lafmax_1m     = 0;
static uint16_t h_lcpeak_1m     = 0;
static uint16_t h_battery       = 0;
static uint16_t h_uptime        = 0;
static uint16_t h_cal_offset    = 0;

// --- Subscription tracking ---------------------------------------------------
// Each characteristic with NOTIFY can be enabled by a client via CCCD write.
// NimBLE delivers these as BLE_GAP_EVENT_SUBSCRIBE.
typedef struct {
  uint16_t* h;
  bool subscribed;
} sub_state_t;

static sub_state_t subs[] = {
  { &h_status,     false },
  { &h_laeq_1s,    false },
  { &h_laeq_1m,    false },
  { &h_laeq_15m,   false },
  { &h_laeq_30m,   false },
  { &h_lceq_1m,    false },
  { &h_lceq_15m,   false },
  { &h_lafmax_1m,  false },
  { &h_lcpeak_1m,  false },
  { &h_battery,    false },
};

static volatile uint16_t curr_conn = 0xffff;

// --- Cached last-notified values (for change-based notify) -------------------
static uint8_t  last_laeq_15m   = 0;
static uint8_t  last_laeq_30m   = 0;
static uint8_t  last_lceq_15m   = 0;
static uint16_t last_battery_mv = 0;
static uint8_t  last_status_flags = 0xff;     // force initial notify
static uint16_t last_status_pending = 0xffff;
static int64_t  last_status_us = 0;
static int64_t  last_battery_us = 0;
static int64_t  last_laeq_15m_us = 0;
static int64_t  last_laeq_30m_us = 0;
static int64_t  last_lceq_15m_us = 0;

// --- DIS strings -------------------------------------------------------------
static const char manufacturer_name[] = "Kulturspektakel";
static char firmware_revision[16] = "unknown";
static char serial_number[DEVICE_ID_LENGTH + 1];

// --- Access callbacks --------------------------------------------------------
static int read_status(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg);
static int read_uint8(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg);
static int read_uint8_with_seconds(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg);
static int read_battery(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg);
static int read_uptime(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg);
static int read_cal_offset(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg);
static int read_dis_string(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg);

// Latest sampled values (for read access and periodic notify).
static volatile uint8_t  v_laeq_1s = 0;
static volatile uint8_t  v_lceq_1s = 0;
static volatile uint8_t  v_lafmax_1s = 0;
static volatile uint8_t  v_lcfmax_1s = 0;
static volatile uint8_t  v_lcpeak_1s = 0;

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
      { .uuid = &chr_status_uuid.u,      .access_cb = read_status,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &h_status },
      { .uuid = &chr_laeq_1s_uuid.u,     .access_cb = read_uint8, .arg = CHR_TAG(TAG_LAEQ_1S),
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &h_laeq_1s },
      { .uuid = &chr_laeq_1m_uuid.u,     .access_cb = read_uint8, .arg = CHR_TAG(TAG_LAEQ_1M),
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &h_laeq_1m },
      { .uuid = &chr_laeq_15m_uuid.u,    .access_cb = read_uint8_with_seconds, .arg = CHR_TAG(TAG_LAEQ_15M),
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &h_laeq_15m },
      { .uuid = &chr_laeq_30m_uuid.u,    .access_cb = read_uint8_with_seconds, .arg = CHR_TAG(TAG_LAEQ_30M),
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &h_laeq_30m },
      { .uuid = &chr_lceq_1m_uuid.u,     .access_cb = read_uint8, .arg = CHR_TAG(TAG_LCEQ_1M),
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &h_lceq_1m },
      { .uuid = &chr_lceq_15m_uuid.u,    .access_cb = read_uint8_with_seconds, .arg = CHR_TAG(TAG_LCEQ_15M),
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &h_lceq_15m },
      { .uuid = &chr_lafmax_1m_uuid.u,   .access_cb = read_uint8, .arg = CHR_TAG(TAG_LAFMAX_1M),
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &h_lafmax_1m },
      { .uuid = &chr_lcpeak_1m_uuid.u,   .access_cb = read_uint8, .arg = CHR_TAG(TAG_LCPEAK_1M),
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &h_lcpeak_1m },
      { .uuid = &chr_battery_uuid.u,     .access_cb = read_battery,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &h_battery },
      { .uuid = &chr_uptime_uuid.u,      .access_cb = read_uptime,
        .flags = BLE_GATT_CHR_F_READ, .val_handle = &h_uptime },
      { .uuid = &chr_cal_offset_uuid.u,  .access_cb = read_cal_offset,
        .flags = BLE_GATT_CHR_F_READ, .val_handle = &h_cal_offset },
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

static int read_status(
    uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg
) {
  EventBits_t bits = xEventGroupGetBits(event_group);
  uint8_t flags = 0;
  if (bits & WIFI_CONNECTED) flags |= 0x01;
  if (bits & USB_CONNECTED)  flags |= 0x02;
  if (bits & CALIBRATED)     flags |= 0x04;
  if (bits & TIME_SET)       flags |= 0x08;
  uint16_t pending = (uint16_t)log_files_to_upload;
  uint8_t payload[3] = { flags, (uint8_t)(pending & 0xff), (uint8_t)(pending >> 8) };
  return os_mbuf_append(ctxt->om, payload, sizeof(payload)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static uint8_t pull_uint8_by_tag(int tag) {
  switch (tag) {
    case TAG_LAEQ_1S:    return v_laeq_1s;
    case TAG_LAEQ_1M:    return audio_dsp_get_laeq_1m(NULL);
    case TAG_LCEQ_1M:    return audio_dsp_get_lceq_1m(NULL);
    case TAG_LAFMAX_1M:  return audio_dsp_get_lafmax_1m(NULL);
    case TAG_LCPEAK_1M:  return audio_dsp_get_lcpeak_1m(NULL);
    default: return 0;
  }
}

static int read_uint8(
    uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg
) {
  uint8_t v = pull_uint8_by_tag((int)(uintptr_t)arg);
  return os_mbuf_append(ctxt->om, &v, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int read_uint8_with_seconds(
    uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg
) {
  int tag = (int)(uintptr_t)arg;
  uint16_t seconds = 0;
  uint8_t v = 0;
  switch (tag) {
    case TAG_LAEQ_15M:   v = audio_dsp_get_laeq_15m(&seconds); break;
    case TAG_LAEQ_30M:   v = audio_dsp_get_laeq_30m(&seconds); break;
    case TAG_LCEQ_15M:   v = audio_dsp_get_lceq_15m(&seconds); break;
  }
  uint8_t payload[3] = { v, (uint8_t)(seconds & 0xff), (uint8_t)(seconds >> 8) };
  return os_mbuf_append(ctxt->om, payload, sizeof(payload)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int read_battery(
    uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg
) {
  uint16_t mv = (uint16_t)battery_voltage;
  uint8_t payload[2] = { (uint8_t)(mv & 0xff), (uint8_t)(mv >> 8) };
  return os_mbuf_append(ctxt->om, payload, sizeof(payload)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int read_uptime(
    uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg
) {
  uint32_t secs = (uint32_t)(esp_timer_get_time() / 1000000ULL);
  uint8_t payload[4] = {
      (uint8_t)(secs & 0xff), (uint8_t)((secs >> 8) & 0xff),
      (uint8_t)((secs >> 16) & 0xff), (uint8_t)((secs >> 24) & 0xff)
  };
  return os_mbuf_append(ctxt->om, payload, sizeof(payload)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int read_cal_offset(
    uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt* ctxt, void* arg
) {
  int16_t v = (int16_t)calibration_offset_x100();
  uint8_t payload[2] = { (uint8_t)(v & 0xff), (uint8_t)((v >> 8) & 0xff) };
  return os_mbuf_append(ctxt->om, payload, sizeof(payload)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// --- GAP --------------------------------------------------------------------
static uint8_t own_addr_type;
static char    adv_name[24];

static int gap_event(struct ble_gap_event* ev, void* arg);

static void start_advertising(void) {
  // Primary advertisement: flags + complete name. The 17-char name plus the
  // 3-byte flags field leaves no room for a 128-bit UUID (18 more bytes;
  // 31-byte cap on primary adv). Put the UUID in the scan response packet
  // instead, which is the standard pattern for named peripherals.
  struct ble_hs_adv_fields adv_fields;
  memset(&adv_fields, 0, sizeof(adv_fields));
  adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  adv_fields.name = (uint8_t*)adv_name;
  adv_fields.name_len = strlen(adv_name);
  adv_fields.name_is_complete = 1;
  int rc = ble_gap_adv_set_fields(&adv_fields);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_adv_set_fields: %d", rc);
    return;
  }

  // Scan response: the Custom Noise Service UUID, so phone apps can filter
  // on it without needing to connect first.
  struct ble_hs_adv_fields rsp_fields;
  memset(&rsp_fields, 0, sizeof(rsp_fields));
  rsp_fields.uuids128 = (ble_uuid128_t*)&noise_svc_uuid;
  rsp_fields.num_uuids128 = 1;
  rsp_fields.uuids128_is_complete = 1;
  rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields: %d", rc);
    return;
  }

  struct ble_gap_adv_params adv_params = {
      .conn_mode = BLE_GAP_CONN_MODE_UND,
      .disc_mode = BLE_GAP_DISC_MODE_GEN,
      .itvl_min = BLE_GAP_ADV_ITVL_MS(1000),
      .itvl_max = BLE_GAP_ADV_ITVL_MS(1000),
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
      for (size_t i = 0; i < sizeof(subs)/sizeof(subs[0]); i++) subs[i].subscribed = false;
      start_advertising();
      break;
    case BLE_GAP_EVENT_SUBSCRIBE: {
      uint16_t h = ev->subscribe.attr_handle;
      bool en = ev->subscribe.cur_notify != 0;
      for (size_t i = 0; i < sizeof(subs)/sizeof(subs[0]); i++) {
        if (*subs[i].h == h) { subs[i].subscribed = en; break; }
      }
      break;
    }
    case BLE_GAP_EVENT_MTU:
      ESP_LOGI(TAG, "MTU updated to %d", ev->mtu.value);
      break;
    default:
      break;
  }
  return 0;
}

static bool is_subscribed(uint16_t h) {
  for (size_t i = 0; i < sizeof(subs)/sizeof(subs[0]); i++) {
    if (*subs[i].h == h) return subs[i].subscribed;
  }
  return false;
}

static void notify_uint8(uint16_t h, uint8_t v) {
  if (curr_conn == 0xffff || !is_subscribed(h)) return;
  struct os_mbuf* om = ble_hs_mbuf_from_flat(&v, 1);
  if (om) ble_gatts_notify_custom(curr_conn, h, om);
}

static void notify_uint8_with_seconds(uint16_t h, uint8_t v, uint16_t seconds) {
  if (curr_conn == 0xffff || !is_subscribed(h)) return;
  uint8_t payload[3] = { v, (uint8_t)(seconds & 0xff), (uint8_t)(seconds >> 8) };
  struct os_mbuf* om = ble_hs_mbuf_from_flat(payload, sizeof(payload));
  if (om) ble_gatts_notify_custom(curr_conn, h, om);
}

static void notify_status(uint8_t flags, uint16_t pending) {
  if (curr_conn == 0xffff || !is_subscribed(h_status)) return;
  uint8_t payload[3] = { flags, (uint8_t)(pending & 0xff), (uint8_t)(pending >> 8) };
  struct os_mbuf* om = ble_hs_mbuf_from_flat(payload, sizeof(payload));
  if (om) ble_gatts_notify_custom(curr_conn, h_status, om);
}

static void notify_battery(uint16_t mv) {
  if (curr_conn == 0xffff || !is_subscribed(h_battery)) return;
  uint8_t payload[2] = { (uint8_t)(mv & 0xff), (uint8_t)(mv >> 8) };
  struct os_mbuf* om = ble_hs_mbuf_from_flat(payload, sizeof(payload));
  if (om) ble_gatts_notify_custom(curr_conn, h_battery, om);
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

  // Drain loop: 1 Hz per-record updates + change/period checks for slow chars.
  while (true) {
    record_t r;
    if (xQueueReceive(ble_publisher_queue, &r, pdMS_TO_TICKS(1000)) == pdTRUE) {
      v_laeq_1s    = r.laeq_1s;
      v_lceq_1s    = r.lceq_1s;
      v_lafmax_1s  = r.lafmax_1s;
      v_lcfmax_1s  = r.lcfmax_1s;
      v_lcpeak_1s  = r.lcpeak_1s;

      notify_uint8(h_laeq_1s,    r.laeq_1s);
      notify_uint8(h_laeq_1m,    audio_dsp_get_laeq_1m(NULL));
      notify_uint8(h_lceq_1m,    audio_dsp_get_lceq_1m(NULL));
      notify_uint8(h_lafmax_1m,  audio_dsp_get_lafmax_1m(NULL));
      notify_uint8(h_lcpeak_1m,  audio_dsp_get_lcpeak_1m(NULL));
    }

    int64_t now = esp_timer_get_time();

    // Status: notify on flag change OR pending change OR every 10 s.
    EventBits_t bits = xEventGroupGetBits(event_group);
    uint8_t flags = ((bits & WIFI_CONNECTED) ? 0x01 : 0)
                  | ((bits & USB_CONNECTED)  ? 0x02 : 0)
                  | ((bits & CALIBRATED)     ? 0x04 : 0)
                  | ((bits & TIME_SET)       ? 0x08 : 0);
    uint16_t pending = (uint16_t)log_files_to_upload;
    bool changed = flags != last_status_flags || pending != last_status_pending;
    if (changed || (now - last_status_us) >= 10 * 1000000) {
      notify_status(flags, pending);
      last_status_flags = flags;
      last_status_pending = pending;
      last_status_us = now;
    }

    // Battery: ≥50 mV change OR every 30 s.
    uint16_t mv = (uint16_t)battery_voltage;
    int dv = (int)mv - (int)last_battery_mv;
    if (dv < 0) dv = -dv;
    if (dv >= 50 || (now - last_battery_us) >= 30 * 1000000) {
      notify_battery(mv);
      last_battery_mv = mv;
      last_battery_us = now;
    }

    // LAeq,15m / LAeq,30m / LCeq,15m: ≥0.5 dB change OR every 30 s.
    // 0.5 dB in encoded uint8 is exactly 1 LSB.
    uint16_t s15 = 0, s30 = 0, sc15 = 0;
    uint8_t laeq_15m = audio_dsp_get_laeq_15m(&s15);
    uint8_t laeq_30m = audio_dsp_get_laeq_30m(&s30);
    uint8_t lceq_15m = audio_dsp_get_lceq_15m(&sc15);

    if (laeq_15m != last_laeq_15m || (now - last_laeq_15m_us) >= 30 * 1000000) {
      notify_uint8_with_seconds(h_laeq_15m, laeq_15m, s15);
      last_laeq_15m = laeq_15m;
      last_laeq_15m_us = now;
    }
    if (laeq_30m != last_laeq_30m || (now - last_laeq_30m_us) >= 30 * 1000000) {
      notify_uint8_with_seconds(h_laeq_30m, laeq_30m, s30);
      last_laeq_30m = laeq_30m;
      last_laeq_30m_us = now;
    }
    if (lceq_15m != last_lceq_15m || (now - last_lceq_15m_us) >= 30 * 1000000) {
      notify_uint8_with_seconds(h_lceq_15m, lceq_15m, sc15);
      last_lceq_15m = lceq_15m;
      last_lceq_15m_us = now;
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
