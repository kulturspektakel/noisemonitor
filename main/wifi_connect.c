#include "wifi_connect.h"
#include <string.h>
#include "constants.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "event_group.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "power_management.h"

static TimerHandle_t update_timer;
wifi_status_t wifi_status = DISCONNECTED;

// Retry cadence while disconnected: exponential backoff from 5 s, doubling to
// a 10 min ceiling (5, 10, 20, 40, 80, 160, 320, 600, 600...). The timer only
// runs while disconnected — disconnects are event-driven — so there is no
// connected-state case to handle here.
//
// Was a flat 2 min (USB) / 5 min (battery). That interval was sized for a
// genuinely absent AP, but it also governed the far more common case, a
// transient association failure at boot, and made it very expensive: measured
// repeatedly on 2026-07-30/31, the first attempt failed at t≈2.5 s and the
// retry that succeeded — on its first try — did not fire until t≈122 s. The
// device sat offline for two minutes, dropping ~120 live records, for a fault
// that clears in seconds.
//
// The ceiling is now the same on USB and battery. That is better than before
// on battery (10 min vs 5 min between radio wake-ups) and worse on USB (10 min
// vs 2 min to recover from a long outage). Note a long ceiling delays but does
// not lose file data — record_writer keeps writing locally and log_uploader
// drains the backlog on reconnect; only live MQTT records are dropped. To make
// the ceiling power-dependent again, branch on USB_CONNECTED here.
#define WIFI_RETRY_BASE_MS 5000
#define WIFI_RETRY_MAX_MS  (10 * 60 * 1000)

// Consecutive failed attempts since the last successful association.
static int retry_attempt = 0;

static int timer_duration() {
  // Loop rather than shift: caps without ever overflowing, however long the
  // outage runs.
  int ms = WIFI_RETRY_BASE_MS;
  for (int i = 0; i < retry_attempt && ms < WIFI_RETRY_MAX_MS; i++) {
    ms *= 2;
  }
  return ms > WIFI_RETRY_MAX_MS ? WIFI_RETRY_MAX_MS : ms;
}

static void timer_cb(TimerHandle_t timer) {
  bool is_connected = xEventGroupGetBits(event_group) & WIFI_CONNECTED;
  if (is_connected) {
    // A retry succeeded between scheduling and firing — stop; the next
    // disconnect event will restart the timer.
    return;
  }
  vTaskNotifyGiveFromISR(xTaskGetHandle(WIFI_CONNECT_TASK), NULL);
  xTimerChangePeriod(timer, pdMS_TO_TICKS(timer_duration()), 0);
  xTimerReset(timer, 0);
}

static void clearTimer() {
  if (update_timer != NULL) {
    xTimerDelete(update_timer, 0);
    update_timer = NULL;
  }
}

static void startTimer() {
  int delay_ms = timer_duration();
  clearTimer();
  update_timer = xTimerCreate("update_timer", pdMS_TO_TICKS(delay_ms), pdFALSE, 0, timer_cb);
  xTimerStart(update_timer, 0);
  ESP_LOGI(WIFI_CONNECT_TASK, "WiFi retry #%d in %d ms", retry_attempt + 1, delay_ms);
  // Saturate: timer_duration() has already capped, this just keeps the counter
  // from growing without bound across a multi-day outage.
  if (retry_attempt < 16) retry_attempt++;
}

static void event_handler(
    void* arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void* event_data
) {
  // Intentionally no WIFI_EVENT_STA_START handler: the wifi_connect task
  // self-notifies once after esp_wifi_start() in init, and the retry timer
  // notifies on later retries. Letting the event handler also notify on
  // every STA_START causes a double-dispatch on the retry path (esp_wifi_start
  // in the retry body re-fires STA_START), producing `sta is connecting,
  // return error` in the log.
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    ESP_LOGI(WIFI_CONNECT_TASK, "WiFi disconnected");
    wifi_status = DISCONNECTED;
    xEventGroupClearBits(event_group, WIFI_CONNECTED);
    // Power down the radio between retries (§12). The retry path will
    // call esp_wifi_start() again.
    esp_wifi_stop();
    startTimer();
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ESP_LOGI(WIFI_CONNECT_TASK, "WiFi connected");
    wifi_status = CONNECTED;
    retry_attempt = 0;  // next outage starts the backoff over at 5 s
    // Modem-sleep (WIFI_PS_MIN_MODEM) per spec §12 — large WiFi power saving
    // when idle. Past concern: iPhone Personal Hotspot used in development
    // drops idle clients after ~10 s when our radio is dozing. If reconnect
    // churn returns, fall back to WIFI_PS_NONE (we publish to MQTT every 1 s
    // anyway, so true idle is rare on a real router).
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    xEventGroupSetBits(event_group, WIFI_CONNECTED);
    xTaskNotify(xTaskGetHandle(LOG_UPLOADER_TASK), 0, eNoAction);
    // No timer while connected: disconnects are event-driven and restart it.
  }
}

static esp_err_t read_nvs_string(
    nvs_handle_t handle,
    const char* key,
    char* target,
    size_t max_size
) {
  size_t required_size = 0;
  esp_err_t err = nvs_get_str(handle, key, NULL, &required_size);

  if (err == ESP_OK && required_size <= max_size) {
    err = nvs_get_str(handle, key, target, &required_size);
  } else {
    ESP_LOGE(WIFI_CONNECT_TASK, "Failed to get %s", key);
  }

  return err;
}

void wifi_connect(void* params) {
  wifi_config_t wifi_config = {
      .sta = {.ssid = "", .password = ""},
  };

  nvs_handle_t nvs_handle;
  ESP_ERROR_CHECK(nvs_open(NVS_DEVICE_CONFIG, NVS_READONLY, &nvs_handle));
  read_nvs_string(
      nvs_handle, NVS_WIFI_SSID, (char*)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid)
  );
  read_nvs_string(
      nvs_handle,
      NVS_WIFI_PASSWORD,
      (char*)wifi_config.sta.password,
      sizeof(wifi_config.sta.password)
  );
  nvs_close(nvs_handle);

  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
  esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, current_task, &instance_any_id
  );
  esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, current_task, &instance_got_ip
  );

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  esp_wifi_start();

  ESP_LOGI(WIFI_CONNECT_TASK, "initialized with ssid=%s", wifi_config.sta.ssid);

  // Drive the loop body once at startup. After that, only the retry timer
  // notifies us (the STA_START event is intentionally not wired in).
  xTaskNotifyGive(xTaskGetCurrentTaskHandle());

  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESP_LOGI(WIFI_CONNECT_TASK, "Trying to connect to WiFi...");
    wifi_status = CONNECTING;
    clearTimer();
    // Bring the radio back up if a previous disconnect stopped it (§12).
    esp_wifi_start();
    esp_wifi_connect();
  }
}

void wifi_connect_trigger(void) {
  // Skip if already connected or mid-connect: notifying during CONNECTING
  // re-enters esp_wifi_connect() and the stack logs "sta is connecting,
  // return error" (same reason the retry timer only notifies when down).
  if (wifi_status == CONNECTED || wifi_status == CONNECTING) return;
  // An explicit trigger (BLE provisioning, USB plugged in) means something
  // changed that could fix the connection, so don't make the user wait out a
  // backoff earned by earlier failures.
  retry_attempt = 0;
  TaskHandle_t h = xTaskGetHandle(WIFI_CONNECT_TASK);
  if (h != NULL) xTaskNotifyGive(h);
}

esp_err_t wifi_connect_set_credentials(const char* ssid, const char* password) {
  if (ssid == NULL || password == NULL) return ESP_ERR_INVALID_ARG;
  size_t ssid_len = strnlen(ssid, 33);
  size_t pw_len   = strnlen(password, 64);
  if (ssid_len == 0 || ssid_len > 32 || pw_len > 63) return ESP_ERR_INVALID_ARG;

  nvs_handle_t h;
  esp_err_t err = nvs_open(NVS_DEVICE_CONFIG, NVS_READWRITE, &h);
  if (err != ESP_OK) return err;
  err = nvs_set_str(h, NVS_WIFI_SSID, ssid);
  if (err == ESP_OK) err = nvs_set_str(h, NVS_WIFI_PASSWORD, password);
  if (err == ESP_OK) err = nvs_commit(h);
  nvs_close(h);
  if (err != ESP_OK) return err;

  // Reboot to pick up the new creds. Reconnecting in-place is doable but
  // requires re-applying wifi_config_t, overriding the retry timer, and
  // dealing with the disconnect/connect event ordering — way more code
  // surface for a path that runs maybe once per festival setup.
  ESP_LOGI(WIFI_CONNECT_TASK, "new credentials stored; rebooting");
  vTaskDelay(pdMS_TO_TICKS(200));  // give the BLE write-response a moment to land
  esp_restart();
  return ESP_OK;  // unreachable
}
