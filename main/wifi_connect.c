#include "wifi_connect.h"
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
int8_t wifi_rssi = INT8_MIN;
wifi_status_t wifi_status = DISCONNECTED;

static void update_signal_strength() {
  wifi_ap_record_t wifidata;
  esp_wifi_sta_get_ap_info(&wifidata);
  wifi_rssi = wifidata.rssi;
  xEventGroupSetBits(event_group, DISPLAY_NEEDS_UPDATE);
}

static int timer_duration() {
  bool is_connected = xEventGroupGetBits(event_group) & WIFI_CONNECTED;
  bool usb_connected = xEventGroupGetBits(event_group) & USB_CONNECTED;
  if (is_connected) {
    return 10000;
  } else if (usb_connected) {
    return 60000 * 2;
  } else {
    return 60000 * 5;
  }
}

static void timer_cb(TimerHandle_t timer) {
  bool is_connected = xEventGroupGetBits(event_group) & WIFI_CONNECTED;
  if (is_connected) {
    update_signal_strength();
  } else {
    vTaskNotifyGiveFromISR(xTaskGetHandle(WIFI_CONNECT_TASK), NULL);
  }
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
  clearTimer();
  update_timer =
      xTimerCreate("update_timer", pdMS_TO_TICKS(timer_duration()), pdFALSE, 0, timer_cb);
  xTimerStart(update_timer, 0);
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
    // Spec §12 wanted modem-sleep (WIFI_PS_MIN_MODEM) for ~50 mA savings,
    // but on the iPhone Personal Hotspot we used for development the hotspot
    // drops idle clients after ~10 s if our radio is dozing. Keeping the
    // radio fully awake (WIFI_PS_NONE) is the workaround. Revisit when
    // running against a real router on battery.
    esp_wifi_set_ps(WIFI_PS_NONE);
    update_signal_strength();
    xEventGroupSetBits(event_group, WIFI_CONNECTED);
    xTaskNotify(xTaskGetHandle(LOG_UPLOADER_TASK), 0, eNoAction);
    startTimer();
  }
  xEventGroupSetBits(event_group, DISPLAY_NEEDS_UPDATE);
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
    xEventGroupSetBits(event_group, DISPLAY_NEEDS_UPDATE);
    clearTimer();
    // Bring the radio back up if a previous disconnect stopped it (§12).
    esp_wifi_start();
    esp_wifi_connect();
  }
}
