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

_Atomic wifi_status_t wifi_status = DISCONNECTED;

#define WIFI_RETRY_BASE_MS 5000
#define WIFI_RETRY_MAX_MS  (10 * 60 * 1000)
#define WIFI_STATE_CHANGED (1U << 0)
#define WIFI_RETRY_NOW (1U << 1)
#define NVS_WIFI_CREDENTIALS "wifi_creds"
#define WIFI_CREDENTIALS_VERSION 1

typedef struct {
  uint8_t version;
  char ssid[33];
  char password[64];
} wifi_credentials_t;

static TickType_t retry_timeout(int* retry_attempt) {
  int delay_ms = WIFI_RETRY_BASE_MS;
  for (int attempt = 0; attempt < *retry_attempt && delay_ms < WIFI_RETRY_MAX_MS; attempt++) {
    delay_ms *= 2;
  }
  if (delay_ms > WIFI_RETRY_MAX_MS) delay_ms = WIFI_RETRY_MAX_MS;
  ESP_LOGI(WIFI_CONNECT_TASK, "WiFi retry #%d in %d ms", *retry_attempt + 1, delay_ms);
  if (*retry_attempt < 16) (*retry_attempt)++;
  return pdMS_TO_TICKS(delay_ms);
}

static void event_handler(
    void* arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void* event_data
) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    ESP_LOGI(WIFI_CONNECT_TASK, "WiFi disconnected");
    xEventGroupClearBits(event_group, WIFI_CONNECTED);
    xTaskNotify((TaskHandle_t)arg, WIFI_STATE_CHANGED, eSetBits);
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ESP_LOGI(WIFI_CONNECT_TASK, "WiFi connected");
    // Modem-sleep (WIFI_PS_MIN_MODEM) per spec §12 — large WiFi power saving
    // when idle. Past concern: iPhone Personal Hotspot used in development
    // drops idle clients after ~10 s when our radio is dozing. If reconnect
    // churn returns, fall back to WIFI_PS_NONE (we publish to MQTT every 1 s
    // anyway, so true idle is rare on a real router).
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    xEventGroupSetBits(event_group, WIFI_CONNECTED);
    xTaskNotify((TaskHandle_t)arg, WIFI_STATE_CHANGED, eSetBits);
    TaskHandle_t uploader = xTaskGetHandle(LOG_UPLOADER_TASK);
    if (uploader != NULL) xTaskNotify(uploader, 0, eNoAction);
  }
}

static esp_err_t read_credentials(nvs_handle_t handle, wifi_credentials_t* credentials) {
  size_t size = sizeof(*credentials);
  esp_err_t err = nvs_get_blob(handle, NVS_WIFI_CREDENTIALS, credentials, &size);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    credentials->version = WIFI_CREDENTIALS_VERSION;
    size = sizeof(credentials->ssid);
    err = nvs_get_str(handle, NVS_WIFI_SSID, credentials->ssid, &size);
    if (err != ESP_OK) return err;
    size = sizeof(credentials->password);
    err = nvs_get_str(handle, NVS_WIFI_PASSWORD, credentials->password, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) credentials->password[0] = '\0';
    else if (err != ESP_OK) return err;
  } else if (err != ESP_OK) {
    return err;
  } else if (size != sizeof(*credentials)) {
    return ESP_ERR_INVALID_SIZE;
  }

  if (credentials->version != WIFI_CREDENTIALS_VERSION) return ESP_ERR_INVALID_VERSION;
  size_t ssid_len = strnlen(credentials->ssid, sizeof(credentials->ssid));
  size_t password_len = strnlen(credentials->password, sizeof(credentials->password));
  if (ssid_len == 0 || ssid_len > 32 || password_len > 63) return ESP_ERR_INVALID_ARG;
  return ESP_OK;
}

void wifi_connect(void* params) {
  wifi_config_t wifi_config = {
      .sta = {.ssid = "", .password = ""},
  };

  wifi_credentials_t credentials = {0};
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open(NVS_DEVICE_CONFIG, NVS_READONLY, &nvs_handle);
  if (err == ESP_OK) {
    err = read_credentials(nvs_handle, &credentials);
    nvs_close(nvs_handle);
  }
  if (err == ESP_OK) {
    memcpy(wifi_config.sta.ssid, credentials.ssid, strlen(credentials.ssid));
    memcpy(wifi_config.sta.password, credentials.password, sizeof(credentials.password));
  } else {
    ESP_LOGW(WIFI_CONNECT_TASK, "WiFi credentials unavailable: %s", esp_err_to_name(err));
  }

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

  ESP_LOGI(WIFI_CONNECT_TASK, "initialized with ssid=%.32s", wifi_config.sta.ssid);

  int retry_attempt = 0;
  TickType_t wait_ticks = 0;

  while (1) {
    uint32_t notifications = 0;
    bool retry_now = xTaskNotifyWait(0, UINT32_MAX, &notifications, wait_ticks) == pdFALSE;
    if (notifications & WIFI_STATE_CHANGED) {
      if (xEventGroupGetBits(event_group) & WIFI_CONNECTED) {
        wifi_status = CONNECTED;
        retry_attempt = 0;
        wait_ticks = portMAX_DELAY;
      } else {
        wifi_status = DISCONNECTED;
        esp_wifi_stop();
        wait_ticks = retry_timeout(&retry_attempt);
      }
    }
    if ((notifications & WIFI_RETRY_NOW) && wifi_status == DISCONNECTED) {
      retry_attempt = 0;
      retry_now = true;
    }
    if (!retry_now || wifi_status != DISCONNECTED) continue;

    ESP_LOGI(WIFI_CONNECT_TASK, "Trying to connect to WiFi...");
    wifi_status = CONNECTING;
    wait_ticks = portMAX_DELAY;
    err = esp_wifi_start();
    if (err == ESP_OK) err = esp_wifi_connect();
    if (err != ESP_OK) {
      ESP_LOGW(WIFI_CONNECT_TASK, "WiFi connect failed: %s", esp_err_to_name(err));
      wifi_status = DISCONNECTED;
      esp_wifi_stop();
      wait_ticks = retry_timeout(&retry_attempt);
    }
  }
}

void wifi_connect_trigger(void) {
  TaskHandle_t task = xTaskGetHandle(WIFI_CONNECT_TASK);
  if (task != NULL) xTaskNotify(task, WIFI_RETRY_NOW, eSetBits);
}

esp_err_t wifi_connect_set_credentials(const char* ssid, const char* password) {
  if (ssid == NULL || password == NULL) return ESP_ERR_INVALID_ARG;
  size_t ssid_len = strnlen(ssid, 33);
  size_t pw_len   = strnlen(password, 64);
  if (ssid_len == 0 || ssid_len > 32 || pw_len > 63) return ESP_ERR_INVALID_ARG;

  wifi_credentials_t credentials = {.version = WIFI_CREDENTIALS_VERSION};
  memcpy(credentials.ssid, ssid, ssid_len);
  memcpy(credentials.password, password, pw_len);

  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_DEVICE_CONFIG, NVS_READWRITE, &handle);
  if (err != ESP_OK) return err;
  err = nvs_set_blob(handle, NVS_WIFI_CREDENTIALS, &credentials, sizeof(credentials));
  if (err == ESP_OK) err = nvs_commit(handle);
  nvs_close(handle);
  if (err != ESP_OK) return err;

  ESP_LOGI(WIFI_CONNECT_TASK, "new credentials stored; rebooting");
  vTaskDelay(pdMS_TO_TICKS(200));
  esp_restart();
  return ESP_OK;
}
