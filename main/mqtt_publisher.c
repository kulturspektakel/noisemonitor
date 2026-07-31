#include "mqtt_publisher.h"
#include "audio_dsp.h"
#include "constants.h"
#include "esp_log.h"
#include "event_group.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "heap_diag.h"
#include "mqtt_client.h"

static const char* TAG = "mqtt_publisher";

// Dev broker per spec §9: plaintext MQTT to a public sandbox broker. NOTE this
// is unauthenticated and unencrypted — anyone can read/inject on this topic.
// The old blocker for mqtts:// (TLS buffers wouldn't fit in the fragmented
// internal heap) is gone post-PSRAM: the log uploader already runs a TLS
// session fine, and TLS buffers now allocate from PSRAM. Switching to
// production is just pointing MQTT_URI at the real broker (mqtts://) and adding
// credentials — see the coexistence notes in PSRAM_MIGRATION.md.
#define MQTT_URI "mqtt://broker.emqx.io:1883"

static esp_mqtt_client_handle_t mqtt_client = NULL;
static volatile bool mqtt_connected = false;

static char topic[64];

static uint8_t encoded_buf[256];   // single Record is < 50 bytes encoded; 256 is generous

static void mqtt_event_handler(
    void* arg, esp_event_base_t base, int32_t event_id, void* event_data
) {
  (void)arg; (void)base; (void)event_data;
  switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
      ESP_LOGI(TAG, "MQTT connected");
      mqtt_connected = true;
      // Opportunistic kick for log_uploader: MQTT coming up is a decent proxy
      // for "the network is healthy". Not required — wifi_connect notifies on
      // link-up, which is the uploader's only real precondition.
      {
        TaskHandle_t h = xTaskGetHandle(LOG_UPLOADER_TASK);
        if (h) xTaskNotify(h, 0, eNoAction);
      }
      break;
    case MQTT_EVENT_DISCONNECTED:
      ESP_LOGI(TAG, "MQTT disconnected");
      mqtt_connected = false;
      break;
    case MQTT_EVENT_ERROR:
      ESP_LOGW(TAG, "MQTT error");
      break;
    default:
      break;
  }
}

void mqtt_publisher(void* params) {
  xEventGroupWaitBits(event_group, WIFI_CONNECTED | DEVICE_ID_LOADED, false, true, portMAX_DELAY);
  snprintf(topic, sizeof(topic), "noise/%s/record", DEVICE_ID);
  ESP_LOGI(TAG, "MQTT topic: %s", topic);

  esp_mqtt_client_config_t cfg = {
      .broker.address.uri = MQTT_URI,
      .broker.verification.skip_cert_common_name_check = true,
  };
  mqtt_client = esp_mqtt_client_init(&cfg);
  esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

  // esp_mqtt_client_start() spawns a task whose 6144 B stack must be contiguous
  // internal RAM, so it can fail under memory pressure. Retrying the same handle
  // is safe: on xTaskCreate failure esp-mqtt allocates nothing and leaves state
  // at MQTT_STATE_INIT. Retry from inside the drain loop rather than a blocking
  // prologue — audio_dsp enqueues a record every second, so a publisher that
  // stops receiving overflows the 16-deep queue.
  bool started = false;
  TickType_t next_attempt = 0;

  while (true) {
    if (!started && xTaskGetTickCount() >= next_attempt) {
      esp_err_t err = esp_mqtt_client_start(mqtt_client);
      if (err == ESP_OK) {
        started = true;
      } else {
        ESP_LOGW(TAG, "mqtt start failed (%s); retrying in 10 s", esp_err_to_name(err));
        heap_diag("mqtt start failed");
        next_attempt = xTaskGetTickCount() + pdMS_TO_TICKS(10000);
      }
    }

    record_t r;
    TickType_t wait = started ? portMAX_DELAY : pdMS_TO_TICKS(1000);
    if (xQueueReceive(mqtt_publisher_queue, &r, wait) != pdTRUE) continue;

    // mqtt_connected implies started — MQTT_EVENT_CONNECTED can't fire otherwise.
    bool wifi_ok = (xEventGroupGetBits(event_group) & WIFI_CONNECTED) != 0;
    if (!wifi_ok || !mqtt_connected) continue;  // drop silently per §9

    size_t n = record_encode_single(&r, encoded_buf, sizeof(encoded_buf));
    if (n == 0) continue;

    int msg_id = esp_mqtt_client_publish(
        mqtt_client, topic, (const char*)encoded_buf, n, 0, 0
    );
    if (msg_id < 0) {
      ESP_LOGW(TAG, "publish failed");
    } else {
      ESP_LOGI(TAG, "published seq=%lu LAeq=%.1f LCpeak=%.1f",
               (unsigned long)r.seq_no,
               20.0f + r.laeq / 2.0f,
               20.0f + r.lcpeak / 2.0f);
    }
  }
}
