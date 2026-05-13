#include "mqtt_publisher.h"
#include <string.h>
#include "audio_dsp.h"
#include "calibration.h"
#include "constants.h"
#include "esp_log.h"
#include "event_group.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "mqtt_client.h"
#include "nanopb/pb_encode.h"
#include "noise.pb.h"

static const char* TAG = "mqtt_publisher";

// Dev broker per spec §9. Plain MQTT (no TLS) for development against the
// public sandbox broker: mbedtls SSL session buffers don't fit in the
// fragmented heap we have left after BLE+audio_dsp init (allocation fails
// with MBEDTLS_ERR_SSL_ALLOC_FAILED). For production against a real broker
// switch to mqtts:// once the in/out content lengths have been tuned and
// the actual heap budget verified.
#define MQTT_URI "mqtt://broker.emqx.io:1883"

static esp_mqtt_client_handle_t mqtt_client = NULL;
static volatile bool mqtt_connected = false;

static char topic[64];

static NoiseRecording publish_buf;
static uint8_t encoded_buf[256];   // single Record is < 50 bytes encoded; 256 is generous

static void mqtt_event_handler(
    void* arg, esp_event_base_t base, int32_t event_id, void* event_data
) {
  (void)arg; (void)base; (void)event_data;
  switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
      ESP_LOGI(TAG, "MQTT connected");
      mqtt_connected = true;
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
  // Wait for WiFi + device_id to compose the topic.
  xEventGroupWaitBits(event_group, WIFI_CONNECTED | DEVICE_ID_LOADED, false, true, portMAX_DELAY);

  snprintf(topic, sizeof(topic), "noise/%s/record", DEVICE_ID);
  ESP_LOGI(TAG, "MQTT topic: %s", topic);

  esp_mqtt_client_config_t cfg = {
      .broker.address.uri = MQTT_URI,
      .broker.verification.skip_cert_common_name_check = true,
  };
  mqtt_client = esp_mqtt_client_init(&cfg);
  esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
  esp_mqtt_client_start(mqtt_client);

  while (true) {
    record_t r;
    if (xQueueReceive(mqtt_publisher_queue, &r, portMAX_DELAY) != pdTRUE) continue;

    bool wifi_ok = (xEventGroupGetBits(event_group) & WIFI_CONNECTED) != 0;
    if (!wifi_ok || !mqtt_connected) continue;  // drop silently per §9

    publish_buf.calibration_offset_db_x100 = calibration_offset_x100();
    publish_buf.records_count = 1;
    publish_buf.records[0].seq_no    = r.seq_no;
    memcpy(publish_buf.records[0].bands, r.bands, sizeof(r.bands));
    publish_buf.records[0].laeq_1s   = r.laeq_1s;
    publish_buf.records[0].lceq_1s   = r.lceq_1s;
    publish_buf.records[0].lafmax_1s = r.lafmax_1s;
    publish_buf.records[0].lcfmax_1s = r.lcfmax_1s;
    publish_buf.records[0].lcpeak_1s = r.lcpeak_1s;

    pb_ostream_t stream = pb_ostream_from_buffer(encoded_buf, sizeof(encoded_buf));
    if (!pb_encode(&stream, NoiseRecording_fields, &publish_buf)) {
      ESP_LOGW(TAG, "encode failed: %s", stream.errmsg);
      continue;
    }

    int msg_id = esp_mqtt_client_publish(
        mqtt_client, topic, (const char*)encoded_buf, stream.bytes_written, 0, 0
    );
    if (msg_id < 0) {
      ESP_LOGW(TAG, "publish failed");
    }
  }
}
