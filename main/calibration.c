#include "calibration.h"
#include "constants.h"
#include "esp_log.h"
#include "event_group.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"

static const char* TAG = "calibration";

static float   offset_db   = 0.0f;
static int32_t offset_x100 = 0;

void calibration_init(void) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NOISE_CAL, NVS_READONLY, &handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(TAG, "no calibration namespace; device is UNCALIBRATED");
    return;
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs_open(%s) failed: %s", NVS_NOISE_CAL, esp_err_to_name(err));
    return;
  }
  int32_t v = 0;
  err = nvs_get_i32(handle, NVS_NOISE_CAL_OFFSET, &v);
  nvs_close(handle);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(TAG, "device is UNCALIBRATED");
    return;
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs_get_i32 failed: %s", esp_err_to_name(err));
    return;
  }
  offset_x100 = v;
  offset_db = v / 100.0f;
  xEventGroupSetBits(event_group, CALIBRATED);
  ESP_LOGI(TAG, "calibration loaded: offset=%+.2f dB", offset_db);
}

esp_err_t calibration_set(int32_t offset_db_x100) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NOISE_CAL, NVS_READWRITE, &handle);
  if (err != ESP_OK) return err;
  err = nvs_set_i32(handle, NVS_NOISE_CAL_OFFSET, offset_db_x100);
  if (err == ESP_OK) err = nvs_commit(handle);
  nvs_close(handle);
  if (err != ESP_OK) return err;

  offset_x100 = offset_db_x100;
  offset_db = offset_db_x100 / 100.0f;
  xEventGroupSetBits(event_group, CALIBRATED);
  return ESP_OK;
}

esp_err_t calibration_clear(void) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NOISE_CAL, NVS_READWRITE, &handle);
  if (err != ESP_OK) return err;
  err = nvs_erase_key(handle, NVS_NOISE_CAL_OFFSET);
  if (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND) {
    nvs_commit(handle);
    err = ESP_OK;
  }
  nvs_close(handle);

  offset_x100 = 0;
  offset_db = 0.0f;
  xEventGroupClearBits(event_group, CALIBRATED);
  return err;
}

float calibration_offset_db(void) {
  return offset_db;
}

int32_t calibration_offset_x100(void) {
  return offset_x100;
}

