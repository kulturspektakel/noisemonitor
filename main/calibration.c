#include "calibration.h"
#include <stdatomic.h>
#include <string.h>
#include "constants.h"
#include "esp_log.h"
#include "event_group.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"

static const char* TAG = "calibration";

// Cached per-band calibration in 0.5 dB steps. All-zero = no correction.
static int8_t band_steps[CALIBRATION_BANDS];
// Set whenever band_steps changes; the DSP task tests-and-clears it to re-fold
// the calibration into its per-bin weights.
static atomic_bool band_dirty = ATOMIC_VAR_INIT(false);

void calibration_init(void) {
  memset(band_steps, 0, sizeof(band_steps));

  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NOISE_CAL, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "nvs_open(%s) failed: %s; device UNCALIBRATED",
             NVS_NOISE_CAL, esp_err_to_name(err));
    return;
  }

  // Drop the obsolete scalar-offset key written by older firmware.
  nvs_erase_key(handle, NVS_NOISE_CAL_OFFSET);

  size_t len = sizeof(band_steps);
  err = nvs_get_blob(handle, NVS_NOISE_CAL_BANDS, band_steps, &len);
  nvs_commit(handle);
  nvs_close(handle);

  if (err == ESP_OK && len == sizeof(band_steps)) {
    xEventGroupSetBits(event_group, CALIBRATED);
    atomic_store(&band_dirty, true);
    ESP_LOGI(TAG, "per-band calibration loaded (%d bands)", CALIBRATION_BANDS);
    return;
  }

  // Anything else (missing key, wrong size, read error) = uncalibrated.
  memset(band_steps, 0, sizeof(band_steps));
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(TAG, "device is UNCALIBRATED");
  } else {
    ESP_LOGW(TAG, "nvs_get_blob failed (%s); device UNCALIBRATED",
             esp_err_to_name(err));
  }
}

esp_err_t calibration_set_bands(const int8_t* steps, size_t n) {
  if (n != CALIBRATION_BANDS) return ESP_ERR_INVALID_ARG;

  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NOISE_CAL, NVS_READWRITE, &handle);
  if (err != ESP_OK) return err;
  err = nvs_set_blob(handle, NVS_NOISE_CAL_BANDS, steps, n);
  if (err == ESP_OK) err = nvs_commit(handle);
  nvs_close(handle);
  if (err != ESP_OK) return err;

  memcpy(band_steps, steps, n);
  atomic_store(&band_dirty, true);
  xEventGroupSetBits(event_group, CALIBRATED);
  return ESP_OK;
}

esp_err_t calibration_clear(void) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NOISE_CAL, NVS_READWRITE, &handle);
  if (err != ESP_OK) return err;
  err = nvs_erase_key(handle, NVS_NOISE_CAL_BANDS);
  if (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND) {
    nvs_commit(handle);
    err = ESP_OK;
  }
  nvs_close(handle);

  memset(band_steps, 0, sizeof(band_steps));
  atomic_store(&band_dirty, true);
  xEventGroupClearBits(event_group, CALIBRATED);
  return err;
}

void calibration_get_bands(int8_t* out, size_t n) {
  if (n > CALIBRATION_BANDS) n = CALIBRATION_BANDS;
  memcpy(out, band_steps, n);
}

void calibration_get_bands_db(float* out_db, size_t n) {
  if (n > CALIBRATION_BANDS) n = CALIBRATION_BANDS;
  for (size_t i = 0; i < n; i++) out_db[i] = (float)band_steps[i] * 0.5f;
}

bool calibration_take_band_dirty(void) {
  return atomic_exchange(&band_dirty, false);
}
