#pragma once

#include <stdint.h>
#include "esp_err.h"

// Read NVS, populate the in-RAM offset, set CALIBRATED event bit if present.
void calibration_init(void);

// Persist offset to NVS, update in-RAM value, set CALIBRATED bit.
esp_err_t calibration_set(int32_t offset_db_x100);

// Erase NVS key, zero offset, clear CALIBRATED bit.
esp_err_t calibration_clear(void);

// Returns the current offset in dB (0.0 while uncalibrated).
float calibration_offset_db(void);

// Returns the current offset in 1/100 dB units (the persisted form).
// Useful for BLE/MQTT consumers that want to avoid float math.
int32_t calibration_offset_x100(void);
