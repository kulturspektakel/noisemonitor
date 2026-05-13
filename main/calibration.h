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

// Returns the persisted x100 value (record_writer/MQTT need this for the
// noise_recording.calibration_offset_db_x100 field).
int32_t calibration_offset_x100(void);
