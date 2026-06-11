#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Number of 1/3-octave bands. Must equal audio_dsp.h NOISE_BANDS.
#define CALIBRATION_BANDS 31

// Per-band calibration: 31 signed offsets in 0.5 dB steps (the wire/NVS form).
// band_offset_dB = step * 0.5. Replaces the old single scalar offset — the
// fixed FFT-energy->dB-SPL anchor now lives in audio_dsp.c, so calibration is
// purely a per-band frequency-response + residual-sensitivity trim.

// Read NVS, cache the per-band steps, set CALIBRATED if a calibration is stored.
// Also erases the obsolete scalar-offset key from older firmware.
void calibration_init(void);

// Persist the 31 per-band steps to NVS, update the cache, set CALIBRATED, and
// flag the DSP task to re-apply. n must equal CALIBRATION_BANDS.
esp_err_t calibration_set_bands(const int8_t* steps, size_t n);

// Erase the stored calibration, zero the cache, clear CALIBRATED, flag re-apply.
esp_err_t calibration_clear(void);

// Copy the cached steps (0.5 dB units) — for the BLE read handler.
void calibration_get_bands(int8_t* out, size_t n);

// Copy the cached calibration as dB (step * 0.5) — for the DSP task.
void calibration_get_bands_db(float* out_db, size_t n);

// Test-and-clear the "calibration changed" flag (consumed by the DSP task).
bool calibration_take_band_dirty(void);
