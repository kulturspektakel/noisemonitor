#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "noise.pb.h"

#define NOISE_BANDS 31

// Per-second measurement payload. Fields use the spec §5 encoding:
//   byte_value = round((dB - 20.0) * 2.0)
typedef struct {
  uint32_t seq_no;
  uint8_t  bands[NOISE_BANDS];
  uint8_t  laeq_1s;
  uint8_t  lceq_1s;
  uint8_t  lafmax_1s;
  uint8_t  lcfmax_1s;
  uint8_t  lcpeak_1s;
} record_t;

extern QueueHandle_t record_writer_queue;
extern QueueHandle_t mqtt_publisher_queue;
extern QueueHandle_t ble_publisher_queue;

// Allocate the 64 KB FFT twiddle table from internal RAM. Must be called from
// app_main BEFORE wifi_connect / ble_publisher tasks start, so the contiguous
// chunk is reserved before WiFi/BLE drivers consume the heap.
esp_err_t audio_dsp_preinit(void);

void audio_dsp(void* params);

// Snapshot of the sliding-window aggregates. Each value is a uint8 in
// spec §5 encoding (= (dB - 20) * 2). `has_5m` / `has_30m` are true only
// once the corresponding ring has fully populated, so callers can gate
// emission without knowing the window sizes.
typedef struct {
  uint8_t laeq_5m;
  uint8_t lceq_5m;
  uint8_t laeq_30m;
  uint8_t lceq_30m;
  bool    has_5m;
  bool    has_30m;
} audio_dsp_aggregates_t;

// Single-pass read of both 5m and 30m A/C-weighted Leqs from the shared
// 30-min ring buffers.
void audio_dsp_get_aggregates(audio_dsp_aggregates_t* out);

// --- Shared record → protobuf helpers ----------------------------------------
// Single source of truth for record_t → wire-format Record mapping. Use these
// in mqtt_publisher / ble_publisher / record_writer so adding/removing fields
// only touches one place.

// Copy the per-second fields of `r` into the protobuf Record slot
// (NoiseRecording.Record).
void record_to_pb(const record_t* r, NoiseRecording_Record* out);

// Encode `r` as a single-Record NoiseRecording, snapshotting the current
// LAeq_15m window value. Returns bytes written to `out`, or 0 on failure.
// Used by MQTT and BLE which both publish one record per message.
size_t record_encode_single(const record_t* r, uint8_t* out, size_t cap);
