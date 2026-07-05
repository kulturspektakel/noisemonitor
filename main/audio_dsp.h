#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "noise.pb.h"

#define NOISE_BANDS 31

// One measurement record. Fields use the spec §5 encoding:
//   byte_value = round((dB - 20.0) * 2.0)
// A record spans 1 s (live MQTT/BLE) or RECORD_INTERVAL_SECONDS (on-disk
// aggregate) — hence no `_1s` suffix; the span is carried on the wire in
// NoiseRecording.record_interval_seconds.
typedef struct {
  uint32_t seq_no;
  uint8_t  bands[NOISE_BANDS];
  uint8_t  laeq;
  uint8_t  lceq;
  uint8_t  lafmax;
  uint8_t  lcfmax;
  uint8_t  lcpeak;
  // Wall-clock start of this record's measurement window. Set by the producer
  // (audio_dsp) so downstream consumers timestamp from when the measurement
  // was taken, not when they happen to dequeue it. Only the on-disk log path
  // reads it; the live 1 s records leave it at their emit time / unused.
  time_t   window_start;
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

// Copy the measurement fields of `r` into a (flattened) NoiseRecording.
// Does NOT set record_interval_seconds / battery / rolling windows — the
// caller sets those for its context.
void record_to_pb(const record_t* r, NoiseRecording* out);

// Stamp the current 5m/30m rolling-window Leqs onto `out` (has_ flags follow
// the ring-full gate). Shared by the live and on-disk encoders.
void record_apply_aggregates(NoiseRecording* out);

// Encode `r` as a live NoiseRecording (record_interval_seconds = 1), attaching
// battery + the current 5m/30m windows. Returns bytes written to `out`, or 0
// on failure. Used by MQTT and BLE.
size_t record_encode_single(const record_t* r, uint8_t* out, size_t cap);
