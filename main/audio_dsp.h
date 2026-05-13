#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

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

// Aggregate snapshots — implemented in Phase C. Each returns a uint8 in spec §5 encoding.
// `*seconds_out` reports how many seconds of the window have actually been observed
// (saturating at window size). Pass NULL if not needed.
uint8_t audio_dsp_get_laeq_1m(uint16_t* seconds_out);
uint8_t audio_dsp_get_laeq_15m(uint16_t* seconds_out);
uint8_t audio_dsp_get_laeq_30m(uint16_t* seconds_out);
uint8_t audio_dsp_get_lceq_1m(uint16_t* seconds_out);
uint8_t audio_dsp_get_lceq_15m(uint16_t* seconds_out);
uint8_t audio_dsp_get_lafmax_1m(uint16_t* seconds_out);
uint8_t audio_dsp_get_lcpeak_1m(uint16_t* seconds_out);
