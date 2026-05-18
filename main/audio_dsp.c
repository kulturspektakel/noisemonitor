#include "audio_dsp.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "calibration.h"
#include "constants.h"
#include "driver/i2s_std.h"
#include "esp_dsp.h"
#include "esp_log.h"
#include "event_group.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "heap_diag.h"
#include "nanopb/pb_encode.h"
#include "power_management.h"

static const char* TAG = "audio_dsp";

// --- Optional: synthesize audio in software ---------------------------------
// Set to 1 to bypass the mic and inject a synthetic signal through the DSP
// pipeline (useful for testing the rest of the chain without a working mic).
#define SIMULATE_MIC 0

// --- I²S configuration --------------------------------------------------------
// INMP441: standard I²S mic, 24-bit signed samples left-aligned in a 32-bit slot.
// Tie L/R pin to GND for left channel (matches I2S_STD_SLOT_LEFT below).
#define SAMPLE_RATE      48000
#define I2S_BCLK_GPIO    GPIO_NUM_11   // SCK on the mic
#define I2S_WS_GPIO      GPIO_NUM_9    // WS on the mic
#define I2S_DATA_GPIO    GPIO_NUM_12   // SD on the mic
#define I2S_BUFFER_FRAMES 1024         // per i2s_channel_read call

// --- FFT configuration --------------------------------------------------------
// Spec §13 prefers radix-4 FFT, but on this no-PSRAM ESP32-S3 the radix-4
// twiddle table (64 KB at 4096-pt) couldn't be allocated contiguously after
// WiFi+BLE fragmented the heap. Radix-2 uses only FFT_SIZE floats = 16 KB
// for its twiddle table — fits anywhere. esp-dsp's dsps_fft2r_fc32_aes3
// (SIMD) closes most of the radix-2-vs-radix-4 speed gap on S3. Our DSP
// budget at 160 MHz has plenty of headroom either way.
#define FFT_SIZE 4096
#define FFT_HOP  2048     // 50% overlap

// --- Aggregate ring size — only 15-min sliding LAeq/LCeq are exposed ---------
#define RING_15M 900

// --- Center frequencies (Hz) per band, spec §5 --------------------------------
static const float band_centers[NOISE_BANDS] = {
    16, 20, 25, 31.5f, 40, 50, 63, 80, 100, 125,
    160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250,
    1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000, 12500,
    16000
};

// A/C-weighting now applied per FFT bin (see compute_bin_weights) using the
// IEC 61672 analog filter formulas; the band-table approximation was removed.

// --- Queues (extern) ---------------------------------------------------------
QueueHandle_t record_writer_queue;
QueueHandle_t mqtt_publisher_queue;
QueueHandle_t ble_publisher_queue;

// --- Static buffers ---------------------------------------------------------
static i2s_chan_handle_t i2s_rx;

static int32_t  raw_buffer[I2S_BUFFER_FRAMES];     // raw 24-in-32-bit samples
static float    fft_ring[FFT_SIZE];                // last FFT_SIZE float samples
static int      fft_ring_head = 0;                 // next write index
static int      samples_since_last_fft = 0;        // hop counter

static float    fft_work[FFT_SIZE * 2];            // complex pairs for FFT in/out
// Twiddle-factor table for dsps_fft2r_init_fc32 (16 KB at FFT_SIZE=4096).
// Heap-allocated in audio_dsp_preinit() (called from app_main).
static float*   fft_table = NULL;
static int      band_start_bin[NOISE_BANDS + 1];   // inclusive start; one extra = exclusive end of last band

// Per-bin A and C weighting (linear power factors), populated at init from
// IEC 61672 formulas. 8 KB each, BSS. Avoid the band-center approximation
// for LAeq/LCeq — apply weighting at each bin's exact frequency instead.
static float    a_weight_bin[FFT_SIZE / 2];
static float    c_weight_bin[FFT_SIZE / 2];

// Per-second energy accumulators
static double   band_energy_sum[NOISE_BANDS];
static double   a_weighted_sum_per_sec = 0.0;
static double   c_weighted_sum_per_sec = 0.0;
// Max-per-second of single-FFT A/C weighted energy. Each FFT covers 85 ms of
// audio (≈ Fast time-weighting's 125 ms response window), so the loudest FFT
// in a second is a reasonable proxy for LAFmax/LCFmax without a per-sample
// IIR biquad cascade.
static double   a_weighted_max_per_fft = 0.0;
static double   c_weighted_max_per_fft = 0.0;
static int      fft_windows_in_second = 0;
static int      broadband_samples_in_second = 0;
static float    peak_abs_sample_in_second = 0.0f;

// 15-min sliding LAeq/LCeq ring buffers (linear energies, not dB).
static float    laeq_ring_15m[RING_15M];
static float    lceq_ring_15m[RING_15M];
static int      ring_idx_15m = 0;
static int      total_seconds = 0;

static uint32_t seq_no = 0;

#if SIMULATE_MIC
// Four tones at frequencies spread across the 1/3-octave bands, each with its
// own slow LFO modulating its amplitude. The LFO periods are coprime so the
// composite spectrum shifts continuously — LAeq swings ~15 dB over time and
// different band[] cells light up at different moments. Plus a white-noise
// floor that contributes broadband content to every band.
// Replace with real I²S (SIMULATE_MIC=0) once the INMP441 is wired.
#define SIM_TONE_COUNT 4
static const float sim_tone_freq[SIM_TONE_COUNT]    = {  80.0f, 500.0f, 2000.0f, 8000.0f };
static const float sim_tone_amp[SIM_TONE_COUNT]     = {   0.10f,  0.08f,   0.06f,   0.04f };
static const float sim_tone_lfo_hz[SIM_TONE_COUNT]  = {   0.05f,  0.07f,   0.11f,   0.13f };
static const float sim_noise_amp                    = 0.01f;

static uint32_t sim_phase = 0;
static uint32_t sim_rng = 0x12345678u;
static void fill_simulated_buffer(int32_t* dst, int n) {
  const float inv_sr = 1.0f / (float)SAMPLE_RATE;
  const float two_pi = 2.0f * (float)M_PI;
  for (int i = 0; i < n; i++) {
    float t = (float)sim_phase * inv_sr;
    // Global envelope: 30 s period, ranges 0.05..1.0 → ~26 dB peak-to-peak
    // amplitude swing so LAeq is visibly rising and falling.
    float global_env = 0.05f + 0.95f * (0.5f + 0.5f * sinf(two_pi * 0.033f * t));
    float sample = 0.0f;
    for (int k = 0; k < SIM_TONE_COUNT; k++) {
      float lfo = 0.5f + 0.5f * sinf(two_pi * sim_tone_lfo_hz[k] * t);
      sample += sim_tone_amp[k] * lfo * sinf(two_pi * sim_tone_freq[k] * t);
    }
    sample *= global_env;
    sim_rng = sim_rng * 1664525u + 1013904223u;  // LCG noise (always-on floor)
    sample += sim_noise_amp * ((int32_t)(sim_rng >> 8) / 8388608.0f - 0.5f) * 2.0f;
    sim_phase++;

    int32_t s24 = (int32_t)(sample * 8388608.0f);
    if (s24 >  8388607)  s24 =  8388607;
    if (s24 < -8388608)  s24 = -8388608;
    dst[i] = s24 << 8;
  }
}
#endif

// --- Init helpers -----------------------------------------------------------
static void i2s_init(void) {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  // Halve the DMA buffer pool vs defaults (6×240 frames → 4×120). Saves ~4 KB
  // of contiguous heap during audio_dsp init. With 120 frames @ 48 kHz the
  // DMA window is 2.5 ms — well under our 21 ms i2s_channel_read cadence.
  chan_cfg.dma_desc_num  = 4;
  chan_cfg.dma_frame_num = 120;
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &i2s_rx));

  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
          .mclk = I2S_GPIO_UNUSED,
          .bclk = I2S_BCLK_GPIO,
          .ws = I2S_WS_GPIO,
          .dout = I2S_GPIO_UNUSED,
          .din = I2S_DATA_GPIO,
          .invert_flags = { 0 },
      },
  };
  // INMP441 L/R tied to GND => mic transmits on the LEFT channel.
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_rx, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(i2s_rx));
}

static void compute_band_edges(void) {
  // 1/3-octave band lower edge = center * 2^(-1/6); upper edge = center * 2^(+1/6).
  // bin index = round(freq / (SAMPLE_RATE / FFT_SIZE))
  const float bin_hz = (float)SAMPLE_RATE / (float)FFT_SIZE;
  const float two_pow_sixth = 1.122462048309373f;   // 2^(1/6)
  for (int i = 0; i < NOISE_BANDS; i++) {
    float lower = band_centers[i] / two_pow_sixth;
    int bin = (int)(lower / bin_hz + 0.5f);
    if (bin < 1) bin = 1;
    if (bin > FFT_SIZE / 2) bin = FFT_SIZE / 2;
    band_start_bin[i] = bin;
  }
  float top_upper = band_centers[NOISE_BANDS - 1] * two_pow_sixth;
  int top_bin = (int)(top_upper / bin_hz + 0.5f);
  if (top_bin > FFT_SIZE / 2) top_bin = FFT_SIZE / 2;
  band_start_bin[NOISE_BANDS] = top_bin;
}

// Populate a_weight_bin[] and c_weight_bin[] with linear power weights
// (10^(weight_dB/10)) per FFT bin, using IEC 61672 analog filter formulas.
// Normalized so weight at 1 kHz = 1.0 (= 0 dB), matching standard A/C weighting.
static void compute_bin_weights(void) {
  const double bin_hz = (double)SAMPLE_RATE / (double)FFT_SIZE;
  // IEC 61672 pole frequencies (Hz)
  const double f1_sq = 20.598997   * 20.598997;
  const double f2_sq = 107.65265   * 107.65265;
  const double f3_sq = 737.86223   * 737.86223;
  const double f4_sq = 12194.217   * 12194.217;

  // R(1000)^2 for A and C — used to normalize so weight at 1 kHz is unity.
  double ra1000_sq, rc1000_sq;
  {
    double f_sq = 1000.0 * 1000.0;
    double t1 = f_sq + f1_sq;
    double t2 = f_sq + f2_sq;
    double t3 = f_sq + f3_sq;
    double t4 = f_sq + f4_sq;
    double ra = (f4_sq * f_sq * f_sq) / (t1 * sqrt(t2 * t3) * t4);
    double rc = (f4_sq * f_sq)        / (t1 * t4);
    ra1000_sq = ra * ra;
    rc1000_sq = rc * rc;
  }

  a_weight_bin[0] = 0.0f;  // DC bin is excluded from sums
  c_weight_bin[0] = 0.0f;
  for (int k = 1; k < FFT_SIZE / 2; k++) {
    double f = (double)k * bin_hz;
    double f_sq = f * f;
    double t1 = f_sq + f1_sq;
    double t2 = f_sq + f2_sq;
    double t3 = f_sq + f3_sq;
    double t4 = f_sq + f4_sq;
    double ra = (f4_sq * f_sq * f_sq) / (t1 * sqrt(t2 * t3) * t4);
    double rc = (f4_sq * f_sq)        / (t1 * t4);
    a_weight_bin[k] = (float)((ra * ra) / ra1000_sq);
    c_weight_bin[k] = (float)((rc * rc) / rc1000_sq);
  }
}

esp_err_t audio_dsp_preinit(void) {
  // Radix-2 twiddle table = FFT_SIZE floats (16 KB at 4096-pt). Aligned to
  // 16 bytes for esp-dsp's SIMD load instructions.
  fft_table = heap_caps_aligned_alloc(16, FFT_SIZE * sizeof(float),
                                      MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (fft_table == NULL) {
    ESP_LOGE(TAG, "preinit: failed to allocate %d-byte FFT table",
             (int)(FFT_SIZE * sizeof(float)));
    return ESP_ERR_NO_MEM;
  }
  ESP_LOGI(TAG, "preinit: FFT table allocated (%d bytes)",
           (int)(FFT_SIZE * sizeof(float)));
  return ESP_OK;
}

static void dsp_init(void) {
  if (fft_table == NULL) {
    ESP_LOGE(TAG, "fft_table not allocated — did app_main forget audio_dsp_preinit()?");
    vTaskDelete(NULL);
    return;
  }
  esp_err_t err = dsps_fft2r_init_fc32(fft_table, FFT_SIZE);
  if (err != ESP_OK && err != ESP_ERR_DSP_REINITIALIZED) {
    ESP_LOGE(TAG, "dsps_fft2r_init_fc32 failed: 0x%x", err);
    ESP_ERROR_CHECK(err);
  }
  compute_band_edges();
  compute_bin_weights();
}

// --- Conversion helpers -----------------------------------------------------
// Spec §5: byte = round((dB - 20) * 2). Clamp to 0..255.
static uint8_t encode_db_to_byte(float db) {
  float v = (db - 20.0f) * 2.0f;
  if (v < 0.0f) return 0;
  if (v > 255.0f) return 255;
  return (uint8_t)(v + 0.5f);
}

// --- FFT execution: pulls FFT_SIZE samples from fft_ring (oldest-first) ----
static void run_fft_and_accumulate(void) {
  // Copy fft_ring into fft_work as complex pairs (real, 0), applying Hann on
  // the fly (saves 16 KB vs. a precomputed table; ~9 ms/sec of cosf calls).
  // fft_ring is circular; oldest sample is at fft_ring_head.
  const float hann_k = 2.0f * (float)M_PI / (float)FFT_SIZE;
  for (int i = 0; i < FFT_SIZE; i++) {
    int idx = (fft_ring_head + i) % FFT_SIZE;
    float h = 0.5f * (1.0f - cosf((float)i * hann_k));
    fft_work[2 * i]     = fft_ring[idx] * h;
    fft_work[2 * i + 1] = 0.0f;
  }

  // ANSI variant: SIMD-accelerated dsps_fft2r_fc32 on ESP32-S3 returns a
  // constant 1.6 in every imaginary bin from zero input (verified empirically
  // 2026-05-15). Bug is in dsps_fft2r_fc32_aes3_.S (community-contributed
  // assembly, esp-dsp 1.8.2). ae32 SIMD doesn't run on S3 either (crashes).
  // ANSI is correct; ~14% of 1 core for 23 FFTs/s — well within budget.
  dsps_fft2r_fc32_ansi(fft_work, FFT_SIZE);
  dsps_bit_rev_fc32_ansi(fft_work, FFT_SIZE);

  // Hann window's global power gain (0.375) is absorbed into the calibration
  // offset — no per-frame compensation needed.

  // Accumulate magnitude-squared into bands (for the 31-band per-second display).
  for (int b = 0; b < NOISE_BANDS; b++) {
    int start = band_start_bin[b];
    int end = band_start_bin[b + 1];
    if (end <= start) end = start + 1;
    if (end > FFT_SIZE / 2) end = FFT_SIZE / 2;
    double sum = 0.0;
    for (int k = start; k < end; k++) {
      float re = fft_work[2 * k];
      float im = fft_work[2 * k + 1];
      sum += (double)re * re + (double)im * im;
    }
    // Sum total band power (not per-bin mean): correct for 1/3-octave SPL
    // and gives flat frequency response under broadband signals.
    band_energy_sum[b] += sum;
  }

  // Per-bin A/C weighted accumulation for LAeq/LCeq. Applying the weight at
  // each bin's exact frequency (vs. at band centers) eliminates the
  // frequency-quantization error that overestimates tones between band
  // centers — especially at low frequencies where bands are only a few bins
  // wide.
  double a_sum = 0.0, c_sum = 0.0;
  for (int k = 1; k < FFT_SIZE / 2; k++) {
    float re = fft_work[2 * k];
    float im = fft_work[2 * k + 1];
    double bin_energy = (double)re * re + (double)im * im;
    a_sum += bin_energy * (double)a_weight_bin[k];
    c_sum += bin_energy * (double)c_weight_bin[k];
  }
  a_weighted_sum_per_sec += a_sum;
  c_weighted_sum_per_sec += c_sum;
  if (a_sum > a_weighted_max_per_fft) a_weighted_max_per_fft = a_sum;
  if (c_sum > c_weighted_max_per_fft) c_weighted_max_per_fft = c_sum;

  fft_windows_in_second++;
}

// --- Aggregate ring helpers --------------------------------------------------
static float compute_leq_from_ring(const float* ring, int size, int valid) {
  if (valid <= 0) return 0.0f;
  double sum = 0.0;
  for (int i = 0; i < valid; i++) sum += ring[i];
  if (sum <= 0.0) return 0.0f;
  return 10.0f * log10f((float)(sum / (double)valid));
}

uint8_t audio_dsp_get_laeq_15m(uint16_t* sec_out) {
  int valid = total_seconds < RING_15M ? total_seconds : RING_15M;
  if (sec_out) *sec_out = (uint16_t)valid;
  return encode_db_to_byte(compute_leq_from_ring(laeq_ring_15m, RING_15M, valid));
}
uint8_t audio_dsp_get_lceq_15m(uint16_t* sec_out) {
  int valid = total_seconds < RING_15M ? total_seconds : RING_15M;
  if (sec_out) *sec_out = (uint16_t)valid;
  return encode_db_to_byte(compute_leq_from_ring(lceq_ring_15m, RING_15M, valid));
}

// --- Shared record → protobuf helpers ----------------------------------------
void record_to_pb(const record_t* r, NoiseRecording_Record* out) {
  out->seq_no = r->seq_no;
  memcpy(out->bands, r->bands, sizeof(r->bands));
  out->laeq_1s   = r->laeq_1s;
  out->lceq_1s   = r->lceq_1s;
  out->lafmax_1s = r->lafmax_1s;
  out->lcfmax_1s = r->lcfmax_1s;
  out->lcpeak_1s = r->lcpeak_1s;
}

size_t record_encode_single(const record_t* r, uint8_t* out, size_t cap) {
  // Stream the NoiseRecording without staging the full struct: with
  // max_count:300 a stack-allocated NoiseRecording is ~12 KB and overflows
  // the 4 KB publisher task stacks. Only the single sub-Record is staged
  // here (40 bytes).
  NoiseRecording_Record sub;
  record_to_pb(r, &sub);

  pb_ostream_t stream = pb_ostream_from_buffer(out, cap);
  // Field 2: repeated NoiseRecording.Record records (= one Record)
  if (!pb_encode_tag(&stream, PB_WT_STRING, 2)) goto fail;
  if (!pb_encode_submessage(&stream, NoiseRecording_Record_fields, &sub)) goto fail;
  // Field 3: laeq_15m (uint32 varint)
  if (!pb_encode_tag(&stream, PB_WT_VARINT, 3)) goto fail;
  if (!pb_encode_varint(&stream, audio_dsp_get_laeq_15m(NULL))) goto fail;
  // Field 4: lceq_15m (uint32 varint)
  if (!pb_encode_tag(&stream, PB_WT_VARINT, 4)) goto fail;
  if (!pb_encode_varint(&stream, audio_dsp_get_lceq_15m(NULL))) goto fail;
  // Field 5: battery_mv (optional) — omitted when USB is connected, since
  // the battery voltage reading isn't meaningful while charging.
  if (usb_voltage <= 1000 && battery_voltage > 0) {
    if (!pb_encode_tag(&stream, PB_WT_VARINT, 5)) goto fail;
    if (!pb_encode_varint(&stream, (uint64_t)battery_voltage)) goto fail;
  }
  return stream.bytes_written;
fail:
  ESP_LOGW(TAG, "record_encode_single: %s", stream.errmsg);
  return 0;
}

// --- Per-second emission -----------------------------------------------------
//
// Derive band-dB from accumulated energy. Apply calibration offset.
// LAeq,1s and LCeq,1s are computed from band dB via the IEC 61672 tables
// (spec Appendix). LAFmax/LCfmax/LCpeak are approximated from the FFT band
// energies in v1 — a Phase E refinement will add per-sample biquad filtering
// with proper Fast time-weighting (125 ms exponential smoother).
static void emit_per_second(void) {
  float cal = calibration_offset_db();

  // LAeq,1s and LCeq,1s from per-bin-weighted FFT energy (accumulated in
  // run_fft_and_accumulate). This applies the IEC 61672 A/C filter at each
  // bin's actual frequency, eliminating the band-center quantization error.
  double a_mean = (fft_windows_in_second > 0)
      ? a_weighted_sum_per_sec / (double)fft_windows_in_second
      : 0.0;
  double c_mean = (fft_windows_in_second > 0)
      ? c_weighted_sum_per_sec / (double)fft_windows_in_second
      : 0.0;
  float laeq_db = (a_mean > 0.0) ? 10.0f * log10f((float)a_mean) + cal : 0.0f;
  float lceq_db = (c_mean > 0.0) ? 10.0f * log10f((float)c_mean) + cal : 0.0f;

  // LAFmax/LCFmax: max single-FFT A/C-weighted energy seen this second.
  // 85 ms FFT window approximates Fast time-weighting; not a perfect IEC
  // 61672 implementation (no exponential smoother) but captures intra-second
  // peaks that the per-second average can't.
  float lafmax_db = (a_weighted_max_per_fft > 0.0)
      ? 10.0f * log10f((float)a_weighted_max_per_fft) + cal : 0.0f;
  float lcfmax_db = (c_weighted_max_per_fft > 0.0)
      ? 10.0f * log10f((float)c_weighted_max_per_fft) + cal : 0.0f;
  // LCpeak: unweighted absolute sample peak. Calibration's `cal` offset maps
  // FFT-energy numbers to dB SPL, but the peak metric operates on raw [-1,1]
  // sample amplitude — a scale that's ~60 dB different from FFT energy for
  // our 4096-pt Hann-windowed FFT. The extra 20·log10(FFT_SIZE/4) term
  // compensates so LCpeak ends up on the same dB SPL scale as LAeq/LCeq.
  // (Still not C-weighted — proper LCpeak needs an IIR C-weight filter on
  // the time-domain samples before the peak detector.)
  const float lcpeak_offset = cal + 20.0f * log10f((float)FFT_SIZE / 4.0f);
  float lcpeak_db = (peak_abs_sample_in_second > 0.0f)
      ? 20.0f * log10f(peak_abs_sample_in_second) + lcpeak_offset
      : 0.0f;

  EventBits_t bits = xEventGroupGetBits(event_group);
  bool time_set = (bits & TIME_SET) != 0;

  // Update the 15-min sliding rings (linear energy).
  float laeq_lin = (float)pow(10.0, laeq_db / 10.0);
  float lceq_lin = (float)pow(10.0, lceq_db / 10.0);
  laeq_ring_15m[ring_idx_15m] = laeq_lin;
  lceq_ring_15m[ring_idx_15m] = lceq_lin;
  ring_idx_15m = (ring_idx_15m + 1) % RING_15M;
  if (total_seconds < RING_15M) total_seconds++;

  if (time_set) {
    record_t r = { 0 };
    r.seq_no    = seq_no++;
    for (int b = 0; b < NOISE_BANDS; b++) {
      double mean = (fft_windows_in_second > 0)
          ? band_energy_sum[b] / (double)fft_windows_in_second
          : 0.0;
      float db = (mean > 0.0) ? 10.0f * log10f((float)mean) + cal : 0.0f;
      r.bands[b] = encode_db_to_byte(db);
    }
    r.laeq_1s   = encode_db_to_byte(laeq_db);
    r.lceq_1s   = encode_db_to_byte(lceq_db);
    r.lafmax_1s = encode_db_to_byte(lafmax_db);
    r.lcfmax_1s = encode_db_to_byte(lcfmax_db);
    r.lcpeak_1s = encode_db_to_byte(lcpeak_db);

    if (xQueueSend(record_writer_queue, &r, 0) != pdTRUE) {
      ESP_LOGW(TAG, "record_writer_queue full, dropping record %lu", (unsigned long)r.seq_no);
    }
    if (xQueueSend(mqtt_publisher_queue, &r, 0) != pdTRUE) {
      ESP_LOGW(TAG, "mqtt_publisher_queue full, dropping record %lu", (unsigned long)r.seq_no);
    }
    if (xQueueSend(ble_publisher_queue, &r, 0) != pdTRUE) {
      ESP_LOGW(TAG, "ble_publisher_queue full, dropping record %lu", (unsigned long)r.seq_no);
    }
  } else {
    ESP_LOGI(TAG, "waiting for time_set; current LAeq=%.1f LCeq=%.1f (cal offset=%+.2f)",
             laeq_db, lceq_db, cal);
  }

  // Reset per-second accumulators
  for (int b = 0; b < NOISE_BANDS; b++) band_energy_sum[b] = 0.0;
  a_weighted_sum_per_sec = 0.0;
  c_weighted_sum_per_sec = 0.0;
  a_weighted_max_per_fft = 0.0;
  c_weighted_max_per_fft = 0.0;
  fft_windows_in_second = 0;
  broadband_samples_in_second = 0;
  peak_abs_sample_in_second = 0.0f;
}

// --- Main task ---------------------------------------------------------------
void audio_dsp(void* params) {
#if SIMULATE_MIC
  ESP_LOGW(TAG, "SIMULATE_MIC=1 — bypassing I²S, injecting 1 kHz sine wave");
#else
  i2s_init();
#endif
  dsp_init();
  ESP_LOGI(TAG, "DSP initialized; sampling at %d Hz, %d-pt FFT", SAMPLE_RATE, FFT_SIZE);
  heap_diag("after dsp ready");

  while (true) {
    size_t bytes_read = 0;
#if SIMULATE_MIC
    fill_simulated_buffer(raw_buffer, I2S_BUFFER_FRAMES);
    bytes_read = I2S_BUFFER_FRAMES * sizeof(int32_t);
    // Pace at real-time: I2S_BUFFER_FRAMES samples at SAMPLE_RATE = ~21 ms.
    vTaskDelay(pdMS_TO_TICKS(1000 * I2S_BUFFER_FRAMES / SAMPLE_RATE));
#else
    esp_err_t err = i2s_channel_read(
        i2s_rx, raw_buffer, sizeof(raw_buffer), &bytes_read, portMAX_DELAY
    );
    if (err != ESP_OK || bytes_read == 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
#endif
    int frames = bytes_read / sizeof(int32_t);

    for (int i = 0; i < frames; i++) {
      // INMP441 outputs 24-bit signed left-aligned in 32-bit slot.
      int32_t s24 = raw_buffer[i] >> 8;
      float f = (float)s24 / 8388608.0f;   // 2^23

      // Broadband accumulators
      broadband_samples_in_second++;
      float af = fabsf(f);
      if (af > peak_abs_sample_in_second) peak_abs_sample_in_second = af;

      // Append to FFT ring (circular)
      fft_ring[fft_ring_head] = f;
      fft_ring_head = (fft_ring_head + 1) % FFT_SIZE;
      samples_since_last_fft++;

      // Run FFT every FFT_HOP new samples (50% overlap)
      if (samples_since_last_fft >= FFT_HOP) {
        samples_since_last_fft = 0;
        run_fft_and_accumulate();
      }

      // Emit a record once we've consumed SAMPLE_RATE samples
      if (broadband_samples_in_second >= SAMPLE_RATE) {
        emit_per_second();
      }
    }
  }
}
