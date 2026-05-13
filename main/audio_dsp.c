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

static const char* TAG = "audio_dsp";

// --- Temporary: synthesize audio in software (no mic hardware yet) -----------
// Set to 1 to bypass the I²S peripheral and inject a 1 kHz sine wave with a
// little broadband noise on top. The rest of the DSP pipeline behaves exactly
// as if real audio were arriving. Flip back to 0 once the ICS-43434 is wired.
#define SIMULATE_MIC 1

// --- I²S configuration --------------------------------------------------------
#define SAMPLE_RATE      48000
#define I2S_BCLK_GPIO    GPIO_NUM_9
#define I2S_WS_GPIO      GPIO_NUM_10
#define I2S_DATA_GPIO    GPIO_NUM_11
#define I2S_BUFFER_FRAMES 1024   // per i2s_channel_read call

// --- FFT configuration --------------------------------------------------------
// Spec §13 prefers radix-4 FFT, but on this no-PSRAM ESP32-S3 the radix-4
// twiddle table (64 KB at 4096-pt) couldn't be allocated contiguously after
// WiFi+BLE fragmented the heap. Radix-2 uses only FFT_SIZE floats = 16 KB
// for its twiddle table — fits anywhere. esp-dsp's dsps_fft2r_fc32_aes3
// (SIMD) closes most of the radix-2-vs-radix-4 speed gap on S3. Our DSP
// budget at 160 MHz has plenty of headroom either way.
#define FFT_SIZE 4096
#define FFT_HOP  2048     // 50% overlap

// --- Aggregate ring sizes (spec §9) -------------------------------------------
#define RING_1M  60
#define RING_15M 900
#define RING_30M 1800

// --- Center frequencies (Hz) per band, spec §5 --------------------------------
static const float band_centers[NOISE_BANDS] = {
    16, 20, 25, 31.5f, 40, 50, 63, 80, 100, 125,
    160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250,
    1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000, 12500,
    16000
};

// --- A/C-weighting tables per band, spec Appendix -----------------------------
static const float a_weight_db[NOISE_BANDS] = {
  -56.7f, -50.5f, -44.7f, -39.4f, -34.6f, -30.2f, -26.2f, -22.5f, -19.1f, -16.1f,
  -13.4f, -10.9f,  -8.6f,  -6.6f,  -4.8f,  -3.2f,  -1.9f,  -0.8f,   0.0f,  +0.6f,
   +1.0f,  +1.2f,  +1.3f,  +1.2f,  +1.0f,  +0.5f,  -0.1f,  -1.1f,  -2.5f,  -4.3f,
   -6.6f
};
static const float c_weight_db[NOISE_BANDS] = {
   -8.5f, -6.2f, -4.4f, -3.0f, -2.0f, -1.3f, -0.8f, -0.5f, -0.3f, -0.2f,
   -0.1f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,
   -0.1f, -0.2f, -0.3f, -0.5f, -0.8f, -1.3f, -2.0f, -3.0f, -4.4f, -6.2f,
   -8.5f
};

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

static float    hann_window[FFT_SIZE];
static float    fft_work[FFT_SIZE * 2];            // complex pairs for FFT in/out
// Twiddle-factor table for dsps_fft2r_init_fc32 (16 KB at FFT_SIZE=4096).
// Heap-allocated in audio_dsp_preinit() (called from app_main).
static float*   fft_table = NULL;
static int      band_start_bin[NOISE_BANDS + 1];   // inclusive start; one extra = exclusive end of last band

// Per-second energy accumulators
static double   band_energy_sum[NOISE_BANDS];
static int      fft_windows_in_second = 0;
static int      broadband_samples_in_second = 0;
static float    peak_abs_sample_in_second = 0.0f;

// Aggregate ring buffers (linear energies, not dB)
static float    laeq_ring_1m[RING_1M];
static float    laeq_ring_15m[RING_15M];
static float    laeq_ring_30m[RING_30M];
static float    lceq_ring_1m[RING_1M];
static float    lceq_ring_15m[RING_15M];
static float    lafmax_ring_1m[RING_1M];
static float    lcpeak_ring_1m[RING_1M];
static int      ring_idx_1m = 0;
static int      ring_idx_15m = 0;
static int      ring_idx_30m = 0;
static int      total_seconds = 0;

static uint32_t seq_no = 0;

// Forward refs
static uint8_t encode_db_to_byte(float db);
static float   decode_byte_to_db(uint8_t b);

#if SIMULATE_MIC
// Four tones at frequencies spread across the 1/3-octave bands, each with its
// own slow LFO modulating its amplitude. The LFO periods are coprime so the
// composite spectrum shifts continuously — LAeq swings ~15 dB over time and
// different band[] cells light up at different moments. Plus a white-noise
// floor that contributes broadband content to every band.
// Replace with real I²S (SIMULATE_MIC=0) once an ICS-43434 is wired.
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
  // ICS-43434 SEL is tied to GND => sends data on the LEFT channel.
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
  dsps_wind_hann_f32(hann_window, FFT_SIZE);
  compute_band_edges();
}

// --- Conversion helpers -----------------------------------------------------
// Spec §5: byte = round((dB - 20) * 2). Clamp to 0..255.
static uint8_t encode_db_to_byte(float db) {
  float v = (db - 20.0f) * 2.0f;
  if (v < 0.0f) return 0;
  if (v > 255.0f) return 255;
  return (uint8_t)(v + 0.5f);
}
static float decode_byte_to_db(uint8_t b) {
  return 20.0f + b / 2.0f;
}

// --- FFT execution: pulls FFT_SIZE samples from fft_ring (oldest-first) ----
static void run_fft_and_accumulate(void) {
  // Copy fft_ring into fft_work as complex pairs (real, 0), applying Hann.
  // fft_ring is circular; oldest sample is at fft_ring_head.
  for (int i = 0; i < FFT_SIZE; i++) {
    int idx = (fft_ring_head + i) % FFT_SIZE;
    fft_work[2 * i]     = fft_ring[idx] * hann_window[i];
    fft_work[2 * i + 1] = 0.0f;
  }

  dsps_fft2r_fc32(fft_work, FFT_SIZE);
  dsps_bit_rev2r_fc32(fft_work, FFT_SIZE);

  // Hann coherent-gain compensation: sum(hann) / N = 0.5 → multiply by 2.
  // But we work with magnitude squared, so factor 4 in linear energy.
  // We'll absorb the global scale into calibration; just compute |X[k]|^2 here.

  // Accumulate magnitude-squared into bands.
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
    // Normalize by number of bins in the band so it's a per-bin mean.
    band_energy_sum[b] += sum / (double)(end - start);
  }
  fft_windows_in_second++;
}

// --- Aggregate ring helpers --------------------------------------------------
static void push_ring_1m(float lin)  { laeq_ring_1m[ring_idx_1m] = lin; }
static void push_ring_15m(float lin) { laeq_ring_15m[ring_idx_15m] = lin; }
static void push_ring_30m(float lin) { laeq_ring_30m[ring_idx_30m] = lin; }

static float compute_leq_from_ring(const float* ring, int size, int valid) {
  if (valid <= 0) return 0.0f;
  double sum = 0.0;
  for (int i = 0; i < valid; i++) sum += ring[i];
  if (sum <= 0.0) return 0.0f;
  return 10.0f * log10f((float)(sum / (double)valid));
}

uint8_t audio_dsp_get_laeq_1m(uint16_t* sec_out) {
  int valid = total_seconds < RING_1M ? total_seconds : RING_1M;
  if (sec_out) *sec_out = (uint16_t)valid;
  return encode_db_to_byte(compute_leq_from_ring(laeq_ring_1m, RING_1M, valid));
}
uint8_t audio_dsp_get_laeq_15m(uint16_t* sec_out) {
  int valid = total_seconds < RING_15M ? total_seconds : RING_15M;
  if (sec_out) *sec_out = (uint16_t)valid;
  return encode_db_to_byte(compute_leq_from_ring(laeq_ring_15m, RING_15M, valid));
}
uint8_t audio_dsp_get_laeq_30m(uint16_t* sec_out) {
  int valid = total_seconds < RING_30M ? total_seconds : RING_30M;
  if (sec_out) *sec_out = (uint16_t)valid;
  return encode_db_to_byte(compute_leq_from_ring(laeq_ring_30m, RING_30M, valid));
}
uint8_t audio_dsp_get_lceq_1m(uint16_t* sec_out) {
  int valid = total_seconds < RING_1M ? total_seconds : RING_1M;
  if (sec_out) *sec_out = (uint16_t)valid;
  return encode_db_to_byte(compute_leq_from_ring(lceq_ring_1m, RING_1M, valid));
}
uint8_t audio_dsp_get_lceq_15m(uint16_t* sec_out) {
  int valid = total_seconds < RING_15M ? total_seconds : RING_15M;
  if (sec_out) *sec_out = (uint16_t)valid;
  return encode_db_to_byte(compute_leq_from_ring(lceq_ring_15m, RING_15M, valid));
}
uint8_t audio_dsp_get_lafmax_1m(uint16_t* sec_out) {
  int valid = total_seconds < RING_1M ? total_seconds : RING_1M;
  if (sec_out) *sec_out = (uint16_t)valid;
  if (valid <= 0) return 0;
  float mx = 0.0f;
  for (int i = 0; i < valid; i++) if (lafmax_ring_1m[i] > mx) mx = lafmax_ring_1m[i];
  return encode_db_to_byte(mx);  // ring stores dB
}
uint8_t audio_dsp_get_lcpeak_1m(uint16_t* sec_out) {
  int valid = total_seconds < RING_1M ? total_seconds : RING_1M;
  if (sec_out) *sec_out = (uint16_t)valid;
  if (valid <= 0) return 0;
  float mx = 0.0f;
  for (int i = 0; i < valid; i++) if (lcpeak_ring_1m[i] > mx) mx = lcpeak_ring_1m[i];
  return encode_db_to_byte(mx);
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
  float band_db[NOISE_BANDS];

  // Energy mean per band → dB. A fixed reference offset converts the
  // numerical FFT magnitude to dB SPL via calibration; here we just convert
  // the linear energy to dB-relative-to-ref and let the cal offset map it
  // onto an SPL scale.
  for (int b = 0; b < NOISE_BANDS; b++) {
    double mean = (fft_windows_in_second > 0)
        ? band_energy_sum[b] / (double)fft_windows_in_second
        : 0.0;
    float db = (mean > 0.0) ? 10.0f * log10f((float)mean) : -100.0f;
    band_db[b] = db + cal;
  }

  // LAeq,1s and LCeq,1s via spec-appendix weighting tables.
  double a_sum_lin = 0.0, c_sum_lin = 0.0;
  for (int b = 0; b < NOISE_BANDS; b++) {
    a_sum_lin += pow(10.0, (band_db[b] + a_weight_db[b]) / 10.0);
    c_sum_lin += pow(10.0, (band_db[b] + c_weight_db[b]) / 10.0);
  }
  float laeq_db = (a_sum_lin > 0.0) ? 10.0f * log10f((float)a_sum_lin) : 0.0f;
  float lceq_db = (c_sum_lin > 0.0) ? 10.0f * log10f((float)c_sum_lin) : 0.0f;

  // Fast (125 ms) time-weighted max and true peak should come from a
  // per-sample biquad cascade — not implemented yet. Approximate Fast-max
  // as Leq (slightly biased low) and peak as the unweighted absolute peak.
  float lcpeak_db = (peak_abs_sample_in_second > 0.0f)
      ? 20.0f * log10f(peak_abs_sample_in_second) + cal
      : 0.0f;
  float lafmax_db = laeq_db;
  float lcfmax_db = lceq_db;

  EventBits_t bits = xEventGroupGetBits(event_group);
  bool calibrated = (bits & CALIBRATED) != 0;
  bool time_set   = (bits & TIME_SET) != 0;

  // Update aggregate rings (in linear energy for Leq, raw dB for max/peak).
  float laeq_lin = (float)pow(10.0, laeq_db / 10.0);
  float lceq_lin = (float)pow(10.0, lceq_db / 10.0);
  laeq_ring_1m[ring_idx_1m]   = laeq_lin;
  lceq_ring_1m[ring_idx_1m]   = lceq_lin;
  lafmax_ring_1m[ring_idx_1m] = lafmax_db;
  lcpeak_ring_1m[ring_idx_1m] = lcpeak_db;
  ring_idx_1m = (ring_idx_1m + 1) % RING_1M;
  laeq_ring_15m[ring_idx_15m] = laeq_lin;
  lceq_ring_15m[ring_idx_15m] = lceq_lin;
  ring_idx_15m = (ring_idx_15m + 1) % RING_15M;
  laeq_ring_30m[ring_idx_30m] = laeq_lin;
  ring_idx_30m = (ring_idx_30m + 1) % RING_30M;
  if (total_seconds < RING_30M) total_seconds++;

  if (calibrated && time_set) {
    record_t r = { 0 };
    r.seq_no    = seq_no++;
    for (int b = 0; b < NOISE_BANDS; b++) {
      r.bands[b] = encode_db_to_byte(band_db[b]);
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
    ESP_LOGI(
        TAG, "NOT LOGGING — calibrated=%s, time_set=%s. Current readings (offset=%+.2f):",
        calibrated ? "yes" : "no", time_set ? "yes" : "no", cal
    );
    ESP_LOGI(TAG, "  LAeq   = %.1f dB(A)", laeq_db);
    ESP_LOGI(TAG, "  LCeq   = %.1f dB(C)", lceq_db);
    ESP_LOGI(TAG, "  LAFmax = %.1f dB(A)", lafmax_db);
    ESP_LOGI(TAG, "  LCpeak = %.1f dB(C)", lcpeak_db);
    if (!calibrated) {
      ESP_LOGI(TAG, "  To calibrate, send: CAL_SET <(reference - LAeq) * 100>");
    }
  }

  // Reset per-second accumulators
  for (int b = 0; b < NOISE_BANDS; b++) band_energy_sum[b] = 0.0;
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
      // ICS-43434 outputs 24-bit signed left-aligned in 32-bit slot.
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
