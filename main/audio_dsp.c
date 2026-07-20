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

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "heap_diag.h"
#include "pb_encode.h"
#include "power_management.h"

static const char* TAG = "audio_dsp";

// --- Optional: synthesize audio in software ---------------------------------
// Set to 1 to bypass the mic and inject a synthetic signal through the DSP
// pipeline (useful for testing the rest of the chain without a working mic).
#define SIMULATE_MIC 0

// --- I²S configuration --------------------------------------------------------
// ICS-43434: standard I²S mic, 24-bit signed samples left-aligned in a 32-bit slot.
// Tie L/R pin to GND for left channel (matches I2S_STD_SLOT_LEFT below).
#define SAMPLE_RATE      48000
#define I2S_BCLK_GPIO    GPIO_NUM_9    // SCK on the mic
#define I2S_WS_GPIO      GPIO_NUM_10   // WS on the mic
#define I2S_DATA_GPIO    GPIO_NUM_11   // SD on the mic
#define I2S_BUFFER_FRAMES 1024         // per i2s_channel_read call

// --- FFT configuration --------------------------------------------------------
// Radix-2, hardware SIMD (aes3) — see the dsps_fft2r_fc32_aes3 call site for why
// the SIMD path is correct once fft_work is aligned. The radix-2 twiddle table is
// only FFT_SIZE floats (16 KB) and lives in internal BSS (fft_table).
// Radix-4 SIMD was benchmarked (2026-07): ~16 % faster per FFT (2875 vs 3427 µs)
// but its twiddle table is 4×FFT_SIZE = 64 KB, and putting that in internal RAM
// starved the BLE controller (esp_bt_controller_init -4 NO_MEM). The FFT is only
// ~8 % of one core at 160 MHz either way, so the 64 KB (or a PSRAM twiddle that
// would erase the speedup) isn't worth it — radix-2 SIMD stays.
#define FFT_SIZE 4096
#define FFT_HOP  2048     // 50% overlap

// --- dB SPL anchor ------------------------------------------------------------
// The pipeline produces un-normalized FFT energy (no 1/N, no Hann-gain
// correction) on an arbitrary scale. The constant that maps it to true dB SPL
// is computed analytically rather than dialed in by hand:
//
//   energy_mean ≈ (N/2)·(3N/8)·RMS²            (single-sided Parseval × ΣHann²)
//   dB_SPL = 10·log10(energy_mean) + C_energy
//   C_energy = MIC_DBFS_TO_SPL + 10log10(2) − 10log10(3·N²/16)   (≈ +58.0 @ N=4096)
//   C_peak   = MIC_DBFS_TO_SPL + 20log10(√2)                     (≈ +123.0)
//
// MIC_DBFS_TO_SPL: INMP441 datasheet sensitivity −26 dBFS @ 94 dB SPL ⇒
// 0 dBFS = 120 dB SPL. The 3.01 dB terms are the full-scale-sine crest factor
// (0 dBFS is defined against a sine of RMS 1/√2). Per-band calibration trims
// the residual on top of these. Computed in dsp_init() so they track FFT_SIZE.
#define MIC_DBFS_TO_SPL_DB 120.0f

// --- Aggregate ring size — 30 min holds enough seconds for 5-min and 30-min
//     sliding-window Leq computations from the same buffer.
// 1800 entries × 2 channels × 4 B = 14.4 KB. The rings (laeq_ring/lceq_ring)
// live in PSRAM via EXT_RAM_BSS_ATTR — they're written and read once per second,
// so PSRAM latency is irrelevant, and keeping them out of internal DRAM leaves
// room for the BLE controller and a second TLS session. has_30m goes true after
// 30 min of uptime; laeq_30m / lceq_30m are emitted then.
#define RING_30M 1800

// --- Center frequencies (Hz) per band, spec §5 --------------------------------
static const float band_centers[NOISE_BANDS] = {
    16, 20, 25, 31.5f, 40, 50, 63, 80, 100, 125,
    160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250,
    1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000, 12500,
    16000
};

// Per-band frequency-response correction for the INMP441 chain, in dB
// (added in dB-space; positive = "this band reads low, push it up").
// Folded into the per-bin A/C weights and the band-sum multiplier, so it
// corrects the 31-band display, LAeq, LCeq, LAFmax, and LCFmax. This is now
// the ONLY calibration knob (the old scalar offset is gone — the fixed
// FFT-energy->dB-SPL anchor lives in dsp_init). Loaded from NVS at init and
// re-loaded whenever a new calibration arrives over BLE (see reload_band_cal).
// LCpeak only gets the mean of these (it's a time-domain metric).
// All zeros = no correction (raw values already near dB SPL via the anchor).
static float band_cal_offset_db[NOISE_BANDS] = { 0 };

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

// 16-byte aligned: the aes3 SIMD FFT uses ee.ldf.64/ee.stf.64 paired loads that
// require ≥8-byte-aligned data. A plain (4-byte) array made those loads straddle
// into adjacent BSS — the source of the "im=1.6 from zero input" artifact.
static float    fft_work[FFT_SIZE * 2] __attribute__((aligned(16)));  // complex pairs for FFT in/out
// Twiddle-factor table for dsps_fft2r_init_fc32 (16 KB at FFT_SIZE=4096).
// Internal BSS (link-time reserved), 16-byte aligned for esp-dsp's SIMD loads.
// Read on every FFT, so it stays in internal RAM (not PSRAM); being a static
// array (not an early heap grab) is what let audio_dsp_preinit() go away.
static float    fft_table[FFT_SIZE] __attribute__((aligned(16)));
static int      band_start_bin[NOISE_BANDS + 1];   // inclusive start; one extra = exclusive end of last band
static float    band_cal_lin[NOISE_BANDS];          // 10^(band_cal_offset_db / 10)

// dB-SPL anchor constants, computed once in dsp_init (see MIC_DBFS_TO_SPL_DB).
static float    fft_energy_to_spl_db;   // ≈ +58.0 — added to every energy->dB
static float    peak_to_spl_db;         // ≈ +123.0 — LCpeak amplitude->dB

// Per-bin A and C weighting (linear power factors), populated at init from
// IEC 61672 formulas. 8 KB each, internal BSS. Read on every FFT in the band
// accumulation loop, so they stay in internal RAM with the rest of the FFT hot
// path. Avoid the band-center approximation for LAeq/LCeq — apply weighting at
// each bin's exact frequency instead.
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
static int      fft_windows_in_second = 0;       // fft_worker-owned
static float    peak_abs_sample_in_second = 0.0f; // reader-owned
// Drives wall-clock-paced WORK_EMIT (1 Hz). Sample-counted emission
// drifted because the old inline FFT blocked I²S reads.
static int64_t  next_emit_us = 0;

// Reader → fft_worker queue. One FIFO carries both FFT and EMIT
// messages so emit always sees every FFT queued before it.
typedef enum {
  DSP_WORK_FFT,
  DSP_WORK_EMIT,
} dsp_work_type_t;

typedef struct {
  dsp_work_type_t type;
  int   fft_ring_head_snapshot;  // FFT: ring index at trigger time
  float peak_abs;                // EMIT: reader's snapshotted peak
} dsp_work_t;

static QueueHandle_t dsp_work_queue;

// 30-min sliding LAeq/LCeq ring buffers (linear energies, not dB).
// Same buffers serve the 5-minute window via compute_leq_recent(..., 300).
// In PSRAM (EXT_RAM_BSS_ATTR): large, cold (one write + one read per second).
EXT_RAM_BSS_ATTR static float laeq_ring[RING_30M];
EXT_RAM_BSS_ATTR static float lceq_ring[RING_30M];
static int      ring_idx = 0;
static int      total_seconds = 0;

// Pre-computed Hann window. The original code computed this on the fly
// (cosf in the FFT inner loop). cosf lives in newlib (flash); during a
// record_writer flash erase, the cache-disabled window resolves cosf to
// a bogus address and the audio_dsp task panics on the next FFT. Caching
// the window in BSS keeps the hot path entirely off flash for this term,
// and also saves ~9 ms/sec of CPU.
static float    hann_window[FFT_SIZE];

static uint32_t seq_no = 0;

// --- On-disk minute aggregation ----------------------------------------------
// The file log stores ONE energy-aggregated record per RECORD_INTERVAL_SECONDS
// (see constants.h). These accumulate the per-second values; at the interval
// boundary we emit one aggregate record_t to record_writer_queue only. MQTT/BLE
// keep getting the per-second record every second. Aggregation is in the energy
// (linear) domain for the Leqs and bands; peaks take the interval max.
static double   min_laeq_energy_sum = 0.0;
static double   min_lceq_energy_sum = 0.0;
static double   min_band_energy_sum[NOISE_BANDS];  // linear, pre-anchor
static float    min_lafmax_db = 0.0f;
static float    min_lcfmax_db = 0.0f;
static float    min_lcpeak_db = 0.0f;
static int      min_seconds_count = 0;
static uint32_t minute_seq_no = 0;
static time_t   min_window_start = 0;  // wall-clock time of this interval's first second

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

// Fold per-band cal into the per-bin A/C weights and pre-compute the
// linear-power factor used by the band-sum multiplier. Each bin gets
// the highest-indexed band whose [start, next_start) range covers it
// — overlaps at low frequencies resolve to last-band-wins. Bins above
// the top band's edge are left untouched.
static void apply_band_cal(void) {
  for (int b = 0; b < NOISE_BANDS; b++) {
    band_cal_lin[b] = powf(10.0f, band_cal_offset_db[b] / 10.0f);
  }
  int b = 0;
  int top = band_start_bin[NOISE_BANDS];
  for (int k = 1; k < FFT_SIZE / 2; k++) {
    if (k >= top) break;
    while (b + 1 < NOISE_BANDS && band_start_bin[b + 1] <= k) b++;
    a_weight_bin[k] *= band_cal_lin[b];
    c_weight_bin[k] *= band_cal_lin[b];
  }
}

// (Re)load per-band calibration from the calibration module and fold it into
// the per-bin weights. apply_band_cal() multiplies the cal into a_weight_bin/
// c_weight_bin destructively, so the base weights must be rebuilt from scratch
// first — compute_bin_weights() is idempotent. Runs in the DSP/FFT-worker task
// only (init + the once-per-second dirty check), so it never races the FFT
// accumulation, which runs in the same task.
static void reload_band_cal(void) {
  calibration_get_bands_db(band_cal_offset_db, NOISE_BANDS);
  compute_bin_weights();
  apply_band_cal();
}

// Mean of the per-band offsets (dB) — applied to LCpeak, which is time-domain
// and can't carry a per-band correction.
static float band_cal_mean_db(void) {
  float sum = 0.0f;
  for (int b = 0; b < NOISE_BANDS; b++) sum += band_cal_offset_db[b];
  return sum / (float)NOISE_BANDS;
}

static void dsp_init(void) {
  esp_err_t err = dsps_fft2r_init_fc32(fft_table, FFT_SIZE);
  if (err != ESP_OK && err != ESP_ERR_DSP_REINITIALIZED) {
    ESP_LOGE(TAG, "dsps_fft2r_init_fc32 failed: 0x%x", err);
    ESP_ERROR_CHECK(err);
  }
  // dB-SPL anchor: un-inflate the FFT-energy scale and add INMP441 sensitivity.
  fft_energy_to_spl_db = MIC_DBFS_TO_SPL_DB + 10.0f * log10f(2.0f)
      - 10.0f * log10f(3.0f * (float)FFT_SIZE * (float)FFT_SIZE / 16.0f);
  peak_to_spl_db = MIC_DBFS_TO_SPL_DB + 20.0f * log10f(sqrtf(2.0f));
  ESP_LOGI(TAG, "SPL anchor: energy %+.2f dB, peak %+.2f dB",
           fft_energy_to_spl_db, peak_to_spl_db);

  compute_band_edges();
  reload_band_cal();   // compute_bin_weights() + fold in NVS per-band cal

  const float hann_k = 2.0f * (float)M_PI / (float)FFT_SIZE;
  for (int i = 0; i < FFT_SIZE; i++) {
    hann_window[i] = 0.5f * (1.0f - cosf((float)i * hann_k));
  }
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
//
// head_snapshot is the value of fft_ring_head at the moment the reader
// posted WORK_FFT. The reader keeps writing fft_ring while the FFT
// runs; as long as the FFT completes before the reader writes another
// FFT_SIZE - FFT_HOP = 2048 samples (~43 ms at 48 kHz), the window
// pointed at by head_snapshot is untouched. ANSI FFT on this S3 runs
// well under that bound.
static void run_fft_and_accumulate(int head_snapshot) {
  for (int i = 0; i < FFT_SIZE; i++) {
    int idx = (head_snapshot + i) & (FFT_SIZE - 1);
    fft_work[2 * i]     = fft_ring[idx] * hann_window[i];
    fft_work[2 * i + 1] = 0.0f;
  }

  // aes3 SIMD FFT. The old "im=1.6 from zero input" artifact that once forced us
  // onto the ANSI variant was NOT an assembly defect — it was an alignment bug on
  // our side: the aes3 kernel uses ee.ldf.64/ee.stf.64 paired loads that need
  // 8-byte-aligned data, and fft_work was a plain 4-byte-aligned array, so the
  // loads straddled into adjacent BSS. With fft_work now __attribute__((aligned
  // (16))) the SIMD output matches ANSI exactly (verified on-device 2026-07,
  // quiet-room LAeq identical). bit_rev stays ANSI (dsps_bit_rev_fc32 resolves to
  // _ansi even under DSP_OPTIMIZED — there is no wired-up SIMD bit-reverse here).
  dsps_fft2r_fc32_aes3(fft_work, FFT_SIZE);
  dsps_bit_rev_fc32_ansi(fft_work, FFT_SIZE);

  // Hann window's global power gain (0.375) is absorbed into the calibration
  // offset — no per-frame compensation needed.

  // Accumulate magnitude-squared into bands (for the 31-band per-second display).
  // Multiply by band_cal_lin[b] to apply the per-band mic-response cal in
  // linear-power space — equivalent to adding band_cal_offset_db[b] in dB.
  for (int b = 0; b < NOISE_BANDS; b++) {
    int start = band_start_bin[b];
    int end = band_start_bin[b + 1];
    if (end <= start) end = start + 1;
    if (end > FFT_SIZE / 2) end = FFT_SIZE / 2;
    // float inner loop: the S3 FPU is single-precision, so double math here is
    // software-emulated (~4× the whole FFT's cost when this was double). float32
    // round-off over a band's bins is ~1e-4 relative ≪ our 0.5 dB output step.
    float sum = 0.0f;
    for (int k = start; k < end; k++) {
      float re = fft_work[2 * k];
      float im = fft_work[2 * k + 1];
      sum += re * re + im * im;
    }
    band_energy_sum[b] += (double)sum * (double)band_cal_lin[b];
  }

  // Per-bin A/C weighted accumulation for LAeq/LCeq. Applying the weight at
  // each bin's exact frequency (vs. at band centers) eliminates the
  // frequency-quantization error that overestimates tones between band
  // centers — especially at low frequencies where bands are only a few bins
  // wide.
  //
  // Restrict the sum to the calibrated band range [16 Hz, 16 kHz). Bins above
  // the top band edge carry the INMP441's uncalibrated HF resonance (apply_band_cal
  // only folds cal up to band_start_bin[NOISE_BANDS]); at 48 kHz sampling that
  // resonance sits near 18-22 kHz where A-weighting only attenuates ~10 dB, so
  // leaving it in inflated LAeq by ~7 dB above the (calibrated) band sum.
  float a_sum = 0.0f, c_sum = 0.0f;
  for (int k = band_start_bin[0]; k < band_start_bin[NOISE_BANDS]; k++) {
    float re = fft_work[2 * k];
    float im = fft_work[2 * k + 1];
    float bin_energy = re * re + im * im;
    a_sum += bin_energy * a_weight_bin[k];
    c_sum += bin_energy * c_weight_bin[k];
  }
  a_weighted_sum_per_sec += a_sum;
  c_weighted_sum_per_sec += c_sum;
  if (a_sum > a_weighted_max_per_fft) a_weighted_max_per_fft = a_sum;
  if (c_sum > c_weighted_max_per_fft) c_weighted_max_per_fft = c_sum;

  fft_windows_in_second++;
}

// --- Aggregate ring helpers --------------------------------------------------
#define WINDOW_5M_SEC  300
#define WINDOW_30M_SEC 1800

// Walk the 30-min ring once, accumulating both the last-300-entry sum
// (5-min window) and the last-1800-entry sum (30-min window). Halves the
// per-second cost vs. two independent passes.
static void leq_pair_sum(const float* ring, int head,
                        double* sum_5m, double* sum_30m) {
  double s5 = 0.0, s30 = 0.0;
  int n5  = total_seconds < WINDOW_5M_SEC  ? total_seconds : WINDOW_5M_SEC;
  int n30 = total_seconds < WINDOW_30M_SEC ? total_seconds : WINDOW_30M_SEC;
  for (int i = 1; i <= n30; i++) {
    int idx = (head - i + RING_30M) % RING_30M;
    float v = ring[idx];
    s30 += v;
    if (i <= n5) s5 += v;
  }
  *sum_5m  = s5;
  *sum_30m = s30;
}

static uint8_t leq_from_sum(double sum, int n) {
  if (n <= 0 || sum <= 0.0) return 0;
  return encode_db_to_byte(10.0f * log10f((float)(sum / (double)n)));
}

void audio_dsp_get_aggregates(audio_dsp_aggregates_t* out) {
  double a5, a30, c5, c30;
  leq_pair_sum(laeq_ring, ring_idx, &a5, &a30);
  leq_pair_sum(lceq_ring, ring_idx, &c5, &c30);
  int n5  = total_seconds < WINDOW_5M_SEC  ? total_seconds : WINDOW_5M_SEC;
  int n30 = total_seconds < WINDOW_30M_SEC ? total_seconds : WINDOW_30M_SEC;
  out->laeq_5m  = leq_from_sum(a5,  n5);
  out->lceq_5m  = leq_from_sum(c5,  n5);
  out->laeq_30m = leq_from_sum(a30, n30);
  out->lceq_30m = leq_from_sum(c30, n30);
  out->has_5m   = (n5  >= WINDOW_5M_SEC);
  out->has_30m  = (n30 >= WINDOW_30M_SEC);
}

// --- Shared record → protobuf helpers ----------------------------------------
void record_to_pb(const record_t* r, NoiseRecording* out) {
  out->seq_no = r->seq_no;
  memcpy(out->bands, r->bands, sizeof(r->bands));
  out->laeq   = r->laeq;
  out->lceq   = r->lceq;
  out->lafmax = r->lafmax;
  out->lcfmax = r->lcfmax;
  out->lcpeak = r->lcpeak;
}

// Stamp the current 5m/30m rolling-window Leqs onto `out`. has_ flags follow
// the ring-full gate (a partial window would average fewer seconds than the
// field implies). Shared by the live (MQTT/BLE) and on-disk encoders.
void record_apply_aggregates(NoiseRecording* out) {
  audio_dsp_aggregates_t agg;
  audio_dsp_get_aggregates(&agg);
  out->has_laeq_5m  = agg.has_5m;  out->laeq_5m  = agg.laeq_5m;
  out->has_lceq_5m  = agg.has_5m;  out->lceq_5m  = agg.lceq_5m;
  out->has_laeq_30m = agg.has_30m; out->laeq_30m = agg.laeq_30m;
  out->has_lceq_30m = agg.has_30m; out->lceq_30m = agg.lceq_30m;
}

size_t record_encode_single(const record_t* r, uint8_t* out, size_t cap) {
  // The flattened NoiseRecording is tiny (~50 B), so we can stage and encode
  // the whole message on the stack — no more field-by-field hand-encoding.
  NoiseRecording rec = NoiseRecording_init_zero;
  record_to_pb(r, &rec);
  rec.record_interval_seconds = 1;  // live per-second sample

  // battery_mv is only meaningful off-USB; skip when charging.
  if (usb_voltage <= 1000 && battery_voltage > 0) {
    rec.has_battery_mv = true;
    rec.battery_mv = (uint32_t)battery_voltage;
  }
  record_apply_aggregates(&rec);

  pb_ostream_t stream = pb_ostream_from_buffer(out, cap);
  if (!pb_encode(&stream, NoiseRecording_fields, &rec)) {
    ESP_LOGW(TAG, "record_encode_single: %s", stream.errmsg);
    return 0;
  }
  return stream.bytes_written;
}

// --- Per-second emission -----------------------------------------------------
//
// Derive band-dB from accumulated energy. Apply calibration offset.
// LAeq,1s and LCeq,1s are computed from band dB via the IEC 61672 tables
// (spec Appendix). LAFmax/LCfmax/LCpeak are approximated from the FFT band
// energies in v1 — a Phase E refinement will add per-sample biquad filtering
// with proper Fast time-weighting (125 ms exponential smoother).
// peak_abs is snapshotted by the reader before posting WORK_EMIT — see
// dsp_work_t. The other accumulators are owned by this task.
static void emit_per_second(float peak_abs) {
  // Re-fold per-band calibration if it changed (BLE write / clear). Done here,
  // in the DSP/FFT-worker task, so the weight-array rebuild can't race the FFT
  // accumulation. Energy accumulated this second used the OLD weights, but the
  // change takes full effect from the next second — acceptable for a manual,
  // infrequent calibration action.
  if (calibration_take_band_dirty()) {
    reload_band_cal();
    ESP_LOGI(TAG, "per-band calibration re-applied");
  }

  // LAeq,1s and LCeq,1s from per-bin-weighted FFT energy (accumulated in
  // run_fft_and_accumulate). This applies the IEC 61672 A/C filter at each
  // bin's actual frequency, eliminating the band-center quantization error.
  // fft_energy_to_spl_db anchors the un-normalized FFT energy to dB SPL.
  double a_mean = (fft_windows_in_second > 0)
      ? a_weighted_sum_per_sec / (double)fft_windows_in_second
      : 0.0;
  double c_mean = (fft_windows_in_second > 0)
      ? c_weighted_sum_per_sec / (double)fft_windows_in_second
      : 0.0;
  float laeq_db = (a_mean > 0.0) ? 10.0f * log10f((float)a_mean) + fft_energy_to_spl_db : 0.0f;
  float lceq_db = (c_mean > 0.0) ? 10.0f * log10f((float)c_mean) + fft_energy_to_spl_db : 0.0f;

  // LAFmax/LCFmax: max single-FFT A/C-weighted energy seen this second.
  // 85 ms FFT window approximates Fast time-weighting; not a perfect IEC
  // 61672 implementation (no exponential smoother) but captures intra-second
  // peaks that the per-second average can't.
  float lafmax_db = (a_weighted_max_per_fft > 0.0)
      ? 10.0f * log10f((float)a_weighted_max_per_fft) + fft_energy_to_spl_db : 0.0f;
  float lcfmax_db = (c_weighted_max_per_fft > 0.0)
      ? 10.0f * log10f((float)c_weighted_max_per_fft) + fft_energy_to_spl_db : 0.0f;
  // LCpeak: unweighted absolute sample peak. peak_to_spl_db maps the raw
  // [-1,1] sample-amplitude scale directly to dB SPL (it's a different scale
  // from FFT energy). Per-band cal can't apply to a time-domain metric, so we
  // add the mean band offset to keep it tracking the overall sensitivity trim.
  // (Still not C-weighted — proper LCpeak needs an IIR C-weight filter on
  // the time-domain samples before the peak detector.)
  const float lcpeak_offset = peak_to_spl_db + band_cal_mean_db();
  float lcpeak_db = (peak_abs > 0.0f)
      ? 20.0f * log10f(peak_abs) + lcpeak_offset
      : 0.0f;

  EventBits_t bits = xEventGroupGetBits(event_group);
  bool time_set = (bits & TIME_SET) != 0;

  // Update the 30-min sliding rings (linear energy).
  float laeq_lin = (float)pow(10.0, laeq_db / 10.0);
  float lceq_lin = (float)pow(10.0, lceq_db / 10.0);
  laeq_ring[ring_idx] = laeq_lin;
  lceq_ring[ring_idx] = lceq_lin;
  ring_idx = (ring_idx + 1) % RING_30M;
  if (total_seconds < RING_30M) total_seconds++;

  if (time_set) {
    record_t r = { 0 };
    r.seq_no    = seq_no++;
    for (int b = 0; b < NOISE_BANDS; b++) {
      double mean = (fft_windows_in_second > 0)
          ? band_energy_sum[b] / (double)fft_windows_in_second
          : 0.0;
      float db = (mean > 0.0) ? 10.0f * log10f((float)mean) + fft_energy_to_spl_db : 0.0f;
      r.bands[b] = encode_db_to_byte(db);
      // Accumulate the per-second linear band energy for the minute average.
      // `mean` is already that linear value (computed above), so there's no
      // pow() to reconstruct it; the constant SPL anchor is added once at emit.
      min_band_energy_sum[b] += mean;
    }
    r.laeq   = encode_db_to_byte(laeq_db);
    r.lceq   = encode_db_to_byte(lceq_db);
    r.lafmax = encode_db_to_byte(lafmax_db);
    r.lcfmax = encode_db_to_byte(lcfmax_db);
    r.lcpeak = encode_db_to_byte(lcpeak_db);

    // Live 1 Hz fan-out to MQTT + BLE (cadence unchanged).
    if (xQueueSend(mqtt_publisher_queue, &r, 0) != pdTRUE) {
      ESP_LOGW(TAG, "mqtt_publisher_queue full, dropping record %lu", (unsigned long)r.seq_no);
    }
    if (xQueueSend(ble_publisher_queue, &r, 0) != pdTRUE) {
      ESP_LOGW(TAG, "ble_publisher_queue full, dropping record %lu", (unsigned long)r.seq_no);
    }

    // On-disk log: accumulate this second, emit ONE aggregate to record_writer
    // every RECORD_INTERVAL_SECONDS so a power loss costs at most one interval.
    if (min_seconds_count == 0) min_window_start = time(NULL);  // true window start
    min_laeq_energy_sum += laeq_lin;      // linear energy (anchor baked in)
    min_lceq_energy_sum += lceq_lin;
    min_lafmax_db = fmaxf(min_lafmax_db, lafmax_db);  // peaks: max over interval
    min_lcfmax_db = fmaxf(min_lcfmax_db, lcfmax_db);
    min_lcpeak_db = fmaxf(min_lcpeak_db, lcpeak_db);
    min_seconds_count++;

    if (min_seconds_count >= RECORD_INTERVAL_SECONDS) {
      record_t agg = { 0 };
      agg.seq_no = minute_seq_no++;
      agg.window_start = min_window_start;
      for (int b = 0; b < NOISE_BANDS; b++) {
        double m = min_band_energy_sum[b] / (double)min_seconds_count;
        agg.bands[b] = (m > 0.0)
            ? encode_db_to_byte(10.0f * log10f((float)m) + fft_energy_to_spl_db)
            : 0;
      }
      agg.laeq   = leq_from_sum(min_laeq_energy_sum, min_seconds_count);
      agg.lceq   = leq_from_sum(min_lceq_energy_sum, min_seconds_count);
      agg.lafmax = encode_db_to_byte(min_lafmax_db);
      agg.lcfmax = encode_db_to_byte(min_lcfmax_db);
      agg.lcpeak = encode_db_to_byte(min_lcpeak_db);

      if (xQueueSend(record_writer_queue, &agg, 0) != pdTRUE) {
        ESP_LOGW(TAG, "record_writer_queue full, dropping aggregate %lu",
                 (unsigned long)agg.seq_no);
      }

      min_laeq_energy_sum = 0.0;
      min_lceq_energy_sum = 0.0;
      for (int b = 0; b < NOISE_BANDS; b++) min_band_energy_sum[b] = 0.0;
      min_lafmax_db = 0.0f;
      min_lcfmax_db = 0.0f;
      min_lcpeak_db = 0.0f;
      min_seconds_count = 0;
    }
  } else {
    ESP_LOGI(TAG, "waiting for time_set; current LAeq=%.1f LCeq=%.1f (mean band cal=%+.2f)",
             laeq_db, lceq_db, band_cal_mean_db());
  }

  // Reset per-second accumulators owned by this task. Reader resets
  // its own (peak_abs_sample_in_second) at the WORK_EMIT post site.
  for (int b = 0; b < NOISE_BANDS; b++) band_energy_sum[b] = 0.0;
  a_weighted_sum_per_sec = 0.0;
  c_weighted_sum_per_sec = 0.0;
  a_weighted_max_per_fft = 0.0;
  c_weighted_max_per_fft = 0.0;
  fft_windows_in_second = 0;
}

// --- FFT worker task --------------------------------------------------------
static void fft_worker(void* params) {
  (void)params;
  dsp_work_t w;
  while (true) {
    if (xQueueReceive(dsp_work_queue, &w, portMAX_DELAY) != pdTRUE) continue;
    switch (w.type) {
      case DSP_WORK_FFT:
        run_fft_and_accumulate(w.fft_ring_head_snapshot);
        break;
      case DSP_WORK_EMIT:
        emit_per_second(w.peak_abs);
        break;
    }
  }
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

  // Spawn the FFT worker. Same core as the reader (Core 1) but lower
  // priority so the reader always preempts to service I²S — the worker
  // gets the gaps. 8 deep covers ~340 ms of buffered work; if it fills
  // the reader logs and drops a message rather than blocking I²S.
  // 6 KB stack covers emit_per_second's printf + three xQueueSends with
  // margin (record_writer hit 4 KB exactly).
  dsp_work_queue = xQueueCreate(8, sizeof(dsp_work_t));
  xTaskCreatePinnedToCore(&fft_worker, "fft_worker", 6144, NULL, TASK_PRIO_NORMAL, NULL, 1);

  next_emit_us = esp_timer_get_time() + 1000000;

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

      float af = fabsf(f);
      if (af > peak_abs_sample_in_second) peak_abs_sample_in_second = af;

      // Append to FFT ring (circular). FFT_SIZE is a power of two so
      // the wrap is a single AND; signed `%` would compile to a real
      // division.
      fft_ring[fft_ring_head] = f;
      fft_ring_head = (fft_ring_head + 1) & (FFT_SIZE - 1);
      samples_since_last_fft++;

      // Hand the FFT off to the worker task every FFT_HOP samples.
      if (samples_since_last_fft >= FFT_HOP) {
        samples_since_last_fft = 0;
        dsp_work_t w = {
          .type = DSP_WORK_FFT,
          .fft_ring_head_snapshot = fft_ring_head,
        };
        if (xQueueSend(dsp_work_queue, &w, 0) != pdTRUE) {
          ESP_LOGW(TAG, "dsp_work_queue full, dropping FFT");
        }
      }
    }

    // Wall-clock-paced emission. Checked once per I²S read (every ~21 ms
    // when sample-paced) so it fires within one buffer of the deadline.
    // Snapshot+reset peak here so the worker reads the value from the
    // message and never races with the per-sample loop. If we fall
    // behind, resync to "now" rather than catch-up bursting.
    int64_t now_us = esp_timer_get_time();
    if (now_us >= next_emit_us) {
      dsp_work_t w = {
        .type = DSP_WORK_EMIT,
        .peak_abs = peak_abs_sample_in_second,
      };
      peak_abs_sample_in_second = 0.0f;
      next_emit_us = now_us + 1000000;
      if (xQueueSend(dsp_work_queue, &w, 0) != pdTRUE) {
        ESP_LOGW(TAG, "dsp_work_queue full, dropping EMIT");
      }
    }
  }
}
