# Festival Noise Monitor

Battery- or USB-powered ESP32-S3 device that continuously measures ambient SPL, computes per-second LAeq/LCeq plus 1/3-octave band levels, and publishes them over MQTT + BLE + LittleFS.

Full requirements live in [`NOISE_MONITOR_SPEC.md`](NOISE_MONITOR_SPEC.md). This README focuses on **how the implementation actually works and why** — the non-obvious decisions a future maintainer needs to know.

---

## Signal chain

```
INMP441 (I²S mic) ──► I²S RX peripheral ──► audio_dsp task
                                              │
                                              ├─► FFT (4096-pt radix-2)
                                              │
                                              ├─► per-bin A/C weighting ──► LAeq, LCeq
                                              │
                                              ├─► 1/3-octave band aggregation ──► 31 band SPLs
                                              │
                                              ├─► per-FFT A/C weighted max ──► LAFmax, LCFmax
                                              ├─► broadband sample peak    ──► LCpeak
                                              │
                                              └─► record_t (1 Hz) ──► MQTT queue
                                                                  ├─► BLE queue
                                                                  └─► record_writer (LittleFS)
```

Records go out every second containing 31 band values + 5 weighted summary metrics. The 5-minute and 30-minute sliding LAeq/LCeq aggregates are computed from in-RAM ring buffers (in PSRAM) and attached to every `NoiseRecording` message. `laeq_30m` / `lceq_30m` populate after 30 min of uptime.

---

## Hardware

| Component | Part / wiring |
|---|---|
| MCU | ESP32-S3-N16R2 (16 MB flash, 2 MB QSPI PSRAM) |
| Microphone | **InvenSense INMP441** (I²S, 3.3V-tolerant) |
| Pins | BCLK = GPIO 11, WS = GPIO 9, SD = GPIO 12; L/R tied to GND |

---

## DSP design

### Why FFT-based (not IIR filters)

A textbook Type-1 SLM uses cascaded biquad filters for A-weighting + 1/3-octave bandpass per band — accurate, but ~62 IIR filters running per sample at 48 kHz is heavy. We use a single 4096-pt FFT every 42 ms (50% overlap → ~23 FFTs/sec) and derive both A-weighting and bands from that. Far less CPU, accurate enough for festival monitoring.

### Why 4096-pt radix-2 SIMD (not radix-4)

The spec preferred radix-4 (faster). We use radix-2 with the **hardware SIMD** kernel (`dsps_fft2r_fc32_aes3`), which is faster than *ANSI* radix-4 while keeping the small 16 KB twiddle table (radix-4 needs ~64 KB). CPU isn't the constraint at 160 MHz, so radix-4 isn't worth the larger table.

### The SIMD FFT: an alignment requirement, not a bug

We previously believed `esp-dsp`'s SIMD `dsps_fft2r_fc32_aes3` on ESP32-S3 was broken — it emitted a constant `~1.6` in every imaginary bin from zero input. That was actually **our** bug: the aes3 kernel uses `ee.ldf.64` / `ee.stf.64` paired loads that require ≥8-byte-aligned data, and `fft_work` was a plain 4-byte-aligned array, so the loads straddled into adjacent BSS. With `fft_work` declared `__attribute__((aligned(16)))` the SIMD output matches ANSI exactly (verified on-device). We call:

```c
dsps_fft2r_fc32_aes3(fft_work, FFT_SIZE);   // fft_work must be ≥8-byte aligned
dsps_bit_rev_fc32_ansi(fft_work, FFT_SIZE); // no SIMD bit-reverse is wired up here
```

### Float, not double, in the DSP hot loops

The ESP32-S3 FPU is single-precision only — `double` math is software-emulated. The per-FFT band + A/C-weighting accumulation (~2048 bins, twice per FFT) was originally written with `double` and cost **12.8 ms/FFT**, ~3.7× the SIMD FFT itself. Converting those inner loops to `float` dropped it to **0.3 ms** (41× faster), taking the whole DSP from ~38% to ~9% of one core. float32 round-off over a few thousand bins is ~1e-4 relative — far below the 0.5 dB output quantization — and on-device LAeq was unchanged. Keep DSP hot-path math in `float`; reserve `double` for cold once-per-second aggregation.

### Why per-bin A-weighting (not per-band)

The 31 1/3-octave bands include some that are only 4–5 FFT bins wide at low frequencies. Applying A-weighting at the band center (the textbook simplification) introduces up to **4 dB of error** for tones falling between band centers — most visible with pure sine inputs around 200–500 Hz. We pre-compute the IEC 61672 A and C weights per FFT bin at startup, then weight each bin's energy by its true-frequency weight before summing for LAeq / LCeq. This eliminates the band-center quantization error.

For broadband signals (music, crowd noise) the error from band-center weighting is < 0.5 dB, so this matters more for compliance and edge cases (single sustained tones, feedback) than for typical festival audio. The cost is 16 KB BSS (2 × `FFT_SIZE/2` floats for A and C weights) and one mul-add per bin per FFT — negligible.

### Bands hold total power, not per-bin mean

Each 1/3-octave band's value is the sum of its bins' magnitude-squared — standard SPL semantics. Dividing by bin count would give a PSD-style value, which under-counts wide bands relative to narrow ones and yields a non-flat frequency response under broadband signals.

---

## Data format

### Wire (`noise.proto`)

```proto
message Record {
  uint32 seq_no    = 1;   // monotonic; gap detection
  bytes  bands     = 2;   // 31 bytes; (dB - 20) * 2 per band
  uint32 laeq_1s   = 3;   // same encoding as bands (uint8 on wire via int_size:IS_8)
  uint32 lceq_1s   = 4;   // C-weighted, per-bin
  uint32 lafmax_1s = 5;   // max A-weighted FFT energy in the second
  uint32 lcfmax_1s = 6;   // max C-weighted FFT energy in the second
  uint32 lcpeak_1s = 7;   // unweighted abs sample peak (v1 approximation)
}

message NoiseRecording {
  reserved 3, 4;                    // previously laeq_15m / lceq_15m

  repeated Record records      = 2; // 1 (MQTT/BLE live) or up to 300 (5-min file batch)
  optional uint32 battery_mv   = 5; // millivolts; only set on live messages when on battery
  optional uint32 laeq_5m      = 6; // last 5 min (same encoding); only set when ring is full
  optional uint32 lceq_5m      = 7;
  optional uint32 laeq_30m     = 8; // last 30 min; only set when ring is full
  optional uint32 lceq_30m     = 9;
}
```

Nanopb options (`noise.options`) downsize the uint32 dB fields to `uint8_t` in C — the wire format is still standard protobuf varint (1 byte for values 0–127, 2 bytes above), so this is purely a C-side memory optimization.

### Intermediate C struct (`audio_dsp.h`)

```c
typedef struct {
  uint32_t seq_no;
  uint8_t  bands[31];     // unweighted 1/3-octave band SPLs (16 Hz to 16 kHz)
  uint8_t  laeq_1s;       // A-weighted, per-bin weighted
  uint8_t  lceq_1s;       // C-weighted, per-bin weighted
  uint8_t  lafmax_1s;     // max A-weighted FFT energy over the 1 s (~85 ms window)
  uint8_t  lcfmax_1s;     // max C-weighted FFT energy over the 1 s (~85 ms window)
  uint8_t  lcpeak_1s;     // unweighted absolute sample peak (v1 approximation)
} record_t;
```

`record_t` is the queue payload between `audio_dsp` and the publishers. The mapping `record_t → Record` lives in **one place** (`audio_dsp.c:record_to_pb`) — MQTT, BLE, and the file batch writer all route through it, so adding/removing a field requires touching only one function.

### dB encoding (spec §5)

Each dB metric is encoded into a single byte:

```
byte = round((dB − 20) × 2)        # 0..255, clamped
dB   = 20 + byte / 2               # decode
```

- Range: 20.0 to 147.5 dB
- Resolution: 0.5 dB
- Floor at 20 dB matches typical ambient minimums
- A jet engine at 140 dB still fits

### Why send both bands and pre-computed LAeq

The 31 bands are **unweighted**, so downstream consumers (dashboards, server) can apply any weighting they want for spectral analysis. The `laeq_1s` / `lceq_1s` summary metrics are pre-computed with **per-bin** A-weighting accuracy — better than what a consumer could derive from the 31 aggregated bands (which lose within-band frequency detail and incur the band-center error we just fixed).

**Use `laeq_1s` from the record for the headline dB(A) number.** Only re-derive from bands if you need a different weighting scheme.

### Sliding-window Leqs at the recording level

`NoiseRecording.laeq_5m / lceq_5m / laeq_30m / lceq_30m` are sliding Leqs over the last 5 and 30 minutes. They share a single ring buffer per channel (`laeq_ring[RING_30M]` and `lceq_ring[RING_30M]`); a single-pass walk via `audio_dsp_get_aggregates` accumulates both window sums at once.

`RING_30M = 1800` (full 30-min window). The rings live in PSRAM (`EXT_RAM_BSS_ATTR`), so both the 5-min and 30-min windows populate; `has_30m` goes true after 30 min of uptime.

All four fields are `optional` and are **only set when the ring holds the full window of data** — 5-min fields appear after the device has been running 5+ minutes; below that threshold the average would be over fewer samples than the field name implies, so the device omits them entirely (consumers see "missing" rather than misleading).

Each MQTT/BLE message snapshots the current values; the file batch writer writes them when the batch is flushed. Consumers don't need to maintain their own sliding window across reconnects/drops.

---

## BLE

MQTT and BLE carry **the same bytes**: both publish the encoded `NoiseRecording` protobuf at 1 Hz, so a consumer app uses one decoder and picks whichever transport is reachable.

### Characteristics

NimBLE peripheral, custom 128-bit base UUID `7ed2f2c4-69e8-4f7c-9c93-7a3b1e5d0a00`. Three characteristics under it:

| Suffix | Properties | Payload |
|---|---|---|
| `…0a01` `record`        | READ, NOTIFY | Encoded `NoiseRecording` protobuf (same bytes as MQTT). 1 Hz notify when subscribed. |
| `…0a02` `cal_offset`    | READ, WRITE  | int32 LE, hundredths of dB. Persisted to NVS on write. |
| `…0a03` `wifi_creds`    | WRITE only   | `[u8 ssid_len][ssid][u8 pw_len][pw]`. Persisted to NVS, triggers reconnect. Write-only on purpose — the password is never readable over BLE. |

A generic BLE scanner can't eyeball the dB number from the `record` characteristic (it's protobuf bytes, not plaintext); the consumer app is the real client.

The `wifi_creds` write happens in plaintext over BLE. We rely on physical proximity for security (~10 m BLE range, brief provisioning window). For stronger protection, switch the flag to `BLE_GATT_CHR_F_WRITE_ENC` and enable bonding — see the comment block in `ble_publisher.c`.

### Advertising and notify cadence

When no client is connected the device advertises every 5 s. When a client connects, advertising stops (undirected mode) and the device notifies `record` once per second.

BLE itself, with MTU 256 and a 30 ms connection interval, could comfortably do 20–30 notifies/sec — but the DSP only emits one record per second. Faster live updates would require a sub-second LAeq path in `audio_dsp.c`, not BLE tuning.

### What's not on BLE

No separate characteristics for battery voltage, status flags, pending-upload count, uptime, or calibration offset. Extend `noise.proto` if those need to be exposed — then both BLE and MQTT carry them automatically.

---

## Output channels

| Channel | What | Where |
|---|---|---|
| MQTT | Per-second protobuf records | `mqtt://broker.emqx.io:1883`, topic `noise/{device_id}/record` |
| BLE | Same protobuf, single `record` GATT characteristic, 1 Hz notify | NimBLE peripheral, custom 128-bit UUID |
| LittleFS | Batched log files (300 records / 5 min each), uploaded via HTTPS POST | `/littlefs/logs/{timestamp}_{client_id}.log` |

Records are only published / written when **both** the `CALIBRATED` and `TIME_SET` event bits are set. Until then, audio_dsp logs `NOT LOGGING — calibrated=… time_set=…` once per second so the developer sees what's gating output.

---

## Calibration

The DSP outputs raw FFT magnitudes in arbitrary units. A calibration offset (`int32_t`, persisted in NVS under `noise_cal/offset`) maps that to dB SPL. The offset is `(reference_dB − measured_dB) × 100`.

### Setting calibration

Calibration is done over BLE via the GATT characteristic at UUID `…0a02`
(int32-LE, signed hundredths of a dB). The `crew/lautstaerke` page on the
companion website exposes "Kalibrierung…" in the connected-device menu,
which reads the current offset, prompts for a new value, and writes it
back. The firmware persists the offset to NVS and sets the `CALIBRATED`
event bit.

### Calibration procedure

1. Play a **1 kHz sine** through a speaker. A-weighting at 1 kHz is 0 dB so dB(A) = dB(SPL).
2. Co-locate the device mic and an SPL meter (phone app like Decibel X is fine, ±2 dB).
3. Note reference dB(A) on the phone, e.g. 75.0.
4. Read the LAeq from the device's MQTT log line ("published seq=N LAeq=X.X dB(A)") — that's the decoded byte (0.5 dB resolution).
5. Compute `reference − device_LAeq` and enter that dB delta in the website's "Kalibrierung…" prompt.
6. Verify: phone and device should now match within ~1 dB at 1 kHz.

For frequency response verification, switch from sine to **pink noise** — it averages cleanly across all bands and matches what real music spectrally looks like.

---

## Build & flash

Standard ESP-IDF v5.4+ flow. The shell tooling requires `click<8.2` pinned (see `~/.claude/projects/.../memory/build_env.md`).

```bash
source ~/.espressif/v5.4.4/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodem* flash monitor
```

---

## Memory tuning

The board is an ESP32-S3-N16R2: 320 KB internal SRAM **plus 2 MB QSPI PSRAM**. We run audio DSP + WiFi + BLE + MQTT-TLS + HTTPS upload + LittleFS concurrently. PSRAM removes the old internal-heap crunch: the pre-PSRAM firmware had to disable BLE (`DEV_NO_BLE`) because NimBLE + the BT controller + WiFi/mbedtls + a second concurrent TLS session (backlog log upload) couldn't all fit in internal DRAM. With PSRAM that constraint is gone.

**What moves to PSRAM:**

- **WiFi / LWIP buffers** — `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` spills these out of internal DRAM automatically, which is what frees the headroom for BLE + dual-TLS.
- **30-min Leq ring buffers** (`laeq_ring` / `lceq_ring`, 14.4 KB combined) — tagged `EXT_RAM_BSS_ATTR`. They're written and read once per second, so PSRAM's higher access latency is irrelevant.

**What stays in internal DRAM** (touched on every FFT, ~23×/sec, or trivially small — PSRAM latency would hurt or gain nothing):

| Buffer | Where | Note |
|---|---|---|
| FFT twiddle table `fft_table` (16 KB) | static BSS, `audio_dsp.c` | 16-byte aligned; internal BSS array (was heap-allocated in the now-removed `audio_dsp_preinit()`) |
| FFT work buffer `fft_work` (32 KB), Hann window | static BSS, `audio_dsp.c` | Hot path |
| A/C per-bin weight tables (8 KB each) | static BSS, `audio_dsp.c` | Read every FFT |
| LogMessage staging | static BSS, `record_writer.c` | ~60 B since the proto was flattened — no reason to move |
| Task stacks, BLE controller, I²S DMA descriptors | internal | Latency- / DMA-sensitive |

**Coexistence settings that matter for WiFi + BLE + two TLS sessions.** Getting BLE back on alongside MQTT-TLS *and* a concurrent HTTPS log upload took more than flipping `CONFIG_SPIRAM=y`, because several things can't or don't move to PSRAM by default:

- **WiFi *static* RX buffers stay in internal DMA RAM** even with PSRAM. At 24 buffers (~38 KB) they starved the BLE controller's HCI init (`hci inits failed`). Kept at the IDF default of 10; the *dynamic* RX/TX pools stay large and live in PSRAM instead (they carry the HTTPS upload throughput).
- **mbedtls uses `CONFIG_MBEDTLS_DEFAULT_MEM_ALLOC`** (not the pre-PSRAM `INTERNAL`). With `SPIRAM_MALLOC_ALWAYSINTERNAL=16384`, TLS buffers ≥16 KB land in PSRAM — which is what lets the second (upload) TLS handshake's ~17 KB buffer allocate while MQTT's session is live. `CONFIG_MBEDTLS_DYNAMIC_BUFFER=y` still sizes/frees them on demand.
- **Software SHA/AES** (`CONFIG_MBEDTLS_HARDWARE_AES/SHA=n`, matching the already-software `MPI`). The HW crypto DMA path can't read PSRAM-resident TLS buffers directly and allocates an internal DMA bounce buffer per record; with two sessions those internal allocs fail. Software crypto works straight from PSRAM; CPU cost is negligible at our TLS volume.
- **`CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL=65536`** (default 32 KB) — reserves internal RAM for the allocations that *must* be internal (FreeRTOS objects, LWIP netconn semaphores, DMA). Without the extra headroom a backlog-drain burst briefly exhausted it (`thread_sem_init: out of memory`).

Also kept regardless of PSRAM: the I²S small-DMA-descriptor config.

See [PSRAM_MIGRATION.md](PSRAM_MIGRATION.md) for the full migration history and the one deferred item (radix-4 FFT).

---

## Power management

- **Dynamic frequency scaling enabled**: `esp_pm_configure()` in `app_main` sets max 160 MHz, min 40 MHz. CPU scales down to 40 MHz between FFT bursts and radio activity. This is the only PM mode that actually fires.
- **Light sleep disabled** (`light_sleep_enable = false`): with continuous I²S RX + a Core 1 DSP task at ~100 % utilization, the kernel never finds an "all cores idle" window. The I²S driver also holds an APB-freq PM lock as long as its RX channel is enabled. Enabling light sleep would just produce the `BLE_INIT: light sleep mode will not be ...` boot warning and never actually save anything.
- **WiFi modem sleep**: `WIFI_PS_MIN_MODEM` activated after STA gets IP. Saves significant idle WiFi current; MQTT publishes still work with a few ms latency on incoming packets. Fall back to `WIFI_PS_NONE` if the AP drops idle clients (some phone hotspots do).
- **BLE modem sleep**: with `BLE_GAP_ADV_ITVL_MS(1000)` the controller spends most of the time between advertising bursts with the radio off.

## Known gotchas

- **esp-dsp SIMD FFT bug:** always use `_ansi` variants, even though they're slightly slower. See "Why the ANSI FFT" above.
- **Calibration offset is hardware-specific.** Each device + mic combo needs its own offset. NVS stores it per-device.
- **LAFmax / LCFmax / LCpeak are pragmatic approximations.** Proper IEC 61672 Fast time-weighting wants an exponential smoother with τ=125 ms on time-domain weighted samples. We instead take the max single-FFT A/C-weighted energy across the second (~85 ms FFT window ≈ Fast response). LCpeak is the unweighted absolute sample peak (not C-weighted in the time domain). Adequate for festival trending; not Type-1 SLM lab-grade.

---

## Where to look in the code

| File | What it does |
|---|---|
| `main/main.c` | Task spawn + queue setup + LittleFS mount |
| `main/audio_dsp.c` | I²S read, FFT, per-bin A-weighting, band aggregation, record emission, shared `record_to_pb` / `record_encode_single` helpers |
| `main/calibration.c` | NVS persistence of cal offset, event bit management |
| `main/mqtt_publisher.c` | Protobuf encode + publish to MQTT broker |
| `main/ble_publisher.c` | NimBLE GATT server — record-notify, calibration read/write, WiFi credential write |
| `main/record_writer.c` | Buffer records to LittleFS in 300-record (5-min) log files |
| `main/log_uploader.c` | HTTPS upload of completed log files |
| `NOISE_MONITOR_SPEC.md` | Original product spec — what we're building and why |
