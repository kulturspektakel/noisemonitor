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

Records go out every second containing 31 band values + 5 weighted summary metrics. The 15-minute sliding LAeq/LCeq aggregates are computed from in-RAM ring buffers and attached to every `NoiseRecording` message.

---

## Hardware

| Component | Part / wiring |
|---|---|
| MCU | ESP32-S3-WROOM-1-N16 (16 MB flash, no PSRAM) |
| Microphone | **InvenSense INMP441** (I²S, 3.3V-tolerant) |
| Pins | BCLK = GPIO 11, WS = GPIO 9, SD = GPIO 12; L/R tied to GND |

---

## DSP design

### Why FFT-based (not IIR filters)

A textbook Type-1 SLM uses cascaded biquad filters for A-weighting + 1/3-octave bandpass per band — accurate, but ~62 IIR filters running per sample at 48 kHz is heavy. We use a single 4096-pt FFT every 42 ms (50% overlap → ~23 FFTs/sec) and derive both A-weighting and bands from that. Far less CPU, accurate enough for festival monitoring.

### Why 4096-pt radix-2 (not radix-4)

The spec preferred radix-4 (faster). But on this no-PSRAM ESP32-S3, the radix-4 twiddle table (~64 KB) wouldn't allocate contiguously after WiFi + BLE fragmented the heap. Radix-2 needs only 16 KB and fits anywhere. CPU is not the constraint — we have plenty of headroom at 160 MHz.

### Why the ANSI FFT, not SIMD

`esp-dsp`'s SIMD-accelerated `dsps_fft2r_fc32` on ESP32-S3 has a bug: with all-zero input, it outputs a constant `1.6` in every imaginary bin. This leaks ~45 dB of fictional broadband noise floor into the spectrum, pinning every band at a constant value regardless of actual signal. We explicitly call the `_ansi` variants:

```c
dsps_fft2r_fc32_ansi(fft_work, FFT_SIZE);
dsps_bit_rev_fc32_ansi(fft_work, FFT_SIZE);
```

The ANSI version is correct and the perf difference at 23 FFTs/s is negligible.

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

`NoiseRecording.laeq_5m / lceq_5m / laeq_30m / lceq_30m` are sliding Leqs over the last 5 and 30 minutes. They share a single 30-min ring buffer per channel (`laeq_ring[1800]` and `lceq_ring[1800]`, ~7.2 KB each); the 5-min window is computed by averaging the 300 most-recent entries of the same ring via `compute_leq_recent`.

All four fields are `optional` and are **only set when the ring holds the full window of data** — 5-min fields appear after the device has been running 5+ minutes, 30-min fields after 30+ minutes. Below those thresholds the average would be over fewer samples than the field name implies, so the device omits them entirely (consumers see "missing" rather than misleading).

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

**Preferred (when USB console input works):** from a serial monitor connected at 115200 baud,

```
CAL_SET 5150     # sets offset to +51.50 dB
CAL_GET          # reads back current offset
CAL_CLEAR        # erases NVS, clears CALIBRATED bit
```

These are handled by `cal_console.c` reading USB-Serial-JTAG stdin.

**Fallback (when console input doesn't reach the device):** there is a `calibration_set(5150)` call in `main.c` right after `calibration_init()` that forces the offset on every boot. Edit the constant, rebuild, flash. Remove the call once `CAL_SET` over USB works for you. (We hit a macOS-specific issue where short-lived writes to `/dev/cu.usbmodem*` didn't reach the firmware's `fgets()` on stdin.)

### Calibration procedure

1. Play a **1 kHz sine** through a speaker. A-weighting at 1 kHz is 0 dB so dB(A) = dB(SPL).
2. Co-locate the device mic and an SPL meter (phone app like Decibel X is fine, ±2 dB).
3. Note reference dB(A) on the phone, e.g. 75.0.
4. Read the LAeq from the device's MQTT log line ("published seq=N LAeq=X.X dB(A)") — that's the decoded byte (0.5 dB resolution).
5. Compute `(reference − device_LAeq) × 100` → send `CAL_SET <result>`.
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

The ESP32-S3 has 320 KB SRAM, no PSRAM, and we run audio DSP + WiFi + BLE + MQTT + LittleFS concurrently. The heap gets aggressively fragmented during driver init — by the time tasks are running, the **largest contiguous free chunk drops to ~4 KB** even when 7+ KB total is free. So we don't fight fragmentation directly; we keep large buffers either in BSS (static, link-time reserved, never on heap) or pre-allocated very early.

| Large buffer | Strategy | Reason |
|---|---|---|
| FFT twiddle table (16 KB) | Heap-allocated in `audio_dsp_preinit()` from `app_main` | Needs to be in DRAM accessible by esp-dsp SIMD/ANSI code; aligned heap alloc is required |
| LogMessage staging (12.5 KB at 300 records) | **Static (BSS)** in `record_writer.c` | Avoids heap pool entirely; no risk of failed lazy alloc later. Initialized once at boot; fields overwritten per batch |
| A/C per-bin weight tables (8 KB each) | Static (BSS) in `audio_dsp.c` | One-time compute at init, never reallocated |
| FFT work buffer (32 KB), Hann | Static (BSS) in `audio_dsp.c` | Hot-path buffers, no allocation churn |

To make the MQTT client fit in the post-init heap, three of its config knobs are tightened from defaults:
- `cfg.buffer.size = 256` (default 1024) — our messages are 50-100 bytes encoded
- `cfg.buffer.out_size = 256` (default 1024)
- `cfg.task.stack_size = 3584` (default 6144) — the MQTT task loop is tight at our message sizes

The I²S driver is also tuned for smaller DMA descriptors (`dma_desc_num=4, dma_frame_num=120`) — ~2.5 ms DMA window, well under our 21 ms read cadence, saves a few KB.

**If you add anything that takes a contiguous heap chunk > ~4 KB at runtime, it will fail.** Either preinit-allocate it (like the FFT table), put it in BSS (like LogMessage), or rework it to stream.

---

## Power management

- **Dynamic frequency scaling enabled**: `esp_pm_configure()` in `app_main` sets max 160 MHz, min 40 MHz, with automatic light-sleep. CPU scales down to 40 MHz between FFT bursts and radio activity.
- **WiFi modem sleep**: `WIFI_PS_MIN_MODEM` activated after STA gets IP. Saves significant idle WiFi current; MQTT publishes still work with a few ms latency on incoming packets. Fall back to `WIFI_PS_NONE` if the AP drops idle clients (some phone hotspots do).
- **Light sleep limitation**: BLE controller blocks light sleep while active (you'll see `BLE_INIT: light sleep mode will not be ...` at boot). DFS still saves power; light sleep activates when no BLE peer is connected.
- **Tickless idle enabled**: FreeRTOS skips ticks during idle/sleep.

## Known gotchas

- **USB-Serial-JTAG input:** writes from `printf > /dev/cu.usbmodem*` on macOS don't reliably reach the firmware's `fgets()` on stdin. Use an interactive terminal session (`idf.py monitor` or `screen`) for `CAL_SET`, or the boot-time `calibration_set()` fallback in `main.c`.
- **esp-dsp SIMD FFT bug:** always use `_ansi` variants, even though they're slightly slower. See "Why the ANSI FFT" above.
- **Calibration offset is hardware-specific.** Each device + mic combo needs its own offset. NVS stores it per-device.
- **LAFmax / LCFmax / LCpeak are pragmatic approximations.** Proper IEC 61672 Fast time-weighting wants an exponential smoother with τ=125 ms on time-domain weighted samples. We instead take the max single-FFT A/C-weighted energy across the second (~85 ms FFT window ≈ Fast response). LCpeak is the unweighted absolute sample peak (not C-weighted in the time domain). Adequate for festival trending; not Type-1 SLM lab-grade.

---

## Console commands

Over USB-Serial-JTAG, 115200 baud:

| Command | Effect |
|---|---|
| `CAL_SET <offset_x100>` | Set calibration offset (dB × 100). Persists to NVS. Sets CALIBRATED event bit. |
| `CAL_GET` | Print current offset, or `uncalibrated` if not set. |
| `CAL_CLEAR` | Erase NVS, zero in-RAM offset, clear CALIBRATED bit. |

---

## Where to look in the code

| File | What it does |
|---|---|
| `main/main.c` | Task spawn + queue setup + LittleFS mount |
| `main/audio_dsp.c` | I²S read, FFT, per-bin A-weighting, band aggregation, record emission, shared `record_to_pb` / `record_encode_single` helpers |
| `main/calibration.c` | NVS persistence of cal offset, event bit management |
| `main/cal_console.c` | USB-Serial-JTAG line parser for CAL_* commands |
| `main/mqtt_publisher.c` | Protobuf encode + publish to MQTT broker |
| `main/ble_publisher.c` | NimBLE GATT server — single `record` notify characteristic, same protobuf as MQTT |
| `main/record_writer.c` | Buffer records to LittleFS in 300-record (5-min) log files |
| `main/log_uploader.c` | HTTPS upload of completed log files |
| `NOISE_MONITOR_SPEC.md` | Original product spec — what we're building and why |
