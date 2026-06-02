# PSRAM migration plan

Target board: ESP32-S3-N16R2 (16 MB flash + 2 MB QSPI PSRAM).
Current board: ESP32-S3 with no PSRAM, ~150 KB internal heap after boot.

The current firmware contains a number of memory-pressure workarounds that
exist solely because we have to fit WiFi + BLE + audio DSP + MQTT/TLS into
internal SRAM. Once the PSRAM board lands, walk this list top-to-bottom and
unwind the items in **Section 1**. The items in **Section 2** are not
memory-driven and should stay.

> **Status (2026-06):** BLE is currently disabled via `DEV_NO_BLE` in
> `main/main.c` to free enough internal heap for HTTPS uploads to work
> while we wait for the PSRAM board. While BLE is off, all of the
> workarounds whose only reason was "BLE eats too much heap" have been
> pre-reverted — see the **Done early** tags below. When the PSRAM
> board lands the remaining work is just: enable SPIRAM, tag the big
> BSS arrays `EXT_RAM_BSS_ATTR`, and remove the `DEV_NO_BLE` define.

## TL;DR — the one-line revert to get the 30-min average back

`main/audio_dsp.c`:

```c
#define RING_30M 300    // <-- bump to 1800 when PSRAM is available
```

The 30-min window is currently shrunk to a 5-min ring (300 entries) to fit
the no-PSRAM heap budget. `WINDOW_30M_SEC` is intentionally decoupled from
`RING_30M` and pinned at 1800, so `has_30m` stays false and the
`laeq_30m` / `lceq_30m` proto fields stay unset (the website renders them
as gaps — graceful degrade). Restoring the 1800-entry ring re-enables the
30-min average with no other code change.

## Section 1 — Revert with PSRAM

### 1. `record_encode_single` stream-encoder
- **Where:** `main/audio_dsp.c:390`
- **What:** Manual `pb_encode_tag` + `pb_encode_submessage` + `pb_encode_varint`
  calls field-by-field, with hard-coded field numbers (2, 5, 6, 7, 8, 9).
- **Why:** A stack-allocated `NoiseRecording` at `max_count:300` is ~12 KB
  and overflows the 4 KB BLE/MQTT publisher task stacks.
- **Revert:** Stage a `NoiseRecording` on heap (in PSRAM) and use plain
  `pb_encode(&stream, NoiseRecording_fields, &msg)`.
- **Benefit:** Drops ~40 lines of fragile encoding; field changes in
  `noise.proto` no longer require touching this function.

### 2. `audio_dsp_preinit()` hook + early FFT-table allocation
- **Where:** `main/main.c:41-47`, `main/audio_dsp.c:240`
- **What:** `app_main` calls `audio_dsp_preinit()` before any driver init,
  to grab a 16 KB contiguous block for the FFT twiddle table while heap is
  still fresh.
- **Why:** After WiFi + BLE fragment the heap, the largest contiguous chunk
  drops below 16 KB and the allocation fails.
- **Revert:** Tag `fft_table` with `EXT_RAM_BSS_ATTR` (lives in PSRAM
  directly), drop the preinit hook, drop the early-allocation invariant
  between `main.c` and `audio_dsp.c`.
- **Benefit:** Removes a fragile cross-file ordering requirement.

### 3. Radix-2 FFT instead of radix-4
- **Where:** `main/audio_dsp.c:36-43`
- **What:** Using `dsps_fft2r_fc32_ansi` (16 KB twiddle table) instead of
  `dsps_fft4r_fc32` (64 KB twiddle table).
- **Why:** 64 KB doesn't fit contiguously after WiFi+BLE init.
- **Revert:** Switch to radix-4; put the table in PSRAM via
  `EXT_RAM_BSS_ATTR`. Radix-4 is ~2× faster than radix-2.
- **Benefit:** More DSP idle time → more time in light sleep → battery win.
  Not a code-complexity revert, but a performance recovery.

### 4. BLE-first startup ordering (`BLE_HOST_READY` event bit) — **Done early**
- **Status:** Removed. `BLE_HOST_READY` no longer exists in
  `event_group.h`; `wifi_connect` no longer waits on it; `ble_publisher`
  no longer sets it. Startup tasks fire in any order.
- **Why it was safe to remove now:** BLE is off (`DEV_NO_BLE`), so the
  ordering it enforced has no current effect. Post-PSRAM the BLE
  controller has enough heap regardless of WiFi init order.

### 5. MQTT client config trim — **Done early**
- **Status:** Removed. `main/mqtt_publisher.c` no longer overrides
  `buffer.size`, `buffer.out_size`, or `task.stack_size` — the IDF
  defaults (1024/1024/6144) apply.
- **Why it was safe to remove now:** With BLE off there's heap to spare;
  with BLE back on PSRAM, the MQTT client buffers go to internal RAM
  but the budget supports defaults.

### 6. NimBLE host pool trims — **Done early**
- **Status:** Removed. The `CONFIG_BT_NIMBLE_MSYS_*`,
  `CONFIG_BT_NIMBLE_TRANSPORT_ACL_FROM_LL_COUNT`, and
  `CONFIG_BT_NIMBLE_TRANSPORT_EVT_*_COUNT` lines are out of
  `sdkconfig.defaults` — IDF defaults (12/24/24/30/8) apply.
- **Why it was safe to remove now:** BLE off → pools aren't allocated.
  When BLE comes back together with PSRAM the defaults fit.

### 7. mbedtls SSL buffers shrunk — **Done early**
- **Status:** Restored. `CONFIG_MBEDTLS_SSL_IN_CONTENT_LEN=16384`,
  `CONFIG_MBEDTLS_SSL_OUT_CONTENT_LEN=16384` (IDF defaults).
- **Why it was safe to restore now:** BLE off frees ~80 KB of internal
  heap; 32 KB of TLS buffers fits comfortably during a handshake. No
  more `-0x7100`/`-0x7F00` surprises when a server presents a fatter
  cert chain.

### 8. LWIP TCP send/recv windows shrunk — **Done early**
- **Status:** Restored. `CONFIG_LWIP_TCP_SND_BUF_DEFAULT=5760`,
  `CONFIG_LWIP_TCP_WND_DEFAULT=5760` (IDF defaults).
- **Why it was safe to restore now:** Same as item 7. Post-PSRAM these
  will move to PSRAM via `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y`.
- **Benefit already realized:** ~2× HTTPS upload throughput today.

### 9. `LogMessage log_message_buf` as static BSS
- **Where:** `main/record_writer.c:29`
- **What:** 12.5 KB struct, unconditionally reserved at link time.
- **Why:** Originally `calloc`'d at flush time; the 12.5 KB alloc failed
  after WiFi/BLE fragmented the heap.
- **Revert (option A):** Tag `EXT_RAM_BSS_ATTR` to push it out of internal
  RAM but keep BSS allocation.
- **Revert (option B):** Go back to `calloc` (in PSRAM), free after flush.
- **Benefit:** Dynamic sizing if `RECORDS_PER_FILE` changes later.

### 10. WiFi RX/TX buffer counts — **Done early**
- **Status:** Bumped. `CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM=24`,
  `CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM=32`,
  `CONFIG_ESP_WIFI_DYNAMIC_TX_BUFFER_NUM=32`.
- **Why it was safe to bump now:** BLE off frees the heap that previously
  blocked the larger pool. Post-PSRAM with
  `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` these move to PSRAM.
- **Benefit already realized:** No more `wifi:m f null` starvation
  warnings during HTTPS uploads.

## Section 2 — Keep (not memory-driven, or zero cost)

These are correct/legitimate decisions independent of heap pressure. Do
**not** revert them with PSRAM.

| Item | Reason to keep |
|------|----------------|
| `CONFIG_ESP_WIFI_NVS_ENABLED=n` | Workaround for cache-disabled-window crash during NVS writes (mbedtls reading flash-mapped .rodata while cache is briefly off). Correctness, not memory. |
| `CONFIG_ESP_PHY_CALIBRATION_AND_DATA_STORAGE=n` | Same cache-disable-window class of issue. |
| `CONFIG_SPI_FLASH_AUTO_SUSPEND=y` | Same — keeps cache live during flash writes. |
| `CONFIG_ESP_WIFI_EXTRA_IRAM_OPT=y` | Places `ppProcTxDone` in IRAM so it keeps running during cache-disable windows. |
| `CONFIG_BT_NIMBLE_ROLE_CENTRAL/OBSERVER/BROADCASTER=n` | We're a peripheral. Correct, not optimization. |
| `CONFIG_BT_NIMBLE_MAX_BONDS=0` | We don't pair. Correct. |
| `CONFIG_BT_NIMBLE_MAX_CONNECTIONS=1` | We accept one client. Correct. |
| `CONFIG_BT_CTRL_BLE_MAX_ACT=2` | We tried 1 and the controller returned `BLE_ERR_MEM_CAPACITY` on `ble_gap_adv_start`. 2 is the correct minimum for advertising + the eventual incoming connection slot. |
| Per-bin A/C weighting tables (16 KB BSS) | Correctness — band-center weighting drifts up to 0.5 dB on broadband signal. Tag `EXT_RAM_BSS_ATTR` if you want them in PSRAM. |
| `cached_buf[128]` in `ble_publisher.c` | One small static buffer for on-demand BLE reads. Trivial. |
| Power management (`CONFIG_PM_ENABLE`, tickless idle, 160 MHz default) | Battery life, not memory. |
| LittleFS partition layout + 300-record file batches | Storage, not memory. |
| ANSI FFT variants instead of SIMD (`dsps_fft2r_fc32_ansi` not `_aes3`) | Independent bug in esp-dsp SIMD path (`im=1.6` artifact). Stay on ANSI until the upstream bug is fixed. |
| `record_to_pb` helper | Single source of truth for `record_t → NoiseRecording_Record`. Code quality, not memory. |
| `audio_dsp` reader + `fft_worker` task split (`audio_dsp.c`) | Decouples I²S read from FFT compute so the DMA pool can stay small and FFT slowdowns don't drop samples. Architectural; keep regardless of PSRAM. |
| Wall-clock-paced `WORK_EMIT` (`audio_dsp.c`, `next_emit_us`) | Pins record emission to 1 Hz wall clock instead of sample count. Correctness for server-side `measuredAt` reconstruction; keep regardless of PSRAM. |

## Workflow when N16R2 arrives

1. **Commit 1 — enable PSRAM:**
   ```
   CONFIG_SPIRAM=y
   CONFIG_SPIRAM_MODE_QUAD=y
   CONFIG_SPIRAM_TYPE_AUTO=y
   CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y
   CONFIG_SPIRAM_USE_MALLOC=y     # treat PSRAM as heap-able
   ```
   Tag the large BSS arrays with `EXT_RAM_BSS_ATTR`: 30-min ring buffers
   (`laeq_ring`, `lceq_ring` in audio_dsp.c), `LogMessage log_message_buf`
   (record_writer.c), `fft_work`, `a_weight_bin`, `c_weight_bin`
   (audio_dsp.c). Verify boot.

2. **Commit 2 — remove `DEV_NO_BLE`:** delete the `DEV_NO_BLE` block in
   `main/main.c` (`ble_publisher_drain` stand-in + `#ifdef`), restoring
   the real `ble_publisher` task. Items 4, 5, 6, 7, 8, 10 are already
   pre-reverted, so re-enabling BLE on the PSRAM heap should Just Work.

3. **Commit 3 — structural reverts that need PSRAM:** items 1, 2, 9
   (stream encoder → heap-staged `NoiseRecording`; drop the
   `audio_dsp_preinit()` hook; `LogMessage log_message_buf` →
   `EXT_RAM_BSS_ATTR`).

4. **Optional later — item 3:** radix-4 FFT. Measure DSP utilization
   before/after; only worth doing if you want the extra idle time.

5. **Update README:** the "Memory budget" section
   (`README.md` §"Memory tuning") describes the ~4 KB largest-contiguous
   ceiling. That goes away — rewrite to mention PSRAM, what stays internal
   (task stacks, BLE controller, DMA buffers, FFT hot path), and what
   moves to PSRAM.

## Things to test after each commit

- Device discoverable in Web Bluetooth (filter on noise service UUID)
- MQTT publishes for ≥ 5 minutes without disconnect or `m f null`
- HTTPS log upload of a backlog file succeeds
- BLE write of calibration value (read-back matches)
- BLE write of WiFi creds (device reboots and reconnects to new AP)
- DSP per-second cadence is steady (no missed records)
