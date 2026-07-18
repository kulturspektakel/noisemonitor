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
>
> **Update (2026-07):** `NoiseRecording` was flattened (the nested `Record`
> submessage was removed; its fields live directly on `NoiseRecording`) and
> on-disk logs now store **one energy-aggregated record per minute** instead
> of up to 300 per-second records. `NoiseRecording_size` fell from 16816 to
> **73 bytes**, which freed **~15.6 KB of internal BSS** and made items **1**
> and **9** below doable *without* PSRAM — both are now done (see their tags).
> That freed BSS was spent on restoring the **30-min average**: `RING_30M` is
> back to 1800 (the no-PSRAM BLE-coexistence path was ruled out — uploads fail
> under BLE, see item 7 — so the headroom went here instead).

## 30-min average — RESTORED (2026-07)

`main/audio_dsp.c`:

```c
#define RING_30M 1800   // full 30-min ring (was 300 = 5-min under heap pressure)
```

The 30-min sliding window (`laeq_30m` / `lceq_30m`) is **re-enabled**: the
proto flatten freed ~15.6 KB BSS, so the full 1800-entry ring (14.4 KB) fits
again while BLE is off. `has_30m` goes true after 30 min of uptime. Previously
the ring was shrunk to 300 (5-min only) and `has_30m` stayed false, leaving the
fields unset (website rendered them as gaps). On the PSRAM board the ring moves
to PSRAM via `EXT_RAM_BSS_ATTR` regardless.

## Section 1 — Revert with PSRAM

### 1. `record_encode_single` stream-encoder — **Done (flattened proto, no PSRAM)**
- **Status:** Reverted. `record_encode_single` (`main/audio_dsp.c`) now stages a
  whole `NoiseRecording` on the stack and calls plain
  `pb_encode(&stream, NoiseRecording_fields, &rec)`. The manual field-by-field
  `pb_encode_tag`/`pb_encode_submessage`/`pb_encode_varint` chain (and the
  hard-coded field numbers) is gone; the 5m/30m stamping moved to the shared
  `record_apply_aggregates()` helper.
- **Why it was safe now (not PSRAM):** flattening removed the nested `Record`
  submessage and dropped the record array from `max_count:300` to a single
  record, so a stack-allocated `NoiseRecording` is only 73 bytes — no risk to
  the 4 KB publisher task stacks.

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

### 3. Radix-2 FFT instead of radix-4 — **Resolved differently (2026-07): SIMD radix-2**
- **What changed:** Rather than switching to radix-4, we enabled the hardware
  SIMD radix-2 kernel (`dsps_fft2r_fc32_aes3`) — see the "ANSI FFT" row in
  Section 2. SIMD radix-2 is faster than *ANSI* radix-4 and keeps the small 16 KB
  twiddle table (radix-4 needs 64 KB), so radix-4 was not worth doing.
- **Status:** No further action. Radix-4 remains a theoretical option but has no
  compelling case now that the SIMD radix-2 path works.

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

### 7. mbedtls SSL buffers + dynamic buffer — **Done early (dynamic buffer added 2026-07)**
- **Status:** `CONFIG_MBEDTLS_SSL_IN_CONTENT_LEN=16384` /
  `OUT_CONTENT_LEN=16384` (cert-chain-safe ceiling) **plus**
  `CONFIG_MBEDTLS_DYNAMIC_BUFFER=y` — the buffers are allocated on demand
  (~3 KB for our Let's Encrypt server) and freed when idle. This is the
  committed `sdkconfig.defaults` (note: a local `sdkconfig` had drifted to
  3072/2048 static — that drift is now removed; `sdkconfig` is gitignored so
  `sdkconfig.defaults` is the source of truth).
- **Why 16384 + dynamic instead of a static shrink:** keeps cert-chain safety
  (no `-0x7100`) while making the ceiling nearly free, so a second concurrent
  TLS session (HTTPS upload) has the best chance of fitting.
- **BLE coexistence test result (2026-07):** with BLE re-enabled, WiFi + BLE +
  MQTT-TLS + DSP + per-minute logging all run **stably** — but the HTTPS
  backlog upload's *second* concurrent TLS handshake still fails with
  **`-0x7F00` (SSL_ALLOC_FAILED)**, even with the dynamic buffer, because
  MQTT's live TLS + the BLE controller leave too little heap for it. With BLE
  off the same build uploads cleanly (HTTP 201) and `-0x7F00` never appears.
  **Conclusion: `DEV_NO_BLE` stays until PSRAM** — the live path coexists, but
  log uploads do not. Dynamic buffer is kept regardless (strict improvement).

### 8. LWIP TCP send/recv windows shrunk — **Done early**
- **Status:** Restored. `CONFIG_LWIP_TCP_SND_BUF_DEFAULT=5760`,
  `CONFIG_LWIP_TCP_WND_DEFAULT=5760` (IDF defaults).
- **Why it was safe to restore now:** Same as item 7. Post-PSRAM these
  will move to PSRAM via `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y`.
- **Benefit already realized:** ~2× HTTPS upload throughput today.

### 9. `LogMessage log_message_buf` as static BSS — **Resolved (flattened proto)**
- **Status:** No longer a memory concern. Flattening `NoiseRecording` and
  writing one aggregated record per file dropped `log_message_buf`
  (`main/record_writer.c`) from ~12.5 KB to ~60 bytes of BSS. It stays static
  BSS — fine at this size — so the `EXT_RAM_BSS_ATTR` / `calloc-in-PSRAM`
  reverts are moot. Nothing to do post-PSRAM.
- **Note:** `RECORDS_PER_FILE` no longer exists (one record per file), so the
  old "dynamic sizing" rationale is gone too.

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
| LittleFS partition layout + one-aggregated-record-per-file (per-minute) | Storage/durability, not memory. |
| ~~ANSI FFT variants instead of SIMD~~ **Superseded (2026-07): now on `dsps_fft2r_fc32_aes3` SIMD.** The `im=1.6` artifact was not an esp-dsp bug — it was an unaligned `fft_work` (aes3 needs ≥8-byte-aligned data). Fixed with `__attribute__((aligned(16)))`. |
| `record_to_pb` / `record_apply_aggregates` helpers | Single source of truth for `record_t → NoiseRecording` and 5m/30m stamping. Code quality, not memory. |
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

3. **Commit 3 — structural reverts that need PSRAM:** item 2 only
   (drop the `audio_dsp_preinit()` hook + early FFT-table allocation).
   Items 1 and 9 are already done — the proto flattening (2026-07)
   made both doable without PSRAM.

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
