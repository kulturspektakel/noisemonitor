# PSRAM migration plan

Target board: ESP32-S3-N16R2 (16 MB flash + 2 MB QSPI PSRAM).
Current board: ESP32-S3 with no PSRAM, ~150 KB internal heap after boot.

The current firmware contains a number of memory-pressure workarounds that
exist solely because we have to fit WiFi + BLE + audio DSP + MQTT/TLS into
internal SRAM. Once the PSRAM board lands, walk this list top-to-bottom and
unwind the items in **Section 1**. The items in **Section 2** are not
memory-driven and should stay.

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

### 4. BLE-first startup ordering (`BLE_HOST_READY` event bit)
- **Where:** `main/ble_publisher.c:355-359`, `main/wifi_connect.c:118-119`,
  `main/event_group.h:12`
- **What:** `wifi_connect` waits on `BLE_HOST_READY` before starting WiFi
  init. `ble_publisher` sets the bit after `nimble_port_freertos_init`.
- **Why:** The BLE controller's runtime allocations need ~30 KB contiguous
  `MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA`. If WiFi runs first and eats
  ~110 KB, the controller's first HCI `set_advertising_data` fails with
  `BLE_ERR_MEM_CAPACITY` and the device becomes silently undiscoverable.
- **Revert:** Drop the `BLE_HOST_READY` bit entirely, drop the
  `xEventGroupWaitBits` call in `wifi_connect`, drop the
  `xEventGroupSetBits` call in `ble_publisher`. Startup tasks fire in any
  order.
- **Benefit:** Two fewer cross-file coupling points.

### 5. MQTT client config trim
- **Where:** `main/mqtt_publisher.c:55-65`
- **What:** `buffer.size = 256`, `buffer.out_size = 256`,
  `task.stack_size = 3072`.
- **Why:** Defaults (1024/1024 buffers, 6144 stack) don't fit in the
  fragmented post-init heap.
- **Revert:** Restore defaults. The buffer cuts currently cap publish size
  to 256 B (we publish ~100 B today, so it's fine, but it's a ceiling worth
  removing).

### 6. NimBLE host pool trims
- **Where:** `sdkconfig.defaults:48-52`
- **What:**
  - `CONFIG_BT_NIMBLE_MSYS_1_BLOCK_COUNT=4` (default 12)
  - `CONFIG_BT_NIMBLE_MSYS_2_BLOCK_COUNT=4` (default 24)
  - `CONFIG_BT_NIMBLE_TRANSPORT_ACL_FROM_LL_COUNT=2` (default 24)
  - `CONFIG_BT_NIMBLE_TRANSPORT_EVT_COUNT=10` (default 30)
  - `CONFIG_BT_NIMBLE_TRANSPORT_EVT_DISCARD_COUNT=4` (default 8)
- **Why:** Each pool is allocated from internal heap. The cuts above save
  ~5 KB which we need for the host task to spawn at all.
- **Revert:** Restore defaults. NimBLE host buffers stay in internal RAM
  (they're touched on hot paths and need fast access) but with PSRAM more
  internal heap is available.
- **Benefit:** Headroom for bursty BLE traffic (multi-byte writes,
  back-to-back notifications).

### 7. mbedtls SSL buffers shrunk
- **Where:** `sdkconfig.defaults:92-93`
- **What:** `MBEDTLS_SSL_IN_CONTENT_LEN=4096`, `MBEDTLS_SSL_OUT_CONTENT_LEN=4096`.
- **Why:** Defaults (16384 each, 32 KB per connection) can't be allocated
  contiguously after BLE.
- **Revert:** Restore defaults.
- **Benefit:** More compatibility with servers that send larger TLS
  handshake records or fragmented certs. No more
  `MBEDTLS_ERR_SSL_ALLOC_FAILED` surprises.

### 8. LWIP TCP send/recv windows shrunk
- **Where:** `sdkconfig.defaults:86-87`
- **What:** `LWIP_TCP_SND_BUF_DEFAULT=2880`, `LWIP_TCP_WND_DEFAULT=2880`
  (default 5760).
- **Why:** TCP buffers eat heap.
- **Revert:** Restore defaults. Together with
  `SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y`, these buffers go to PSRAM.
- **Benefit:** ~2× throughput on HTTPS log uploads.

### 9. `LogMessage log_message_buf` as static BSS
- **Where:** `main/record_writer.c:29`
- **What:** 12.5 KB struct, unconditionally reserved at link time.
- **Why:** Originally `calloc`'d at flush time; the 12.5 KB alloc failed
  after WiFi/BLE fragmented the heap.
- **Revert (option A):** Tag `EXT_RAM_BSS_ATTR` to push it out of internal
  RAM but keep BSS allocation.
- **Revert (option B):** Go back to `calloc` (in PSRAM), free after flush.
- **Benefit:** Dynamic sizing if `RECORDS_PER_FILE` changes later.

### 10. WiFi RX/TX buffer counts
- **Where:** `sdkconfig.defaults:83-85`
- **What:**
  - `CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM=6`
  - `CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM=16`
  - `CONFIG_ESP_WIFI_DYNAMIC_TX_BUFFER_NUM=16`
- **Why:** Each static RX buffer is 1600 B; defaults can't all be allocated
  in the post-BLE heap. We've seen runtime `wifi:m f null` starvation
  warnings when these get too tight.
- **Revert:** With `SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` the WiFi buffer pool
  moves to PSRAM. Then increase the counts (24/32/32 is reasonable).
- **Benefit:** No more packet drops or `m f null` under sustained load,
  better TLS resilience.

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

2. **Commit 2 — structural reverts:** items 1, 2, 4 above (stream encoder,
   preinit hook, BLE_HOST_READY ordering bit). Highest code-complexity
   wins.

3. **Commit 3 — config restorations:** items 5, 6, 7, 8, 10. All
   sdkconfig knobs back near defaults.

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
