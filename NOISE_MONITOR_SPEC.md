# Festival Noise Monitor — Implementation Spec

**Reference codebase:** `kulturspektakel/contactless` (v4.0 tag) — this spec calls out specific files to reuse, adapt, or skip.

---

## 1. Goal

Build a battery- or USB-powered ESP32-S3 device that continuously measures ambient sound, computes 1/3 octave band Leq plus broadband peak metrics every second, stores them locally, and publishes/uploads to a backend.

---

## 2. Hardware

| Component | Part / configuration |
|---|---|
| Microcontroller | ESP32-S3-WROOM-1-N16 (16 MB flash, no PSRAM) |
| Microphone | TDK InvenSense ICS-43434 (Adafruit breakout #4346 for dev), I²S |
| RTC | DS3231 module over I²C |
| Status indicator | RGB LED via LEDC PWM (3 GPIO pins) |
| Connectivity | WiFi STA only — no cellular, no Bluetooth |
| Display | none |

**I²S cable length constraint:** ICS-43434 must be wired with a short pigtail (≤30 cm, ideally 10–15 cm) from the ESP32. Longer runs require buffer ICs and are out of scope for this spec.

### Pin assignments

Pins for RGB LED, DS3231 RTC, USB voltage, and battery voltage match the contactless project (RevE) so the same code reuses them without modification. Microphone pins are new.

```
Function                     GPIO    Source
─────────────────────────────────────────────────────────────────────
LED red                      42      contactless power_management.c (LED_RED_PIN)
LED green                    41      contactless power_management.c (LED_GREEN_PIN)
LED blue                     40      contactless power_management.c (LED_BLUE_PIN)
I²C SDA   (DS3231)           39      contactless rfid.c / time_sync.c
I²C SCL   (DS3231)           38      contactless rfid.c / time_sync.c
I²C port                     I2C_NUM_0
USB voltage sense            1       contactless power_management.c (USB_PIN, ADC1_CH0)
Battery voltage sense        2       contactless power_management.c (BATTERY_CHANNEL, ADC1_CH1)

I²S BCLK / SCK   (mic)       9       NEW
I²S WS   / LRCK  (mic)       10      NEW
I²S DATA / DOUT  (mic)       11      NEW
ICS-43434 SEL (L/R)          tied to GND for left channel
ICS-43434 VDD                3V3
ICS-43434 GND                GND
```

The I²S pins (9, 10, 11) are clean GPIOs on ESP32-S3 with no special-function constraints and no conflict with any pin used by the contactless project. ESP32-S3's I²S peripheral has a flexible pin matrix, so these can be remapped if hardware constraints require.

Strapping/reserved pins to avoid: 0 (BOOT), 19/20 (USB-CDC for `cal_console`), 26–32 (internal flash on WROOM-1-N16), 43/44 (UART0 console), 45/46 (strapping). Pin 3 (JTAG MTDO) is free if JTAG not in use.

---

## 3. Software Stack

- ESP-IDF v5.4.1 (matches contactless project)
- LittleFS (`joltwallet/littlefs`) for on-flash storage
- nanopb for Protocol Buffers (component already in contactless repo at `components/nanopb/`)
- `esp-mqtt` (built into ESP-IDF) for MQTT publishing
- `esp_http_client` for HTTPS file uploads

---

## 4. Reuse from contactless

### Direct reuse (copy, rename if needed, no architectural changes)

| File | Use as-is for |
|---|---|
| `main/time_sync.c` + `.h` | DS3231 + NTP sync; sets system time from RTC at boot, then re-syncs from NTP after WiFi connects. Time is never set from any HTTP response. |
| `main/log_uploader.c` + `.h` | HTTP POST batch file uploader with retry timer and network-request semaphore |
| `main/http_auth_headers.c` + `.h` | HTTP authentication header construction. **Update:** change the `User-Agent` string from `"Contactless/<version>"` to `"NoiseMonitor/<version>"` (or leave as-is if backend doesn't distinguish). The salt+device_id SHA1 auth scheme is reused unchanged. |
| `main/network_request.h` (semaphore pattern) | Serializes concurrent network access |
| `main/constants.c` + `.h` | Task names, priorities, eFuse-backed `device_id` and `salt` loading. **Drop unused NVS keys:** `NVS_PRODUCT_LIST`, `NVS_SOUND_MODE`. **Review:** `TZ` macro — keep contactless's CET value or change to `"UTC"` depending on intent. |
| `scripts/proto.sh` | nanopb code generation pipeline |
| `partitions.csv` | Custom partition table (16 MB flash, ~13 MB LittleFS); reuse layout, expand `factory` if DSP code requires |
| `sdkconfig.defaults` | ESP32-S3 + 16 MB flash + custom partition base config |
| `components/nanopb/` | nanopb library component |
| `main/idf_component.yml` | Dependency manifest; add new deps as needed |
| `.clang-format`, `.devcontainer/`, `.gitignore` | Dev environment |
| `main/power_management.c` | Keep: ADC initialization + voltage measurement (USB on GPIO 1 / ADC1_CH0, battery on ADC1_CH1), the LEDC PWM init for the RGB LED (`ledc_init()`, `set_rgb_color()`), the periodic voltage update timer, the USB-plug interrupt + `USB_CONNECTED` event bit. Drop: the 15-minute power-off timer and `esp_deep_sleep_start()` logic — the noise monitor stays awake while powered. |

### Adapt

| File | Change |
|---|---|
| `main/wifi_connect.c` + `.h` | Reuse the WiFi STA management with auto-reconnect, NVS-backed credentials, signal-strength polling. Add power-management changes per §12: call `esp_wifi_set_ps(WIFI_PS_MIN_MODEM)` after IP acquisition, call `esp_wifi_stop()` on disconnect, call `esp_wifi_start()` before each retry. **Note:** these power-management changes are general-purpose and should be upstreamed back into the contactless project's `wifi_connect.c` so both projects share the improvement. |
| `main/event_group.h` | Drop `LOCAL_CONFIG_LOADED` (no local config). Add `CALIBRATED = BIT8`. Update `STARTUP_BITS` to `TIME_SET | DEVICE_ID_LOADED | SALT_LOADED`. |
| `main/log_writer.c` | Pattern stays the same (build a `LogMessage`, encode it, write to file, notify uploader). Buffer 300 `Record` entries in RAM; on the 300th record, populate `LogMessage.noise_recording.records` and write the file. See §7 for details. |
| `components/nanopb/logmessage.proto` and `.options` | Add a new optional `NoiseRecording` payload variant at field 11. See §5. |
| `main/main.c` | Replace task list — see §6. Remove tasks for components we don't have. |

### Skip (not applicable)

`display.c`, `u8g2_esp32_hal.c`, `keypad.c`, `buzzer.c`, `rfid.c`, `pn532.c`, `state_machine.c`, `battery_test.c`, `fetch_config.c`, `local_config.c`. Domain-specific to the contactless terminal — no analog on the noise monitor (no product lists, no privilege tokens, nothing to load from `/littlefs/config.cfg`). Likewise the contactless `configs.proto` and `config.proto` are not used; only `logmessage.proto` (modified) and `product.proto` (referenced by logmessage.proto unchanged).

---

## 5. Protobuf Schema

The noise monitor reuses the contactless `LogMessage` envelope and adds a new payload variant. Two files change/are added:

### New: `components/nanopb/noise.proto`

```proto
syntax = "proto3";

message Record {
  uint32 seq_no    = 1;   // monotonic; gap detection
  bytes  bands     = 2;   // 31 bytes; (dB - 20) * 2 per band
  uint32 laeq_1s   = 3;   // same encoding as bands
  uint32 lceq_1s   = 4;
  uint32 lafmax_1s = 5;
  uint32 lcfmax_1s = 6;
  uint32 lcpeak_1s = 7;
}

message NoiseRecording {
  sint32 calibration_offset_db_x100 = 1;   // ZigZag-encoded for compact negative values
  repeated Record records           = 2;   // 1 (MQTT live) or up to 300 (file)
}
```

### New: `components/nanopb/noise.options`

```
Record.bands                          max_size:31, fixed_length:true
NoiseRecording.records                max_count:300
```

### Modify: `components/nanopb/logmessage.proto`

Add an `import` and one new optional field. Existing fields and field numbers must not change:

```proto
syntax = "proto3";
import "product.proto";
import "noise.proto";   // NEW

message LogMessage {
  string device_id              = 1;
  string client_id              = 2;
  int32  device_time            = 3;
  bool   device_time_is_utc     = 4;
  optional Order order                              = 5;
  optional CardTransaction card_transaction         = 6;
  optional int32 battery_voltage                    = 7;
  optional int32 usb_voltage                        = 8;
  optional CrewCardEnrollment crew_card_enrollment  = 10;
  optional NoiseRecording noise_recording           = 11;   // NEW

  // existing nested messages unchanged
}
```

Field 11 is the next available slot after the existing payload variants (field 9 is reserved historically; do not reuse). All other fields and types in `LogMessage` and its nested types are untouched.

### Where each message is used

| Context | Message | Notes |
|---|---|---|
| File on flash | `LogMessage` with `noise_recording.records` populated to 300 entries | One `LogMessage` per file, same pattern as contactless |
| MQTT live publish | `NoiseRecording` with `records` populated to exactly 1 entry | Topic identifies the device, so no `LogMessage` envelope needed |

### Band value encoding

```
byte_value = round( (dB - 20.0) * 2.0 )
dB         = 20.0 + byte_value / 2.0
```

Range 20.0–147.5 dB, 0.5 dB resolution.

The `bands` field contains 31 bytes, one per IEC 61260 1/3 octave band, in this fixed order:

```
band[0..30] center frequencies (Hz):
16, 20, 25, 31.5, 40, 50, 63, 80, 100, 125,
160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250,
1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, 10000, 12500,
16000
```

Each band value is the unweighted Leq for that band's frequency range over the 1-second window. A and C weighted broadband levels (LAeq, LCeq) are derived server-side from the bands using IEC 61672 weighting tables. The `laeq_1s` and `lceq_1s` fields are cached on-device as a query convenience.

---

## 6. Task Architecture

ESP32-S3 dual-core layout:

**Core 1 (APP_CPU) — real-time audio DSP, isolated from networking**

```c
xTaskCreatePinnedToCore(&audio_dsp, "audio_dsp", 8192, NULL, TASK_PRIO_HIGH, NULL, 1);
```

**Core 0 (PRO_CPU) — networking, storage, housekeeping (default placement)**

```c
// Reused from contactless (unchanged or lightly renamed):
xTaskCreate(&wifi_connect,      WIFI_CONNECT_TASK,      4096, NULL, TASK_PRIO_NORMAL, NULL);
xTaskCreate(&time_sync,         "time_sync",            4096, NULL, TASK_PRIO_NORMAL, NULL);
xTaskCreate(&log_uploader,      LOG_UPLOADER_TASK,      4096, NULL, TASK_PRIO_NORMAL, NULL);
xTaskCreate(&power_management,  POWER_MANAGEMENT_TASK,  4096, NULL, TASK_PRIO_NORMAL, NULL);
xTaskCreate(&load_device_id,    "load_device_id",       3072, NULL, TASK_PRIO_NORMAL, NULL);
xTaskCreate(&load_salt,         "load_salt",            3072, NULL, TASK_PRIO_NORMAL, NULL);

// Adapted:
xTaskCreate(&record_writer,     "record_writer",        4096, NULL, TASK_PRIO_NORMAL, NULL);

// New:
xTaskCreate(&mqtt_publisher,    "mqtt_publisher",       4096, NULL, TASK_PRIO_NORMAL, NULL);
xTaskCreate(&cal_console,       "cal_console",          3072, NULL, TASK_PRIO_NORMAL, NULL);
xTaskCreate(&ble_publisher,     "ble_publisher",        4096, NULL, TASK_PRIO_NORMAL, NULL);
```

Task priorities follow contactless conventions: `TASK_PRIO_NORMAL = 5`, `TASK_PRIO_HIGH = 10`.

### Inter-task queues

Three FreeRTOS queues fan out from `audio_dsp` to its consumers. All are statically sized at boot:

```c
QueueHandle_t record_writer_queue  = xQueueCreate(16, sizeof(record_t));
QueueHandle_t mqtt_publisher_queue = xQueueCreate(16, sizeof(record_t));
QueueHandle_t ble_publisher_queue  = xQueueCreate(16, sizeof(record_t));
```

`record_t` is a small (~50 byte) struct holding the per-second measurement payload (sequence number, 31 band uint8s, four broadband uint8s). 16 slots = 16 seconds of buffering, generously sized against any reasonable consumer delay. Sends from DSP are non-blocking (`xQueueSend(..., 0)`); see §7 audio_dsp. On overflow of any queue, the record is dropped — the file path (`record_writer_queue`) is durability, MQTT and BLE are live-monitoring side-channels.

### Stack-size guidance

Initial stack sizes per task (above) are intentionally generous to avoid stack overflow during development. Once code stabilizes, monitor actual usage with `uxTaskGetStackHighWaterMark` and trim conservatively. **Don't preemptively shave stacks** — stack overflows are silent and corrupt; the savings (a few KB per task) aren't worth the risk during active development.

---

## 7. New Components

### Implementation conventions (apply to all components in this section)

These apply across `audio_dsp`, `record_writer`, `mqtt_publisher`, and `cal_console`:

- **Static allocation only.** No `pvPortMalloc` for anything that lives longer than a single function call. All buffers, accumulators, queues, and protobuf message structs are file-scope static. This eliminates heap fragmentation as a long-running concern and makes memory deterministic.
- **Encode protobuf directly to file via stream callback.** Use the contactless pattern from `log_writer.c::write_pb_to_file`. Never `pb_encode` to an intermediate byte buffer and then `fwrite` — peak RAM usage stays at the encoder's small internal state (~hundreds of bytes), not the full ~15 KB encoded message size.
- **Right-size stacks during development; trim only after measurement.** See §6.

### `audio_dsp` (Core 1)

Single-task DSP pipeline:

1. Configure I²S RX peripheral (mono, 24-bit, 48 kHz) for ICS-43434 with DMA double-buffering
2. Continuously consume audio buffers (no duty-cycling)
3. Per audio sample (48 kHz):
   - Apply A-weighting biquad → Fast (125 ms) exponential smoother → max-hold for `lafmax_1s`
   - Apply C-weighting biquad → Fast smoother → max-hold for `lcfmax_1s`
   - Apply C-weighting → absolute value → max-hold for `lcpeak_1s`
4. Per FFT window (4096 points, Hann, 50% overlap, ~10–20 Hz internal rate):
   - Compute magnitude squared
   - Group into 31 1/3 octave bands using IEC 61260 weights
   - Accumulate energy per band into per-second accumulators
5. Every 1 second:
   - Compute energy-mean per band, convert to dB, apply calibration offset
   - Compute cached `laeq_1s` and `lceq_1s` from bands (using A/C weighting tables)
   - Update the static ring buffers for aggregate Leq computation (per §9 BLE)
   - Encode all values to uint8 per the encoding formula
   - **If both `CALIBRATED` and `TIME_SET` event bits are set:** push `record_t` onto three queues — `record_writer_queue`, `mqtt_publisher_queue`, and `ble_publisher_queue` — using `xQueueSend(..., 0)` (non-blocking, zero timeout). DSP must never block on any send.
   - **Otherwise:** print broadband levels and current state to console (see §10), do not push to queues. Both calibration and a valid clock are required before any data is emitted, because file names depend on a valid timestamp and downstream consumers need calibrated values.
   - Reset accumulators and running maxes

#### Queue overflow behavior

Both queues are sized 16 (see §6). Overflow happens only if a downstream consumer is stalled longer than ~16 seconds — rare under normal flash/network behavior. On overflow:

- `xQueueSend` returns `errQUEUE_FULL`
- DSP logs a `WARN`-level message ("record_writer_queue full, dropping record N") and continues
- The dropped record is lost; DSP never blocks waiting for queue capacity

Acceptable tradeoff: blocking the DSP would cause I²S DMA buffer overruns and corrupt all future audio capture, which is far worse than losing one second of data.

### `record_writer` (Core 0)

Adapts contactless `log_writer.c`. Consumes records from `record_writer_queue`. Records are buffered in RAM as a `NoiseRecording` and flushed to flash inside a `LogMessage` envelope when 300 have accumulated.

#### Static allocation

All buffers are statically allocated at file scope. No `pvPortMalloc` calls in this component. This eliminates heap fragmentation as a long-running concern and makes memory usage deterministic.

```c
// File-scope statics in record_writer.c
static LogMessage     log_message_buf;        // contactless-generated nanopb struct
static int            records_count = 0;      // 0..300
```

The 300-record buffer lives inside `log_message_buf.noise_recording.records` (the nanopb-generated array of fixed size 300 from the `.options` `max_count:300` constraint). Total static footprint ~20 KB.

#### Algorithm

1. On startup: ensure `LOG_DIR` exists (reuse pattern from `log_uploader.c::maybe_create_log_dir`). Initialize `log_message_buf.noise_recording.calibration_offset_db_x100` from NVS. Set `records_count = 0`.
2. Per record received via `xQueueReceive(record_writer_queue, ..., portMAX_DELAY)`:
   - Copy into `log_message_buf.noise_recording.records[records_count]`
   - Increment `records_count`
3. When `records_count == 300`:
   - Populate the rest of the `LogMessage` envelope:
     - `device_id` from eFuse (existing contactless code)
     - `client_id` = 8-character random ID (existing contactless ALPHABET pattern)
     - `device_time` = current Unix time
     - `device_time_is_utc` = true
     - Optionally `battery_voltage` / `usb_voltage` if `power_management.c` is in use
     - `noise_recording.records_count = 300`
   - **Run capacity management** (see below) to ensure sufficient free space
   - Open a new file `<LOG_DIR>/<unix_ts>_<client_id>.log` where `<unix_ts>` is the current Unix time as a 10-digit zero-padded decimal
   - Encode the entire `LogMessage` directly to file via nanopb stream callback (matches contactless `log_writer.c` pattern exactly — no intermediate byte buffer)
   - Close the file, notify `log_uploader` via `xTaskNotify`
   - Reset `records_count = 0`; start accumulating the next file
4. Trade-off acknowledged: a power loss before reaching 300 records loses the in-progress buffer (up to 5 minutes). This is the chosen trade for simpler file format and reduced flash wear.

#### Capacity management

Before opening each new file, ensure the LittleFS partition has at least 200 KB headroom by deleting oldest files until enough space is available. This is the ring-buffer behavior that prevents the partition from filling up during sustained WiFi outages.

```c
const size_t HEADROOM = 200 * 1024;

while (true) {
    size_t total, used;
    esp_littlefs_info("littlefs", &total, &used);
    if (total - used >= HEADROOM) break;

    char* oldest = find_oldest_file(LOG_DIR);  // see below
    if (oldest == NULL) {
        ESP_LOGE(TAG, "out of space and no files to delete");
        break;
    }

    char path[256];
    snprintf(path, sizeof(path), "%s/%s", LOG_DIR, oldest);
    ESP_LOGW(TAG, "low storage, deleting oldest file: %s", oldest);
    remove(path);
    free(oldest);
}
```

`find_oldest_file` is a single-pass `readdir` scan tracking the lexicographically smallest filename — efficient because filenames begin with the timestamp:

```c
static char* find_oldest_file(const char* dir_path) {
    DIR* dir = opendir(dir_path);
    if (!dir) return NULL;

    char oldest[FILENAME_MAX] = {0};
    struct dirent* entry;
    while ((entry = readdir(dir))) {
        if (entry->d_type != DT_REG) continue;
        if (oldest[0] == '\0' || strcmp(entry->d_name, oldest) < 0) {
            strncpy(oldest, entry->d_name, sizeof(oldest) - 1);
        }
    }
    closedir(dir);

    return oldest[0] ? strdup(oldest) : NULL;
}
```

Properties:

- O(N) in number of files, single directory scan, no `stat()` calls (filenames carry the timestamp)
- Runs only when free space is below the headroom threshold — typical case is a no-op
- Deletes oldest first, preserving the most recent ~3 days of data under sustained outage
- Each deletion logs at `WARN` level so the operator can see this happened on inspection
- If `find_oldest_file` returns NULL (no files to delete) and we're still out of space, log error and let the subsequent `fopen` fail naturally — the writer's normal error path handles it.

#### Race-safety with the DSP task

DSP keeps producing records during the file write (~100–200 ms typically). Those records accumulate in `record_writer_queue` while `record_writer` is blocked in `pb_encode`. When the write completes, `record_writer` returns to `xQueueReceive` and drains the backlog in order. The 16-slot queue absorbs any reasonable write delay including occasional 1–2 second LittleFS garbage-collection pauses.

DSP never blocks on the queue (uses non-blocking send per §7). On the rare event of queue overflow, DSP drops the record and logs a warning — see "Queue overflow behavior" above.

### `mqtt_publisher` (Core 0)

New task. Consumes records from `mqtt_publisher_queue`. Strictly fire-and-forget — never blocks the audio pipeline, never retries, never queues for later delivery beyond the queue itself.

1. Per record received via `xQueueReceive(mqtt_publisher_queue, ...)`:
   - If the `WIFI_CONNECTED` event group bit is **not** set: drop the record silently, do nothing
   - If WiFi is connected but the MQTT client is not yet connected: drop the record silently
   - Otherwise: build a `NoiseRecording` containing `calibration_offset_db_x100` and exactly one `Record` in `records`. Serialize to protobuf and publish to topic `noise/{device_id}/record` with QoS 0.
2. Each MQTT message is fully self-describing (includes session metadata) so dashboard subscribers don't need to subscribe to a separate session topic.
3. MQTT client connection lifecycle:
   - Attempt connection only after `WIFI_CONNECTED` is set
   - Development broker: `broker.emqx.io:8884` over `wss://` (no auth)
   - On disconnect: do not buffer; subsequent records are dropped until reconnection
4. The DSP→MQTT queue is sized 16 (matches the writer queue). On overflow, DSP drops the record (see §7 audio_dsp queue overflow behavior). Acceptable here because MQTT is a live-monitoring channel; the file path provides durability.
5. Static allocation: the `NoiseRecording` build buffer is file-scope static. No heap allocation per publish.

### `ble_publisher` (Core 0)

BLE peripheral exposing live measurements over GATT. Consumes from `ble_publisher_queue`. Manages connection lifecycle, characteristic value updates, and subscription-aware notifications. See §9 BLE for full behavior, GATT service definitions, characteristic list, and notification rate strategy.

### `cal_console` (Core 0)

USB-CDC text-command interface for setting and inspecting calibration. Reads lines from stdin, parses three commands, writes to NVS. See §10 for full behavior.

---

## 8. Storage

### Partition table

Reuse contactless `partitions.csv` structure:

```csv
nvs,      data, nvs,           , 0x3000,
phy_init, data, phy,           , 0x1000,
factory,  app,  factory,       , 0x170000,
littlefs, data, spiffs,        , 0xDC70A0,
```

Adjust `factory` partition size if DSP code exceeds 1.5 MB; expand into LittleFS region.

### Log directory and rotation

- Log directory: `/littlefs/logs/`
- File rotation: triggered by record count, not wall-clock time. A new file is written when the in-RAM buffer reaches 300 records (≈ 5 minutes at 1 Hz)
- File contents: one `LogMessage` protobuf message with the `noise_recording` field populated; that `NoiseRecording` contains 300 `Record` entries
- Filename: `<unix_ts>_<client_id>.log`, where `<unix_ts>` is the file's creation time as a 10-digit zero-padded Unix timestamp and `<client_id>` is the 8-character random ID from the contactless ALPHABET pattern. Lexicographic sort = chronological sort.
- Approx file size: ~15 KB
- **Ring buffer behavior**: see §7 record_writer "Capacity management". Before opening each new file, the partition is checked for ≥200 KB free; if not, the oldest file (by filename, since timestamps prefix the filenames) is deleted. Repeated until sufficient space exists or no files remain. Uploaded files are deleted by `log_uploader` on success, so files-on-disk are by definition unuploaded; ring-buffer eviction therefore drops the oldest unuploaded data first.
- **Precondition for writing**: both `CALIBRATED` and `TIME_SET` event bits must be set. While either is unset, no files are created and DSP records are dropped after console-printing for calibration use (see §10).

---

## 9. Communication

### WiFi

Reuse `wifi_connect.c` directly. WiFi credentials live in NVS (`NVS_WIFI_SSID`, `NVS_WIFI_PASSWORD`).

### MQTT (live monitoring)

- Development broker: `broker.emqx.io:8884` over `wss://` (no auth)
- Topic: `noise/{device_id}/record`
- QoS 0 (fire-and-forget)
- **Strictly best-effort:** if WiFi is not connected, do not attempt to publish. If MQTT client is not connected, do not attempt to publish. Records that can't be published are dropped silently. No queueing, no retry, no buffering.
- Durability is provided exclusively by the file-writing path (§ `record_writer` + `log_uploader`). MQTT is a live-monitoring side-channel.
- Message payload: a `NoiseRecording` protobuf message with header fields populated and exactly one `Record` in `records[]`. Each MQTT message is fully self-describing.

### HTTPS (batch file upload)

Reuse `log_uploader.c` directly. Endpoint, host, and authentication to be configured later. The retry-on-failure timer (5-minute backoff) and HTTP status handling (201/409 = handled and delete, 400 = handled and delete, others = retry) carry over unchanged.

**Backlog handling is automatic.** When WiFi reconnects after an outage, `log_uploader` is notified (by `wifi_connect.c` on `IP_EVENT_STA_GOT_IP`) and iterates `LOG_DIR`, uploading any accumulated files. The same iteration handles boot-time scans (existing files from a previous run) and per-file notifications from `record_writer`. No additional logic needed beyond the contactless-inherited behavior. Files are uploaded in `readdir` order (no chronological sorting).

### BLE (live monitoring fallback)

A parallel live-monitoring channel useful when WiFi is unavailable, or when a nearby operator wants on-device readings without server infrastructure. BLE runs continuously alongside WiFi — not as a fallback that activates only when WiFi fails, but as an always-available secondary channel.

#### Bluetooth stack and configuration

Use **NimBLE** (preferred for new ESP-IDF projects — smaller footprint than Bluedroid). In `sdkconfig.defaults`:

```
CONFIG_BT_ENABLED=y                    # overrides the §12 default
CONFIG_BT_NIMBLE_ENABLED=y
CONFIG_BT_CONTROLLER_ENABLED=y
CONFIG_BT_NIMBLE_ROLE_PERIPHERAL=y
CONFIG_BT_NIMBLE_ROLE_CENTRAL=n        # we only act as peripheral
CONFIG_BT_NIMBLE_ROLE_OBSERVER=n
CONFIG_BT_NIMBLE_ROLE_BROADCASTER=n
```

Adds approximately 25 KB RAM and 80 KB flash. Comfortably within ESP32-S3-WROOM-1-N16 budget.

#### Advertising and connection model

- Advertise continuously while powered. Advertising interval: 1 second (low power, ~1–2 mA).
- Device name: `"NoiseMonitor-XXXX"` where `XXXX` is the last 4 hex characters of `device_id`.
- Accept incoming connections; one client at a time (BLE peripheral default).
- BLE and WiFi run concurrently. Both add value; neither displaces the other.

#### GATT services

Use the standard Device Information Service; one custom service for noise-specific data and telemetry. No standard Battery Service — raw voltage is exposed in the custom service instead.

**Device Information Service** (UUID `0x180A`):

```
Manufacturer Name (0x2A29) [READ]           "Kulturspektakel"
Firmware Revision (0x2A26) [READ]           e.g. "1.0.0"
Serial Number     (0x2A25) [READ]           device_id (hex string)
```

**Custom Noise Service** (UUID: random 128-bit, defined in firmware):

```
Characteristic              Properties        Encoding
─────────────────────────────────────────────────────────────────────
Status                      READ, NOTIFY      uint8 flags + uint16 log_files_pending
                                              bit 0: wifi_connected
                                              bit 1: usb_connected
                                              bit 2: calibrated
                                              bit 3: time_set

LAeq,1s                     READ, NOTIFY      uint8 (byte = (dB − 20) × 2)
LAeq,1m                     READ, NOTIFY      uint8
LAeq,15m                    READ, NOTIFY      uint8 + uint16 seconds_in_window
LAeq,30m                    READ, NOTIFY      uint8 + uint16 seconds_in_window
LCeq,1m                     READ, NOTIFY      uint8
LCeq,15m                    READ, NOTIFY      uint8 + uint16 seconds_in_window
LAFmax,1m                   READ, NOTIFY      uint8
LCpeak,1m                   READ, NOTIFY      uint8

Battery voltage             READ, NOTIFY      uint16 millivolts
                                              (sourced from power_management.c::battery_voltage)
Uptime seconds              READ              uint32
Calibration offset          READ              sint16, dB × 100
```

Encoding for dB values matches the protobuf record encoding (uint8 with `byte = (dB − 20) × 2`), so the phone app can share a decoder. Pending-upload count is part of the Status characteristic payload (already exposed as `log_files_pending`).

#### Subscription-aware notification

Only emit notifications to subscribed clients. Implement via NimBLE's CCCD-aware notify path: track subscription state per characteristic, gate the notify call on subscription. Characteristics remain readable on-demand regardless of subscription.

```c
if (ble_connected && characteristic_subscribed[i]) {
    ble_gatts_notify_custom(conn_handle, char_handles[i], om);
}
// else: value updated in GATT table; client can READ if interested
```

#### Notification rates

```
Characteristic                   Notify rate when subscribed
──────────────────────────────────────────────────────────────────
LAeq,1s                          1 Hz (live readout)
LAeq,1m                          1 Hz
LAFmax,1m                        1 Hz
LCpeak,1m                        1 Hz
LCeq,1m                          1 Hz
LAeq,15m                         on ≥ 0.5 dB change OR every 30 s
LAeq,30m                         on ≥ 0.5 dB change OR every 30 s
LCeq,15m                         on ≥ 0.5 dB change OR every 30 s
Status                           on any flag change OR on log_files_pending change OR every 10 s
Battery voltage                  on ≥ 50 mV change OR every 30 s
```

Change-based notify avoids spamming the phone with rounding-noise updates on slow-moving aggregates.

#### Ring buffers for aggregate computation

`audio_dsp` maintains three statically-allocated ring buffers of per-second linear-energy values (one for each weighting that requires aggregation), plus tracking of total seconds observed since boot:

```c
// File-scope statics in audio_dsp.c
static float laeq_ring_1m[60];      // last 60 seconds
static float laeq_ring_15m[900];    // last 900 seconds
static float laeq_ring_30m[1800];   // last 1800 seconds
static float lceq_ring_1m[60];
static float lceq_ring_15m[900];
static int   ring_index = 0;        // monotonic, modulo per ring
static int   total_seconds = 0;     // saturating at 30*60
```

Total static memory: ~14 KB across all rings. Each second, append the new linear-energy value to each ring (overwriting oldest). Compute Leq over the current contents:

```c
int n = min(total_seconds, RING_SIZE);
float sum = sum_of_first_n_entries_in_ring(n);
float leq = 10.0f * log10f(sum / n);
```

For partial windows (just-booted device), `seconds_in_window` is set to `min(total_seconds, RING_SIZE)` so the phone app can display "LAeq,15m (8 min so far)" with a caveat.

#### `ble_publisher` task (Core 0)

New FreeRTOS task pinned to Core 0:

1. Initialize NimBLE stack
2. Register GATT services and characteristics from the table above
3. Start advertising (1 Hz interval, includes Custom Noise Service UUID)
4. On CCCD writes: update `characteristic_subscribed[i]` array
5. Receive per-second record + aggregate values from `audio_dsp` via a dedicated queue (separate from MQTT queue — different rate-limiting needs)
6. Update characteristic values in GATT table
7. Send notifications to subscribed clients per the rate table above
8. Stack: 4096 bytes

---

## 10. Calibration

Each device must be calibrated once before deployment. Calibration produces a single dB offset that's added to all band/peak readings before encoding, and is persisted in NVS so it survives reboot. While uncalibrated **or while the system clock has not been set**, the device does not log to flash and does not publish to MQTT — instead it prints raw broadband readings to the console once per second so the operator can compare against a reference (NIOSH SLM phone app or any handheld SLM) and compute the offset.

The clock-not-set precondition is necessary because file names embed a Unix timestamp; emitting data without a valid clock would produce non-sortable filenames and make the ring-buffer eviction logic ambiguous.

### NVS schema

Namespace: `noise_cal`

```
key                   type    content
─────────────────────────────────────────────────────────
offset_db_x100        int32   signed; e.g. +260 = +2.60 dB
                              presence of this key = device is calibrated
```

Single key. `nvs_get_i32` returning `ESP_ERR_NVS_NOT_FOUND` means uncalibrated; any value (including 0) means calibrated. No separate boolean flag.

### Event group bit

Add `CALIBRATED = BIT8` to `event_group.h`. Set when NVS contains `offset_db_x100`; cleared when NVS doesn't or `CAL_CLEAR` is issued.

### Boot logic

1. Read `noise_cal/offset_db_x100` from NVS
2. If present: store value in a global `calibration_offset_db` (float), set `CALIBRATED` event bit
3. If absent: leave offset at 0.0, do not set `CALIBRATED` bit, log a warning

### Behavior when fully ready (CALIBRATED && TIME_SET)

- `audio_dsp` adds `calibration_offset_db` to every band/peak value during the dB conversion step (after `10 · log₁₀`, before uint8 encoding)
- `record_writer` consumes records from its queue, writes files normally
- `mqtt_publisher` publishes records normally
- `NoiseRecording` in every file and every MQTT publish carries `calibration_offset_db_x100` so the server knows what offset produced the values

### Behavior when not ready (uncalibrated or no clock)

- `audio_dsp` runs DSP normally; if `CALIBRATED` is unset, the offset applied is 0.0
- Records are **not** pushed to either queue
- Once per second, the DSP task prints state and current readings to the ESP-IDF log:

```
I (32145) cal: NOT LOGGING — calibrated=no, time_set=no. Current readings (raw, offset=0):
I (32145) cal:   LAeq   = 84.2 dB(A)
I (32145) cal:   LCeq   = 89.1 dB(C)
I (32146) cal:   LAFmax = 86.5 dB(A)
I (32146) cal:   LCpeak = 95.7 dB(C)
I (32146) cal: To calibrate, send: CAL_SET <(reference - LAeq) * 100>
```

The `calibrated` and `time_set` fields reflect the current event group state. Once both are set, normal logging begins.

### `cal_console` task — USB-CDC commands

Small task that reads lines from `stdin` (USB-CDC) and parses three commands:

```
CAL_SET <offset_db_x100>
  - parse signed integer argument
  - write to NVS key offset_db_x100
  - update global calibration_offset_db
  - set CALIBRATED event bit
  - reply: "OK offset=+X.XX dB"

CAL_GET
  - if CALIBRATED bit set:
      reply: "offset=+X.XX dB"
  - else:
      reply: "uncalibrated"

CAL_CLEAR
  - erase NVS key offset_db_x100
  - reset global calibration_offset_db to 0.0
  - clear CALIBRATED event bit
  - reply: "OK calibration cleared"
```

Implementation: `fgets` from `stdin` in a loop, `strtok` parsing, `nvs_set_i32` for persistence. ~50 lines of code. ESP-IDF's `esp_console` component is acceptable but not required.

### Calibration workflow (operator-facing)

1. Plug device into laptop via USB
2. Run `idf.py monitor` or any serial terminal at 115200 baud
3. Device boots, prints "UNCALIBRATED" output once per second
4. Place phone (NIOSH SLM app) or handheld SLM within a few cm of the ICS-43434 mic
5. In a steady noise environment (~50–80 dB ambient, e.g. office hum, running shower, white-noise speaker), let both readings settle
6. Compute: `offset = reference_LAeq - device_LAeq` (in dB)
7. Send: `CAL_SET <offset * 100>` (e.g., for +2.60 dB: `CAL_SET 260`)
8. Device confirms storage, sets `CALIBRATED` bit, begins normal logging
9. Optional: verify by checking the device's next reported LAeq matches the reference

### Edge cases

- **`CAL_SET 0`** is valid — explicit "zero offset" calibration. Sets `CALIBRATED` bit. Distinguishable from uncalibrated.
- **NVS read failure** (corrupt storage): treat as uncalibrated, print error to console.

---

## 11. RGB Status LED

Reuse `power_management.c::ledc_init()` and `set_rgb_color()`. State mapping is implementation-defined but should distinguish at minimum:

- Uncalibrated (no NVS calibration; awaiting `CAL_SET`)
- Healthy (calibrated + recording + WiFi connected + uploads succeeding)
- Recording but WiFi disconnected
- Error state (storage full / mic fault / etc.)

Specific colors are implementation-defined.

---

## 12. Power Configuration

The DSP load is moderate, the WiFi load is bursty, and most of the device's time is spent waiting for I²S buffers. A few baseline configurations cut current draw substantially without affecting functionality.

### Static configuration (sdkconfig.defaults)

```
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=160
# Bluetooth: enabled with NimBLE for live monitoring fallback (see §9 BLE)
CONFIG_BT_ENABLED=y
CONFIG_BT_NIMBLE_ENABLED=y
```

- **CPU at 160 MHz** instead of 240 MHz default. The DSP loop (FFT + 1/3-octave grouping + per-sample weighting) fits comfortably in Core 1's budget at 160 MHz with significant headroom. Saves ~25 mA continuous.
- **Bluetooth enabled with NimBLE.** Required for the BLE live-monitoring channel (§9). Continuous advertising costs ~1–2 mA; an active connection with 1 Hz notifications costs ~5–7 mA. We accept this overhead in exchange for BLE-based monitoring being available when WiFi isn't.

### Runtime configuration

In `wifi_connect.c`:

After WiFi association (on `IP_EVENT_STA_GOT_IP`):

```c
esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
```

On disconnect (`WIFI_EVENT_STA_DISCONNECTED`):

```c
esp_wifi_stop();   // power down radio between retry attempts
```

Before each reconnect attempt (timer-fired retry path):

```c
esp_wifi_start();   // bring radio back up
esp_wifi_connect();
```

- **WiFi modem-sleep** (`WIFI_PS_MIN_MODEM`) keeps the AP association but lets the radio sleep between beacons. Drops idle WiFi current from ~50–70 mA to ~10–20 mA. Brief wake-ups for the 1 Hz MQTT publish don't materially affect this.
- **Radio off when disconnected** (`esp_wifi_stop()` between retries) saves the ~30–50 mA the radio draws while idle-but-not-associated. Stop/start cycle is ~100–300 ms, negligible against the 5-minute retry interval. The `WIFI_EVENT_STA_STOP` event fires during intentional stop; the handler should ignore it (or check an "intentional stop" flag).

### Expected total current draw

```
State                                               Approx current
─────────────────────────────────────────────────────────────────────
Default config (240 MHz, no WiFi PS), connected     ~110–130 mA
160 MHz + modem-sleep + BLE advertising only        ~57–77 mA
160 MHz, WiFi disconnected, BLE advertising only    ~27–42 mA
160 MHz + WiFi connected + BLE client subscribed    ~62–82 mA
```

The BLE column adds ~2–7 mA depending on whether a BLE client is connected and subscribed. Trivial during advertising; modest while a phone is actively monitoring.

### Levers not used (documented for future reference)

These were considered and intentionally not implemented in v1:

- **CPU at 80 MHz**: technically achievable but the FFT loop becomes tight against the 1-second deadline. Revisit only if profiling shows comfortable margin at 80 MHz.
- **Light sleep between I²S buffers**: marginal savings, fragile against I²S DMA timing, complex.
- **Dynamic frequency scaling (ESP-IDF `pm_config`)**: load is constant per second, no benefit.
- **Reduced I²S sample rate (24 kHz)**: would lose 12.5 / 16 kHz bands. Spec change, not a power optimization.
- **Smaller FFT (2048-pt)**: trades band-edge precision for CPU. Revisit only if 4096-pt budget is tight.

---

## 13. DSP Optimization

The ESP32-S3 has SIMD-style vector instructions (PIE — Processor Instruction Extensions) that dramatically accelerate FFT and filter operations. Espressif's `esp-dsp` library provides hand-tuned assembly variants. Default function variants compile but don't use the silicon's accelerator; we must opt in explicitly.

### Component dependency

Add to `main/idf_component.yml`:

```yaml
espressif/esp-dsp: "^1.5.0"
```

### Use ESP32-S3 PIE-accelerated function variants

ESP-DSP exports each function in multiple flavors:

```
no suffix or _ansi   portable C, slowest
_ae32                Xtensa LX6 (original ESP32) optimized
_aes3                ESP32-S3 PIE optimized — use these
```

The library auto-selects `_aes3` when targeting ESP32-S3 if the unsuffixed name is called, but explicitly invoking `_aes3` makes the intent clear and triggers a compile error on wrong target.

### FFT configuration

- **Radix-4** (`dsps_fft4r_*`) instead of radix-2 (`dsps_fft2r_*`). 4096 = 4^6, so radix-4 applies. ~30% faster on S3.
- **Real-input mode**: pack 4096 real samples as 2048 complex pairs, run half-size FFT, unpack with `dsps_cplx2real_fc32`. ~2× speedup over naive complex.
- **Initialize once at boot**:

```c
dsps_fft4r_init_fc32(NULL, 4096);
```

### Pre-computed tables in static DRAM

Compute once at init, reference for the lifetime of the program. Never recompute per-FFT.

```c
static float hann_window[4096];                    // dsps_wind_hann_f32
static const float a_weight_coeffs[4][5];          // IEC 61672 A-weighting cascade
static const float c_weight_coeffs[2][5];          // IEC 61672 C-weighting cascade
static int   band_edges[32];                       // FFT bin index per 1/3-octave boundary
```

`dsps_wind_hann_f32` from ESP-DSP generates the Hann window — no need to compute manually.

### Per-sample filtering

A-weighting and C-weighting are biquad cascades applied at the audio sample rate (48 kHz). Use `dsps_biquad_f32_aes3` per section. State is preserved across calls.

### Compiler optimization

In `sdkconfig.defaults`:

```
CONFIG_COMPILER_OPTIMIZATION_PERF=y
```

Sets global default to `-O2` instead of the size-optimizing `-Os`. Slightly more flash, meaningfully faster code. Acceptable trade-off given 16 MB flash.

### Float math throughout

ESP32-S3 has a per-core single-precision FPU. Use `float` for all DSP variables. Don't fixed-point optimize — no benefit, adds bug surface. FreeRTOS auto-saves FPU context on task switches (`CONFIG_FREERTOS_FPU_IN_TASK=y` is default).

### Memory placement

Default flash + I-cache placement is sufficient for DSP loops. The S3's instruction cache holds hot DSP code with no measurable performance penalty vs IRAM. **Do not** preemptively use `IRAM_ATTR`; revisit only if profiling shows cache misses cause audible jitter or 1-second deadline misses.

### Task pinning and priority

Already specified in §6: `audio_dsp` pinned to Core 1 at `TASK_PRIO_HIGH`. The DSP loop should never voluntarily yield except waiting on I²S DMA buffer completion.

---

## 14. Deliverables

1. ESP-IDF project structure mirroring contactless layout
2. `partitions.csv`, `sdkconfig.defaults` (with §12 power-config and §13 `CONFIG_COMPILER_OPTIMIZATION_PERF=y` additions), `dependencies.lock`, `CMakeLists.txt` (root + `main/`)
3. Modified `components/nanopb/logmessage.proto` (adds `noise_recording` field at index 11)
4. New `components/nanopb/noise.proto` and `noise.options` defining `Record` and `NoiseRecording`
5. Modified `main/event_group.h` (adds `CALIBRATED = BIT8`, drops `LOCAL_CONFIG_LOADED` from `STARTUP_BITS`)
6. `main/main.c` with task list as defined in §6 (no `local_config`, `fetch_config`, `state_machine`, etc.)
7. Reused contactless files (with renames where needed). Dropped: `local_config.c`/`.h`, `fetch_config.c`/`.h`, `config.proto`, `configs.proto`, `config.options`, `configs.options`
8. Adapted `log_writer.c` populating `LogMessage.noise_recording` (matches contactless write pattern)
9. Adapted `wifi_connect.c` with the §12 power-management changes: modem-sleep on connect, radio stop/start across disconnect-retry cycles
10. New components: `audio_dsp.c`, `mqtt_publisher.c`, `calibration.c`, `cal_console.c`, `ble_publisher.c` — `audio_dsp.c` uses ESP-DSP `_aes3` PIE-accelerated variants per §13, and maintains static ring buffers for 1m/15m/30m aggregates per §9 BLE
11. Updated `idf_component.yml` with all required dependencies (including `espressif/esp-dsp`); NimBLE is built into ESP-IDF and activated via sdkconfig
12. `scripts/proto.sh` regenerated for the new schema (regenerates both `noise.pb.{c,h}` and `logmessage.pb.{c,h}`)
13. Builds successfully with `idf.py build` targeting `esp32s3`

---

## 15. Out of Scope (do not implement)

- Cellular connectivity
- Display
- Battery deep-sleep / power-off (the contactless `power_management.c` deep-sleep logic is not used)
- Pistonphone-based calibration (NIOSH SLM phone app or handheld SLM is the supported reference)
- Production server stack
- Pre-deployment provisioning workflow
- Audio recording / event-triggered audio capture
- GPS

---

## Appendix: Standard A/C Weighting Tables (for server-side derivation)

These are not implemented on-device but are needed by any server-side consumer that derives LAeq/LCeq from `bands`:

A-weighting corrections (dB) per band index 0..30:
```
−56.7, −50.5, −44.7, −39.4, −34.6, −30.2, −26.2, −22.5, −19.1, −16.1,
−13.4, −10.9, −8.6,  −6.6,  −4.8,  −3.2,  −1.9,  −0.8,   0.0,  +0.6,
 +1.0,  +1.2,  +1.3,  +1.2,  +1.0,  +0.5,  −0.1,  −1.1,  −2.5,  −4.3,
 −6.6
```

C-weighting corrections (dB) per band index 0..30:
```
−8.5, −6.2, −4.4, −3.0, −2.0, −1.3, −0.8, −0.5, −0.3, −0.2,
−0.1,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
−0.1, −0.2, −0.3, −0.5, −0.8, −1.3, −2.0, −3.0, −4.4, −6.2,
−8.5
```

Derivation formula:

```
LAeq = 10 · log₁₀ ( Σ over bands [ 10^( (band_dB[i] + A[i]) / 10 ) ] )
LCeq = 10 · log₁₀ ( Σ over bands [ 10^( (band_dB[i] + C[i]) / 10 ) ] )
```

The on-device cached `laeq_1s` and `lceq_1s` are computed using these same tables.
