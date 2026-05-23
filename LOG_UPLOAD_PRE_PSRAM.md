# Log upload on the no-PSRAM board

Open issue: HTTPS log uploads from `log_uploader` can't reliably establish
a TLS session while the `mqtt_publisher` task already has one. We have
~150 KB of internal SRAM total; two simultaneous mbedtls sessions plus
WiFi/LWIP buffers plus everything else don't fit.

Once the **ESP32-S3-N16R2 (2 MB PSRAM)** board arrives this resolves
itself — see `PSRAM_MIGRATION.md`. The mbedtls buffers route to PSRAM
(`CONFIG_SPIRAM_USE_MALLOC=y`) and the contiguous-RAM contention goes
away. Items 7 and 11 there cover the trims that need restoring.

## What we already tried

| Change | Result |
|---|---|
| `log_uploader` waits for `MQTT_CONNECTED` event-group bit before opening TLS | Eliminated the boot-time race against MQTT's own handshake; didn't fix the fundamental memory shortage. |
| Retry interval 5 min → 30 s | Faster feedback but same outcome on every retry. |
| `CONFIG_MBEDTLS_SSL_IN/OUT_CONTENT_LEN` 4096 → 2048 | mbedtls alloc now succeeds (no more `MBEDTLS_ERR_SSL_ALLOC_FAILED`), but the WiFi driver runs out of packet buffers during the handshake (`wifi:m f null` storm) and the TCP connection times out (`esp-tls: Failed to open new connection in specified timeout`). |

Net: we moved the failure from the mbedtls layer to the WiFi-driver
layer. Symptoms now look like:

```
I (3979) log_uploader: Uploading /littlefs/logs/...
W (5099) wifi:m f null    [...many lines...]
W (6959) wifi:m f null
E (9089) esp-tls: Failed to open new connection in specified timeout
E (9089) log_uploader: Request failed (Error: ESP_ERR_HTTP_CONNECT)
```

## Options to make it work today

Pick one if you need to validate the upload path / develop the backend
before the PSRAM board lands. Each is a few lines + reflash.

### A. Pause MQTT for the duration of the upload (recommended)

Add a `mqtt_publisher_pause()` / `mqtt_publisher_resume()` API that
internally calls `esp_mqtt_client_stop()` / `esp_mqtt_client_start()`.
`log_uploader` calls these around each upload attempt.

- **Pro:** real fix, no privacy/transport changes, works regardless of
  what the backend exposes.
- **Pro:** harmless to keep after PSRAM lands (becomes effectively a
  no-op since both can coexist).
- **Con:** ~1 s gap in 1 Hz MQTT records per uploaded file (one missing
  record on the live MQTT topic).
- **Effort:** ~30 lines (`mqtt_publisher.{c,h}` + `log_uploader.c`).
- **Sketch:**
  ```c
  // mqtt_publisher.c
  void mqtt_publisher_pause(void)  { esp_mqtt_client_stop(mqtt_client); }
  void mqtt_publisher_resume(void) { esp_mqtt_client_start(mqtt_client); }

  // log_uploader.c, just before opening HTTPS:
  mqtt_publisher_pause();
  status = upload_file(filename);
  mqtt_publisher_resume();
  ```

### B. Drop HTTPS → plain HTTP for log uploads

Have the backend accept plain HTTP on the log-upload endpoint (or stick a
TLS-terminating reverse proxy in front, like Caddy/nginx). `log_uploader`
then doesn't use mbedtls at all; only MQTT does.

- **Pro:** biggest heap win — no second mbedtls session ever.
- **Pro:** simplest log_uploader code path; one less thing that can fail.
- **Con:** log payloads travel cleartext. Noise-dB data probably isn't
  sensitive, but verify with whoever owns the deployment.
- **Effort:** `esp_http_client_config_t.transport_type =
  HTTP_TRANSPORT_OVER_TCP` and change the URL scheme. ~3 lines.

### C. Disable MQTT entirely for now

Comment out the `xTaskCreate(&mqtt_publisher, ...)` in `main.c`. With
MQTT gone, `log_uploader` has the full heap to itself and uploads work
trivially.

- **Pro:** zero-effort hack. Backend dev proceeds immediately.
- **Con:** no live data path during dev. Re-enable for production /
  PSRAM board, at which point you re-discover the same coexistence
  issue (or PSRAM has arrived and it just works).
- **Effort:** 1 line.

### D. Just keep retrying

Leave the 30 s retry. With WiFi-buffer starvation it doesn't seem to
succeed within a boot cycle, so this isn't useful for backend dev today.
Documented for completeness.

## Recommendation order

1. **B** if the backend can run plain HTTP (or you can put a proxy in
   front). Simplest, cleanest, biggest heap win, and good even
   post-PSRAM.
2. **A** if HTTPS is required end-to-end. Sustainable; ~30 lines.
3. **C** as a development-time hack while building the backend.

## When PSRAM arrives

Per `PSRAM_MIGRATION.md`:
- Restore `MBEDTLS_SSL_IN/OUT_CONTENT_LEN` from 2048 → 16384 (item 7).
- Restore WiFi RX/TX buffer counts (item 11) with
  `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y`.
- If you went with option **A** above, the pause/resume becomes a no-op
  in practice but can be left in place as defense-in-depth.
- If you went with option **B**, decide separately whether to flip
  log_uploader back to HTTPS or leave it as HTTP behind a proxy
  (probably the latter, since one less mbedtls session is always a
  reasonable design choice).
