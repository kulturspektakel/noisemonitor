#include <stdio.h>

// Dev-time hacks for the no-PSRAM board, where NimBLE + the BT controller
// and the WiFi/mbedtls stack can't both fit in contiguous heap (see
// LOG_UPLOAD_PRE_PSRAM.md and PSRAM_MIGRATION.md). Pick at most ONE:
//
//   DEV_NO_BLE — boot WiFi/MQTT/upload only; BLE off. Calibration + WiFi
//                creds entry over BLE are unavailable, so creds must already
//                be in NVS. This is the normal field config pre-PSRAM.
//   DEV_NO_NET — boot BLE only; WiFi/MQTT/upload off. For BLE-only bench
//                tests. The net consumer tasks self-gate on WIFI_CONNECTED,
//                so skipping wifi_connect is enough to keep the stack (and
//                its heap) from ever coming up.
//
// Both become irrelevant once the PSRAM board lands; remove together with
// the other no-PSRAM workarounds then.
#define DEV_NO_BLE
// #define DEV_NO_NET

#include "audio_dsp.h"
#include "ble_publisher.h"
#include "calibration.h"
#include "constants.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "event_group.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heap_diag.h"
#include "log_uploader.h"
#include "mqtt_publisher.h"
#include "network_request.h"
#include "nvs_flash.h"
#include "power_management.h"
#include "record_writer.h"
#include "status_led.h"
#include "time_sync.h"
#include "wifi_connect.h"

EventGroupHandle_t event_group;
SemaphoreHandle_t network_request;

#if defined(DEV_NO_BLE) || defined(DEV_NO_NET)
// Stand-in for a disabled consumer task: keep its record queue drained so the
// DSP's non-blocking sends don't accumulate "queue full" warnings. The queue
// handle is passed as the task argument.
static void queue_drain(void* queue) {
  record_t r;
  while (true) xQueueReceive((QueueHandle_t)queue, &r, portMAX_DELAY);
}
#endif

void app_main(void) {
  // Dynamic frequency scaling between 40 MHz idle and 160 MHz active.
  // Light sleep is disabled: the I²S driver holds an APB-freq lock while
  // its RX channel is running, and the audio_dsp task on Core 1 runs at
  // ~100 % utilization (4096-pt FFT every 21 ms), so the kernel never
  // gets a window where both cores are idle. DFS is the only PM mode
  // that actually fires here.
  esp_pm_config_t pm_cfg = {
      .max_freq_mhz = 160,
      .min_freq_mhz = 40,
      .light_sleep_enable = false,
  };
  ESP_ERROR_CHECK(esp_pm_configure(&pm_cfg));

  heap_diag("before preinit");

  // Reserve the FFT twiddle table early (16 KB). LogMessage is in BSS
  // (record_writer.c) so it doesn't go through heap.
  ESP_ERROR_CHECK(audio_dsp_preinit());

  heap_diag("after fft_table alloc");

  event_group = xEventGroupCreate();
  network_request = xSemaphoreCreateBinary();
  xSemaphoreGive(network_request);

#ifdef DEV_NO_NET
  // No WiFi -> no SNTP, so TIME_SET would never fire and the DSP gates off all
  // record emission (audio_dsp.c). Force it so per-second records flow to the
  // BLE queue. Timestamps are bogus (clock starts at boot epoch) — acceptable
  // for a bench test; never ship with DEV_NO_NET defined.
  xEventGroupSetBits(event_group, TIME_SET);
#endif

  ESP_ERROR_CHECK(nvs_flash_init());
  esp_vfs_littlefs_conf_t conf = {
      .base_path = "/littlefs",
      .partition_label = "littlefs",
      .format_if_mount_failed = true,
      .dont_mount = false,
  };
  ESP_ERROR_CHECK(esp_vfs_littlefs_register(&conf));

  // The power_management task installs its own USB-plug ISR; the GPIO ISR
  // service must be installed before any task adds a handler.
  gpio_install_isr_service(ESP_INTR_FLAG_EDGE);

  // Load persisted per-band calibration if present.
  // Uncalibrated devices start with all-zero band offsets and still read near
  // dB SPL thanks to the fixed FFT-energy->dB-SPL anchor in audio_dsp.c; the
  // operator trims the per-band frequency response via the BLE calibration
  // characteristic (read/write 31 signed bytes, 0.5 dB steps).
  calibration_init();

  // Inter-task queues (spec §6). Sized 16 = ~16 s of buffering against any
  // reasonable consumer delay. DSP sends are non-blocking.
  record_writer_queue  = xQueueCreate(16, sizeof(record_t));
  mqtt_publisher_queue = xQueueCreate(16, sizeof(record_t));
  ble_publisher_queue  = xQueueCreate(16, sizeof(record_t));

  // Reused infrastructure (Core 0)
#ifndef DEV_NO_NET
  xTaskCreate(&wifi_connect,     WIFI_CONNECT_TASK,     4096, NULL, TASK_PRIO_NORMAL, NULL);
  xTaskCreate(&time_sync,        "time_sync",           4096, NULL, TASK_PRIO_NORMAL, NULL);
  xTaskCreate(&log_uploader,     LOG_UPLOADER_TASK,     4096, NULL, TASK_PRIO_NORMAL, NULL);
#endif
  xTaskCreate(&power_management, POWER_MANAGEMENT_TASK, 4096, NULL, TASK_PRIO_NORMAL, NULL);
  xTaskCreate(&load_device_id,   "load_device_id",      3072, NULL, TASK_PRIO_NORMAL, NULL);
  xTaskCreate(&load_salt,        "load_salt",           3072, NULL, TASK_PRIO_NORMAL, NULL);

  // 6144 (not 4096): flush_to_file's pb_encode plus esp_littlefs_info and the
  // LittleFS write/commit share this stack; 4096 overflowed the canary.
  xTaskCreate(&record_writer,    "record_writer",       6144, NULL, TASK_PRIO_NORMAL, NULL);

  // New
#ifdef DEV_NO_NET
  xTaskCreate(&queue_drain,      "mqtt_drain",          2048, mqtt_publisher_queue, TASK_PRIO_NORMAL, NULL);
#else
  xTaskCreate(&mqtt_publisher,   "mqtt_publisher",      4096, NULL, TASK_PRIO_NORMAL, NULL);
#endif
#ifdef DEV_NO_BLE
  xTaskCreate(&queue_drain,      "ble_drain",           2048, ble_publisher_queue, TASK_PRIO_NORMAL, NULL);
#else
  xTaskCreate(&ble_publisher,    "ble_publisher",       4096, NULL, TASK_PRIO_NORMAL, NULL);
#endif
  xTaskCreate(&status_led,       "status_led",          2048, NULL, TASK_PRIO_NORMAL, NULL);

  // Real-time DSP pinned to Core 1, isolated from networking (§6).
  xTaskCreatePinnedToCore(&audio_dsp, "audio_dsp", 8192, NULL, TASK_PRIO_HIGH, NULL, 1);

  heap_diag("after task spawn");
}
