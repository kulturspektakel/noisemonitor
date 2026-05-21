#include <stdio.h>
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

  // Load persisted calibration into the in-RAM offset + set CALIBRATED bit if present.
  // Devices that have never been calibrated start with offset=0 and produce
  // raw-FFT-magnitude dB values; the operator can calibrate via the BLE
  // "cal_offset" characteristic (read/write int32 hundredths-of-dB).
  calibration_init();

  // Inter-task queues (spec §6). Sized 16 = ~16 s of buffering against any
  // reasonable consumer delay. DSP sends are non-blocking.
  record_writer_queue  = xQueueCreate(16, sizeof(record_t));
  mqtt_publisher_queue = xQueueCreate(16, sizeof(record_t));
  ble_publisher_queue  = xQueueCreate(16, sizeof(record_t));

  // Reused infrastructure (Core 0)
  xTaskCreate(&wifi_connect,     WIFI_CONNECT_TASK,     4096, NULL, TASK_PRIO_NORMAL, NULL);
  xTaskCreate(&time_sync,        "time_sync",           4096, NULL, TASK_PRIO_NORMAL, NULL);
  xTaskCreate(&log_uploader,     LOG_UPLOADER_TASK,     4096, NULL, TASK_PRIO_NORMAL, NULL);
  xTaskCreate(&power_management, POWER_MANAGEMENT_TASK, 4096, NULL, TASK_PRIO_NORMAL, NULL);
  xTaskCreate(&load_device_id,   "load_device_id",      3072, NULL, TASK_PRIO_NORMAL, NULL);
  xTaskCreate(&load_salt,        "load_salt",           3072, NULL, TASK_PRIO_NORMAL, NULL);

  xTaskCreate(&record_writer,    "record_writer",       4096, NULL, TASK_PRIO_NORMAL, NULL);

  // New
  xTaskCreate(&mqtt_publisher,   "mqtt_publisher",      4096, NULL, TASK_PRIO_NORMAL, NULL);
  xTaskCreate(&ble_publisher,    "ble_publisher",       4096, NULL, TASK_PRIO_NORMAL, NULL);
  xTaskCreate(&status_led,       "status_led",          2048, NULL, TASK_PRIO_NORMAL, NULL);

  // Real-time DSP pinned to Core 1, isolated from networking (§6).
  xTaskCreatePinnedToCore(&audio_dsp, "audio_dsp", 8192, NULL, TASK_PRIO_HIGH, NULL, 1);

  heap_diag("after task spawn");
}
