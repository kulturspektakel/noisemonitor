#include "cal_console.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "calibration.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_err.h"
#include "event_group.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

static void handle_line(char* line) {
  size_t len = strlen(line);
  while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r' || line[len - 1] == ' ')) {
    line[--len] = '\0';
  }
  if (len == 0)
    return;

  char* cmd = strtok(line, " ");
  if (!cmd)
    return;

  if (strcmp(cmd, "CAL_SET") == 0) {
    char* arg = strtok(NULL, " ");
    if (!arg) {
      printf("ERR usage: CAL_SET <offset_db_x100>\n");
      return;
    }
    char* end = NULL;
    long v = strtol(arg, &end, 10);
    if (end == arg || *end != '\0') {
      printf("ERR invalid integer: %s\n", arg);
      return;
    }
    esp_err_t err = calibration_set((int32_t)v);
    if (err != ESP_OK) {
      printf("ERR calibration_set: %s\n", esp_err_to_name(err));
      return;
    }
    printf("OK offset=%+.2f dB\n", (float)v / 100.0f);
  } else if (strcmp(cmd, "CAL_GET") == 0) {
    if (xEventGroupGetBits(event_group) & CALIBRATED) {
      printf("offset=%+.2f dB\n", calibration_offset_db());
    } else {
      printf("uncalibrated\n");
    }
  } else if (strcmp(cmd, "CAL_CLEAR") == 0) {
    esp_err_t err = calibration_clear();
    if (err != ESP_OK) {
      printf("ERR calibration_clear: %s\n", esp_err_to_name(err));
      return;
    }
    printf("OK calibration cleared\n");
  } else {
    printf("ERR unknown command: %s\n", cmd);
  }
}

void cal_console(void* params) {
  // Default USB-Serial-JTAG console on ESP32-S3 uses a non-blocking VFS that
  // makes fgets() return immediately at EOF. Install the interrupt-driven
  // driver and swap the VFS so stdin actually delivers typed bytes.
  usb_serial_jtag_driver_config_t usj_cfg = {
      .rx_buffer_size = 256,
      .tx_buffer_size = 256,
  };
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usj_cfg));
  usb_serial_jtag_vfs_use_driver();
  setvbuf(stdin, NULL, _IONBF, 0);

  char line[128];
  while (true) {
    if (fgets(line, sizeof(line), stdin) != NULL) {
      handle_line(line);
    } else {
      clearerr(stdin);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}
