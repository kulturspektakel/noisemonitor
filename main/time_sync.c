#include "time_sync.h"
#include <stdint.h>
#include <sys/time.h>
#include <time.h>
#include "constants.h"
#include "ds3231.h"
#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "event_group.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "network_request.h"

static const char* TAG = "time_sync";

static bool rtc_time_is_valid(const struct tm* rtc_time) {
  if (rtc_time->tm_year < 123 || rtc_time->tm_year > 199
      || rtc_time->tm_mon < 0 || rtc_time->tm_mon > 11
      || rtc_time->tm_hour < 0 || rtc_time->tm_hour > 23
      || rtc_time->tm_min < 0 || rtc_time->tm_min > 59
      || rtc_time->tm_sec < 0 || rtc_time->tm_sec > 59) {
    return false;
  }
  static const int days_per_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  int days = days_per_month[rtc_time->tm_mon];
  int year = rtc_time->tm_year + 1900;
  if (rtc_time->tm_mon == 1 && year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) {
    days++;
  }
  return rtc_time->tm_mday >= 1 && rtc_time->tm_mday <= days;
}

static bool decode_rtc_bcd(uint8_t value, int* decoded) {
  if ((value & 0x0f) > 9 || (value >> 4) > 9) return false;
  *decoded = (value >> 4) * 10 + (value & 0x0f);
  return true;
}

static esp_err_t read_rtc_time(i2c_dev_t* device, struct tm* rtc_time) {
  uint8_t registers[7];
  I2C_DEV_TAKE_MUTEX(device);
  I2C_DEV_CHECK(device, i2c_dev_read_reg(device, 0x00, registers, sizeof(registers)));
  I2C_DEV_GIVE_MUTEX(device);

  struct tm decoded = {0};
  bool twelve_hour = (registers[2] & 0x40) != 0;
  if ((registers[2] & 0x80) != 0 || (registers[5] & 0xe0) != 0
      || !decode_rtc_bcd(registers[0], &decoded.tm_sec)
      || !decode_rtc_bcd(registers[1], &decoded.tm_min)
      || !decode_rtc_bcd(registers[2] & (twelve_hour ? 0x1f : 0x3f), &decoded.tm_hour)
      || !decode_rtc_bcd(registers[3], &decoded.tm_wday)
      || !decode_rtc_bcd(registers[4], &decoded.tm_mday)
      || !decode_rtc_bcd(registers[5], &decoded.tm_mon)
      || !decode_rtc_bcd(registers[6], &decoded.tm_year)) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  if (twelve_hour) {
    if (decoded.tm_hour < 1 || decoded.tm_hour > 12) return ESP_ERR_INVALID_RESPONSE;
    decoded.tm_hour = decoded.tm_hour % 12 + ((registers[2] & 0x20) ? 12 : 0);
  }
  decoded.tm_wday--;
  decoded.tm_mon--;
  decoded.tm_year += 100;
  if (decoded.tm_wday < 0 || decoded.tm_wday > 6 || !rtc_time_is_valid(&decoded)) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  *rtc_time = decoded;
  return ESP_OK;
}

void time_sync(void* params) {
  ESP_ERROR_CHECK(i2cdev_init());
  i2c_dev_t dev = {
      .addr = DS3231_ADDR,
  };
  ESP_ERROR_CHECK(ds3231_init_desc(&dev, I2C_NUM_0, 39, 38));

  struct tm rtc_time = {0};
  esp_err_t rtc_result = read_rtc_time(&dev, &rtc_time);

  if (rtc_result == ESP_OK && rtc_time_is_valid(&rtc_time)) {
    struct timeval tv = {.tv_sec = mktime(&rtc_time)};
    if (tv.tv_sec != (time_t)-1 && settimeofday(&tv, NULL) == 0) {
      xEventGroupSetBits(event_group, TIME_SET);
      ESP_LOGI(TAG, "RTC time seems valid, setting system time to %s UTC", asctime(&rtc_time));
    } else {
      ESP_LOGW(TAG, "Failed to set system time from RTC. Waiting for NTP.");
    }
  } else {
    ESP_LOGW(TAG, "RTC read failed or date is invalid (%s). Waiting for NTP.",
             esp_err_to_name(rtc_result));
  }

  // set timezone after settimeofday, because it receives UTC
  setenv("TZ", TZ, 1);
  tzset();

  // wait for wifi to connect
  xEventGroupWaitBits(event_group, WIFI_CONNECTED, false, true, portMAX_DELAY);

  // Multi-server SNTP: time.apple.com is whitelisted by iPhone hotspots;
  // time.google.com and time.cloudflare.com are widely reachable and don't
  // depend on the rotating pool.ntp.org pool resolving to a routable server.
  esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(
      3, ESP_SNTP_SERVER_LIST("time.apple.com", "time.google.com", "time.cloudflare.com"));

  while (true) {
    ESP_LOGI(TAG, "Fetching time from NTP");
    xSemaphoreTake(network_request, portMAX_DELAY);
    esp_err_t ret = esp_netif_sntp_init(&config);
    if (ret == ESP_OK) {
      ret = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(15000));
      esp_netif_sntp_deinit();
    }
    xSemaphoreGive(network_request);

    if (ret == ESP_OK) {
      time_t now;
      time(&now);
      struct tm timeinfo;
      if (gmtime_r(&now, &timeinfo) != NULL) {
        ESP_LOGI(TAG, "Received NTP time. Setting RTC to %s UTC", asctime(&timeinfo));
        esp_err_t rtc_write_result = ds3231_set_time(&dev, &timeinfo);
        if (rtc_write_result != ESP_OK) {
          ESP_LOGW(TAG, "Failed to update RTC: %s", esp_err_to_name(rtc_write_result));
        }
      }
      xEventGroupSetBits(event_group, TIME_SET);
      break;
    }
    ESP_LOGW(TAG, "NTP sync failed (%s); retrying in 60 s", esp_err_to_name(ret));
    vTaskDelay(pdMS_TO_TICKS(60000));
  }
  ds3231_free_desc(&dev);

  ESP_LOGI(TAG, "Done, terminating task");
  vTaskDelete(NULL);
}
