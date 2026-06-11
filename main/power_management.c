#include "power_management.h"
#include "constants.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_event.h"
#include "esp_log.h"
#include "event_group.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define USB_CHANNEL ADC_CHANNEL_0
#define BATTERY_CHANNEL ADC_CHANNEL_1
#define USB_PIN GPIO_NUM_1
#define USB_VOLTAGE_THRESHOLD 1000

#define BATTERY_LOW 1700

#define UPDATE_INTERVAL 60000

#define LED_BLUE_PIN GPIO_NUM_40
#define LED_GREEN_PIN GPIO_NUM_41
#define LED_RED_PIN GPIO_NUM_42

int battery_voltage = 0;
int usb_voltage = 0;
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t battery_cali_handle = NULL;
static adc_cali_handle_t usb_cali_handle = NULL;
static TimerHandle_t voltage_update_timer;

int battery_percentage() {
  /*
  WITH A(v, p) AS (
    SELECT  "batteryVoltage", 1 - extract(EPOCH FROM ("deviceTime" - '2024-07-09 23:00:19')) / 74888
    FROM "DeviceLog"
    WHERE "deviceId" = 'Döner'
      AND "deviceTime" >= '2024-07-09 23:00:19'
      AND "deviceTime" < '2024-07-10 23:00:00'
    ORDER BY "createdAt" ASC
  )
  SELECT max(v) AS max, floor(p / 0.01) * 0.01 AS group_bin
  FROM A
  GROUP BY floor(p / 0.01)
  ORDER BY group_bin;
  */
  static const uint16_t voltages[101] = {
      1545, 1608, 1647, 1677, 1700, 1715, 1724, 1727, 1730, 1735, 1739, 1742, 1750, 1756, 1765,
      1769, 1779, 1789, 1794, 1803, 1810, 1822, 1828, 1833, 1838, 1845, 1850, 1854, 1858, 1862,
      1869, 1870, 1874, 1877, 1877, 1882, 1883, 1886, 1886, 1886, 1889, 1892, 1896, 1897, 1897,
      1898, 1900, 1902, 1902, 1902, 1907, 1908, 1908, 1910, 1911, 1913, 1914, 1915, 1923, 1925,
      1927, 1931, 1935, 1937, 1941, 1943, 1949, 1952, 1955, 1961, 1964, 1964, 1967, 1968, 1974,
      1977, 1977, 1978, 1979, 1982, 1982, 1982, 1985, 1985, 1987, 1989, 1989, 1989, 1991, 1991,
      1994, 1994, 1996, 1999, 2002, 2004, 2008, 2014, 2019, 2028, 2030
  };

  for (int i = 0; i < (sizeof(voltages) / sizeof(uint16_t)); i++) {
    if (battery_voltage < voltages[i]) {
      return i;
    }
  }
  return 100;
}

bool battery_is_low() {
  return battery_voltage < BATTERY_LOW;
}

static void adc_calibration_init(
    adc_unit_t unit,
    adc_channel_t channel,
    adc_atten_t atten,
    adc_cali_handle_t* out_handle
) {
  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = unit,
      .chan = channel,
      .atten = atten,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  adc_cali_create_scheme_curve_fitting(&cali_config, out_handle);
}

static void IRAM_ATTR gpio_interrupt_handler(void* args) {
  vTaskNotifyGiveFromISR(xTaskGetHandle(POWER_MANAGEMENT_TASK), NULL);
}

static void voltage_update_timer_callback(TimerHandle_t xTimer) {
  // Re-arm: this is a one-shot timer; reset it for next interval.
  xTimerReset(xTimer, 0);
  gpio_interrupt_handler(NULL);
}

static void ledc_init() {
  ledc_timer_config_t ledc_timer = {
      .duty_resolution = LEDC_TIMER_8_BIT,
      .freq_hz = 5000,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .timer_num = LEDC_TIMER_0
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel_config_t ledc_channel[3] = {
      {.channel = LEDC_CHANNEL_0,
       .duty = 0,
       .gpio_num = LED_RED_PIN,
       .flags = {.output_invert = 1},
       .speed_mode = ledc_timer.speed_mode,
       .timer_sel = ledc_timer.timer_num},
      {.channel = LEDC_CHANNEL_1,
       .duty = 0,
       .gpio_num = LED_GREEN_PIN,
       .flags = {.output_invert = 1},
       .speed_mode = ledc_timer.speed_mode,
       .timer_sel = ledc_timer.timer_num},
      {.channel = LEDC_CHANNEL_2,
       .duty = 0,
       .gpio_num = LED_BLUE_PIN,
       .flags = {.output_invert = 1},
       .speed_mode = ledc_timer.speed_mode,
       .timer_sel = ledc_timer.timer_num}
  };

  for (int ch = 0; ch < 3; ch++) {
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[ch]));
  }
}

// Global LED brightness scale (percent). The status colors are authored at
// full intensity; scaling every channel here tunes brightness in one place
// while keeping each color's hue (e.g. amber's 255/90/0 ratio) intact.
#define LED_BRIGHTNESS_PCT 33

void set_rgb_color(uint8_t red, uint8_t green, uint8_t blue) {
  red   = (uint16_t)red   * LED_BRIGHTNESS_PCT / 100;
  green = (uint16_t)green * LED_BRIGHTNESS_PCT / 100;
  blue  = (uint16_t)blue  * LED_BRIGHTNESS_PCT / 100;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, red);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, green);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, blue);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

static void read_voltages() {
  battery_voltage = 0;
  usb_voltage = 0;
  int battery_voltage_tmp;
  int usb_voltage_tmp;
  for (int i = 0; i < 3; i++) {
    adc_oneshot_read(adc1_handle, BATTERY_CHANNEL, &battery_voltage_tmp);
    adc_cali_raw_to_voltage(battery_cali_handle, battery_voltage_tmp, &battery_voltage_tmp);
    adc_oneshot_read(adc1_handle, USB_CHANNEL, &usb_voltage_tmp);
    adc_cali_raw_to_voltage(usb_cali_handle, usb_voltage_tmp, &usb_voltage_tmp);

    battery_voltage = battery_voltage_tmp > battery_voltage ? battery_voltage_tmp : battery_voltage;
    usb_voltage = usb_voltage_tmp > usb_voltage ? usb_voltage_tmp : usb_voltage;

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

static void init_voltage_measurements() {
  adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = ADC_UNIT_1,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));
  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN_DB_11,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BATTERY_CHANNEL, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, USB_CHANNEL, &config));
  adc_calibration_init(init_config.unit_id, BATTERY_CHANNEL, config.atten, &battery_cali_handle);
  adc_calibration_init(init_config.unit_id, USB_CHANNEL, config.atten, &usb_cali_handle);
  if (battery_cali_handle == NULL || usb_cali_handle == NULL) {
    ESP_LOGE(POWER_MANAGEMENT_TASK, "Failed to initialize calibration");
    return;
  }
}

void power_management(void* params) {
  init_voltage_measurements();
  ledc_init();

  voltage_update_timer = xTimerCreate(
      "voltage_update_timer",
      pdMS_TO_TICKS(UPDATE_INTERVAL),
      pdFALSE,
      0,
      voltage_update_timer_callback
  );
  xTimerStart(voltage_update_timer, 0);

  // notify for initial reading
  xTaskNotifyGive(xTaskGetCurrentTaskHandle());

  while (true) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .pin_bit_mask = (1ULL << USB_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1,
    };
    gpio_config(&io_conf);
    gpio_isr_handler_add(USB_PIN, gpio_interrupt_handler, NULL);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    vTaskDelay(200 / portTICK_PERIOD_MS);
    // while unplugging, multiple interrupts might be triggered
    ulTaskNotifyTake(pdTRUE, 0);
    read_voltages();

    bool usb_connected = usb_voltage > USB_VOLTAGE_THRESHOLD;

    if (usb_connected) {
      xEventGroupSetBits(event_group, USB_CONNECTED);
    } else {
      xEventGroupClearBits(event_group, USB_CONNECTED);
    }

    ESP_LOGI(POWER_MANAGEMENT_TASK, "USB %dmV, battery %dmV", usb_voltage, battery_voltage);
    xEventGroupSetBits(event_group, DISPLAY_NEEDS_UPDATE);
  }
}
