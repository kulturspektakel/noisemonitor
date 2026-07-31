#include "status_led.h"
#include "esp_log.h"
#include "event_group.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "power_management.h"

static const char* TAG = "status_led";

// Boot self-test: drive each channel alone, at full brightness, in a known
// order, logging each step. Watch the LED and compare against the log — if the
// observed order isn't red → green → blue, the LED_*_PIN constants in
// power_management.c don't match this board's wiring.
//
// Worth keeping as a field-diagnosis tool: a permuted RGB LED is invisible in
// normal operation whenever the healthy state is green, because green is the
// middle channel and survives a red/blue swap unchanged.
static void led_self_test(void) {
  // ledc_init() runs inside the power_management task (after its ADC warm-up),
  // so LEDC channels may not be configured yet when this task first runs.
  // Wait before the first color, or it gets swallowed.
  vTaskDelay(pdMS_TO_TICKS(1000));

  static const struct {
    const char* name;
    uint8_t r, g, b;
  } steps[] = {
      {"RED",   255, 0,   0  },
      {"GREEN", 0,   255, 0  },
      {"BLUE",  0,   0,   255},
  };

  for (int i = 0; i < 3; i++) {
    ESP_LOGI(TAG, "self-test %d/3: driving %s", i + 1, steps[i].name);
    set_rgb_color(steps[i].r, steps[i].g, steps[i].b);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  ESP_LOGI(TAG, "self-test done; resuming status colors");
}

// Poll the event group at 2 Hz; pick a color reflecting overall device state.
// Colors are implementation-defined per spec §11; the mapping below distinguishes
// healthy / recording-but-offline / pre-time-sync.
void status_led(void* params) {
  led_self_test();

  bool blink_phase = false;
  while (true) {
    EventBits_t bits = xEventGroupGetBits(event_group);
    bool time_set   = bits & TIME_SET;
    bool wifi       = bits & WIFI_CONNECTED;

    if (!time_set) {
      // Blue blink — awaiting RTC/NTP.
      set_rgb_color(0, 0, blink_phase ? 0 : 255);
    } else if (!wifi) {
      // Steady amber — recording locally; no WiFi for upload/MQTT/etc.
      set_rgb_color(255, 90, 0);
    } else {
      // Steady green — time set, WiFi up.
      set_rgb_color(0, 255, 0);
    }
    blink_phase = !blink_phase;
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
