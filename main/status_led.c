#include "status_led.h"
#include "event_group.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "power_management.h"

// Poll the event group at 2 Hz; pick a color reflecting overall device state.
// Colors are implementation-defined per spec §11; the mapping below distinguishes
// healthy / recording-but-offline / pre-time-sync.
void status_led(void* params) {
  bool blink_phase = false;
  while (true) {
    EventBits_t bits = xEventGroupGetBits(event_group);
    bool time_set   = bits & TIME_SET;
    bool wifi       = bits & WIFI_CONNECTED;

    if (!time_set) {
      // Blue blink — awaiting RTC/NTP.
      set_rgb_color(blink_phase ? 0 : 0, 0, blink_phase ? 0 : 255);
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
