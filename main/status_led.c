#include "status_led.h"
#include "esp_log.h"
#include "event_group.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "power_management.h"

static const char* TAG = "status_led";

// Poll the event group at 2 Hz; pick a color reflecting overall device state.
// Colors are implementation-defined per spec §11; the mapping below distinguishes
// healthy / recording-but-offline / pre-time-sync.
void status_led(void* params) {
  bool blink_phase = false;
  const char* last_state = NULL;
  while (true) {
    EventBits_t bits = xEventGroupGetBits(event_group);
    bool time_set   = bits & TIME_SET;
    bool wifi       = bits & WIFI_CONNECTED;

    const char* state;
    if (!time_set) {
      // Blue blink — awaiting RTC/NTP.
      set_rgb_color(0, 0, blink_phase ? 0 : 255);
      state = "BLUE blink (awaiting time)";
    } else if (!wifi) {
      // Steady amber — recording locally; no WiFi for upload/MQTT/etc.
      set_rgb_color(255, 90, 0);
      state = "AMBER steady (no WiFi)";
    } else {
      // Steady green — time set, WiFi up.
      set_rgb_color(0, 255, 0);
      state = "GREEN steady (healthy)";
    }

    // Log only on change: makes "what colour was the LED at time T" answerable
    // from the serial log, which is otherwise guesswork when reading a colour
    // by eye (and is how a red/blue pin swap went unnoticed for so long).
    if (state != last_state) {
      ESP_LOGI(TAG, "%s", state);
      last_state = state;
    }

    blink_phase = !blink_phase;
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
