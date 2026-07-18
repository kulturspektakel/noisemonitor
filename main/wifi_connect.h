#pragma once
#include <stdint.h>
#include "esp_err.h"

typedef enum { DISCONNECTED, CONNECTING, CONNECTED } wifi_status_t;
extern wifi_status_t wifi_status;

void wifi_connect(void* params);

// Kick an immediate WiFi connect attempt if we're not already connected or
// mid-connect. Safe to call from any (non-ISR) task; a no-op if the WiFi task
// isn't running. Used to opportunistically connect when a BLE client attaches,
// instead of waiting for the retry timer.
void wifi_connect_trigger(void);

// Persist new WiFi credentials to NVS and trigger reconnect with them.
// Safe to call from any task. SSID must be 1..32 bytes, password 0..63 bytes
// (empty password = open network). Both are NUL-terminated C strings.
esp_err_t wifi_connect_set_credentials(const char* ssid, const char* password);
