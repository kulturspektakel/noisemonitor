#pragma once

#define NVS_DEVICE_CONFIG "device_config"
#define NVS_WIFI_SSID "wifi_ssid"
#define NVS_WIFI_PASSWORD "wifi_password"
#define NVS_SALT "salt"

#define NVS_NOISE_CAL "noise_cal"
#define NVS_NOISE_CAL_OFFSET "offset_db_x100"   // obsolete scalar offset (erased on boot)
#define NVS_NOISE_CAL_BANDS "band_cal_q5"       // blob: 31 int8 per-band offsets, 0.5 dB steps

#define TZ "CET-1CEST,M3.5.0,M10.5.0/3"
#define API_HOST "www.kulturspektakel.de"

#define TASK_PRIO_NORMAL 5
#define TASK_PRIO_HIGH 10

// On-disk log cadence: one aggregated record is written per this many seconds,
// so a power loss drops at most the current in-progress interval. MQTT/BLE stay
// at 1 Hz regardless. Signalled to the server via NoiseRecording.record_interval_seconds.
#define RECORD_INTERVAL_SECONDS 60

#define LOG_UPLOADER_TASK "log_uploader"
#define WIFI_CONNECT_TASK "wifi_connect"
#define POWER_MANAGEMENT_TASK "power_mgmt"

extern const char* LOG_DIR;

#define SALT_LENGTH 32
#define DEVICE_ID_LENGTH 16
extern char SALT[SALT_LENGTH + 1];
extern char DEVICE_ID[DEVICE_ID_LENGTH + 1];

void load_device_id(void* params);
void load_salt(void* params);
