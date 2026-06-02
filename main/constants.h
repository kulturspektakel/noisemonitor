#pragma once

#define NVS_DEVICE_CONFIG "device_config"
#define NVS_WIFI_SSID "wifi_ssid"
#define NVS_WIFI_PASSWORD "wifi_password"
#define NVS_SALT "salt"

#define NVS_NOISE_CAL "noise_cal"
#define NVS_NOISE_CAL_OFFSET "offset_db_x100"

#define TZ "CET-1CEST,M3.5.0,M10.5.0/3"
#define API_HOST "www.kulturspektakel.de"

#define TASK_PRIO_NORMAL 5
#define TASK_PRIO_HIGH 10

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
