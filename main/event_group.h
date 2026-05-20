#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define WIFI_CONNECTED       BIT2
#define TIME_SET             BIT3
#define DISPLAY_NEEDS_UPDATE BIT4
#define DEVICE_ID_LOADED     BIT5
#define SALT_LOADED          BIT6
#define USB_CONNECTED        BIT7
#define CALIBRATED           BIT8
#define BLE_HOST_READY       BIT9

static const EventBits_t STARTUP_BITS = TIME_SET | DEVICE_ID_LOADED | SALT_LOADED;

extern EventGroupHandle_t event_group;
