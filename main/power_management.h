#pragma once

#include <stdbool.h>
#include <stdint.h>

extern int battery_voltage;
extern int usb_voltage;
bool battery_is_low();
int battery_percentage();
void power_management(void* params);
void set_rgb_color(uint8_t red, uint8_t green, uint8_t blue);
