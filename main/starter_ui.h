#pragma once

#include "lvgl.h"
#include "PCF85063.h"
#include "QMI8658.h"

void starter_ui_init(void);
void starter_ui_update(float battery_volts, const datetime_t *rtc, const IMUdata *accel, uint8_t backlight);
