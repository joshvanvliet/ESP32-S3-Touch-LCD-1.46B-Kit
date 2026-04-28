#pragma once

#include <stdbool.h>

void app_motion_reset(void);
void app_motion_update_from_imu(float accel_x_g,
                                float accel_y_g,
                                float accel_z_g,
                                float gyro_x_dps,
                                float gyro_y_dps,
                                float gyro_z_dps);
void app_motion_update_from_accel(float accel_x_g, float accel_y_g, float accel_z_g);
bool app_motion_get_axes(float *axis_x, float *axis_y, float *axis_z);
