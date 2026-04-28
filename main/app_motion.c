#include "app_motion.h"

#include <math.h>

#include "app_face_math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static portMUX_TYPE s_motion_lock = portMUX_INITIALIZER_UNLOCKED;
static float s_gx;
static float s_gy;
static float s_gz = 1.0f;
static float s_axis_x;
static float s_axis_y;
static float s_axis_z;
static bool s_has_sample;

void app_motion_reset(void)
{
    portENTER_CRITICAL(&s_motion_lock);
    s_gx = 0.0f;
    s_gy = 0.0f;
    s_gz = 1.0f;
    s_axis_x = 0.0f;
    s_axis_y = 0.0f;
    s_axis_z = 0.0f;
    s_has_sample = false;
    portEXIT_CRITICAL(&s_motion_lock);
}

void app_motion_update_from_accel(float accel_x_g, float accel_y_g, float accel_z_g)
{
    float len = sqrtf(accel_x_g * accel_x_g + accel_y_g * accel_y_g + accel_z_g * accel_z_g);
    if (len < 0.08f) {
        return;
    }

    float gx = accel_x_g / len;
    float gy = accel_y_g / len;
    float gz = accel_z_g / len;
    const float alpha = 0.18f;

    portENTER_CRITICAL(&s_motion_lock);
    if (!s_has_sample) {
        s_gx = gx;
        s_gy = gy;
        s_gz = gz;
        s_has_sample = true;
    } else {
        s_gx += (gx - s_gx) * alpha;
        s_gy += (gy - s_gy) * alpha;
        s_gz += (gz - s_gz) * alpha;
        float n = sqrtf(s_gx * s_gx + s_gy * s_gy + s_gz * s_gz);
        if (n > 0.0001f) {
            s_gx /= n;
            s_gy /= n;
            s_gz /= n;
        }
    }

    float target_x = app_face_clampf(s_gx / 0.72f, -1.0f, 1.0f);
    float target_y = app_face_clampf(-s_gy / 0.72f, -1.0f, 1.0f);
    float roll_strength = sqrtf(s_gx * s_gx + s_gy * s_gy);
    float target_z = s_axis_z;
    if (roll_strength > 0.18f) {
        target_z = app_face_wrap_unit(atan2f(s_gx, -s_gy) / APP_FACE_PI);
    } else {
        target_z = 0.0f;
    }

    s_axis_x += (target_x - s_axis_x) * 0.24f;
    s_axis_y += (target_y - s_axis_y) * 0.24f;
    s_axis_z = app_face_wrap_unit(s_axis_z + app_face_shortest_unit_delta(s_axis_z, target_z) * 0.16f);
    portEXIT_CRITICAL(&s_motion_lock);
}

bool app_motion_get_axes(float *axis_x, float *axis_y, float *axis_z)
{
    bool has_sample;
    portENTER_CRITICAL(&s_motion_lock);
    has_sample = s_has_sample;
    if (axis_x) {
        *axis_x = s_axis_x;
    }
    if (axis_y) {
        *axis_y = s_axis_y;
    }
    if (axis_z) {
        *axis_z = s_axis_z;
    }
    portEXIT_CRITICAL(&s_motion_lock);
    return has_sample;
}

