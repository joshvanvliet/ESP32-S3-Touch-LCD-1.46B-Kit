#include "app_motion.h"

#include <math.h>
#include <stdint.h>

#include "app_face_math.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "sdkconfig.h"

#ifndef CONFIG_APP_MOTION_LOG_ENABLED
#define CONFIG_APP_MOTION_LOG_ENABLED 0
#endif

#ifndef CONFIG_APP_MOTION_LOG_INTERVAL_MS
#define CONFIG_APP_MOTION_LOG_INTERVAL_MS 500
#endif

#ifndef CONFIG_APP_MOTION_ROLL_GYRO_SIGN
#define CONFIG_APP_MOTION_ROLL_GYRO_SIGN 1
#endif

#define APP_MOTION_ACCEL_ALPHA 0.18f
#define APP_MOTION_NEUTRAL_GX 0.87f
#define APP_MOTION_NEUTRAL_GY 0.17f
#define APP_MOTION_ROLL_GRAVITY_MIN_STRENGTH 0.26f
#define APP_MOTION_ROLL_GYRO_DEADBAND_DPS 0.35f
#define APP_MOTION_ROLL_GRAVITY_BLEND_STILL 0.24f
#define APP_MOTION_ROLL_GRAVITY_BLEND_MOVING 0.12f
#define APP_MOTION_ROLL_GRAVITY_BLEND_FAST 0.34f
#define APP_MOTION_ROLL_FAST_ERROR 0.12f
#define APP_MOTION_ROLL_STILL_DPS 2.0f
#define APP_MOTION_ROLL_UNRELIABLE_DECAY 0.004f
#define APP_MOTION_LOOK_ACCEL_DEADBAND 0.0075f
#define APP_MOTION_LOOK_IMPULSE_DEADBAND 0.0045f
#define APP_MOTION_LOOK_GYRO_DEADBAND_DPS 3.75f
#define APP_MOTION_LOOK_GYRO_FULL_AXIS_DEG 150.0f
#define APP_MOTION_LOOK_ACCEL_SCALE_X 1.15f
#define APP_MOTION_LOOK_ACCEL_SCALE_Y 1.05f
#define APP_MOTION_LOOK_DECAY_STILL 0.18f
#define APP_MOTION_LOOK_DECAY_MOVING 0.09f
#define APP_MOTION_LOOK_LIMIT_X 0.36f
#define APP_MOTION_LOOK_LIMIT_Y 0.30f
#define APP_MOTION_FLAT_LOOK_SUPPRESS 0.22f
#define APP_MOTION_GYRO_BIAS_ALPHA 0.018f
#define APP_MOTION_GYRO_BIAS_MAX_DPS 12.0f
#define APP_MOTION_STILL_GRAVITY_DELTA 0.028f
#define APP_MOTION_POSE_MOVING_DPS 9.0f
#define APP_MOTION_POSE_ACCEL_DELTA 0.025f

#if CONFIG_APP_MOTION_LOG_ENABLED
static const char *TAG = "APP_MOTION";
#endif

static portMUX_TYPE s_motion_lock = portMUX_INITIALIZER_UNLOCKED;
static float s_gx;
static float s_gy;
static float s_gz = 1.0f;
static float s_axis_x;
static float s_axis_y;
static float s_axis_z;
static float s_gyro_bias_x;
static float s_gyro_bias_y;
static float s_gyro_bias_z;
static int64_t s_last_update_us;
static int64_t s_last_log_us;
static bool s_has_sample;

static float roll_sign(void)
{
    return CONFIG_APP_MOTION_ROLL_GYRO_SIGN < 0 ? -1.0f : 1.0f;
}

static float neutral_roll_unit(void)
{
    return roll_sign() * atan2f(APP_MOTION_NEUTRAL_GY, APP_MOTION_NEUTRAL_GX) / APP_FACE_PI;
}

static bool roll_from_gravity(float gx, float gy, float *roll_unit, float *plane_strength)
{
    float strength = sqrtf(gx * gx + gy * gy);
    if (plane_strength) {
        *plane_strength = strength;
    }
    if (strength < APP_MOTION_ROLL_GRAVITY_MIN_STRENGTH) {
        if (roll_unit) {
            *roll_unit = 0.0f;
        }
        return false;
    }
    if (roll_unit) {
        float raw = roll_sign() * atan2f(gy, gx) / APP_FACE_PI;
        *roll_unit = app_face_wrap_unit(raw - neutral_roll_unit());
    }
    return true;
}

static float with_deadband(float value, float deadband)
{
    if (fabsf(value) <= deadband) {
        return 0.0f;
    }
    return value > 0.0f ? value - deadband : value + deadband;
}

#if CONFIG_APP_MOTION_LOG_ENABLED
static const char *dominant_gravity_axis(float gx, float gy, float gz)
{
    float ax = fabsf(gx);
    float ay = fabsf(gy);
    float az = fabsf(gz);
    if (ax >= ay && ax >= az) {
        return gx >= 0.0f ? "+X" : "-X";
    }
    if (ay >= ax && ay >= az) {
        return gy >= 0.0f ? "+Y" : "-Y";
    }
    return gz >= 0.0f ? "+Z" : "-Z";
}
#endif

void app_motion_reset(void)
{
    portENTER_CRITICAL(&s_motion_lock);
    s_gx = 0.0f;
    s_gy = 0.0f;
    s_gz = 1.0f;
    s_axis_x = 0.0f;
    s_axis_y = 0.0f;
    s_axis_z = 0.0f;
    s_gyro_bias_x = 0.0f;
    s_gyro_bias_y = 0.0f;
    s_gyro_bias_z = 0.0f;
    s_last_update_us = 0;
    s_last_log_us = 0;
    s_has_sample = false;
    portEXIT_CRITICAL(&s_motion_lock);
}

void app_motion_update_from_imu(float accel_x_g,
                                float accel_y_g,
                                float accel_z_g,
                                float gyro_x_dps,
                                float gyro_y_dps,
                                float gyro_z_dps)
{
    float len = sqrtf(accel_x_g * accel_x_g + accel_y_g * accel_y_g + accel_z_g * accel_z_g);
    if (len < 0.08f) {
        return;
    }

    int64_t now_us = esp_timer_get_time();
    float gx = accel_x_g / len;
    float gy = accel_y_g / len;
    float gz = accel_z_g / len;
    float gyro_mag = sqrtf(gyro_x_dps * gyro_x_dps + gyro_y_dps * gyro_y_dps + gyro_z_dps * gyro_z_dps);

#if CONFIG_APP_MOTION_LOG_ENABLED
    bool should_log = false;
    float log_gx = 0.0f;
    float log_gy = 0.0f;
    float log_gz = 1.0f;
    float log_axis_x = 0.0f;
    float log_axis_y = 0.0f;
    float log_axis_z = 0.0f;
    float log_gravity_roll = 0.0f;
    float log_plane_strength = 0.0f;
    bool log_roll_reliable = false;
    const char *log_pose_mode = "stable";
    float log_motion_x = 0.0f;
    float log_motion_y = 0.0f;
    float log_dt_ms = 0.0f;
    float log_bias_x = 0.0f;
    float log_bias_y = 0.0f;
    float log_bias_z = 0.0f;
    float log_corrected_gyro_x = 0.0f;
    float log_corrected_gyro_y = 0.0f;
    float log_corrected_gyro_z = 0.0f;
#endif

    portENTER_CRITICAL(&s_motion_lock);
    float dt_s = 0.02f;
    if (s_last_update_us > 0) {
        dt_s = (float)(now_us - s_last_update_us) * 0.000001f;
        dt_s = app_face_clampf(dt_s, 0.001f, 0.10f);
    }
    s_last_update_us = now_us;

    if (!s_has_sample) {
        s_gx = gx;
        s_gy = gy;
        s_gz = gz;
        float initial_plane_strength = 0.0f;
        float initial_gravity_roll = 0.0f;
        if (roll_from_gravity(gx, gy, &initial_gravity_roll, &initial_plane_strength)) {
            s_axis_z = initial_gravity_roll;
        } else {
            s_axis_z = 0.0f;
        }
        s_axis_x = 0.0f;
        s_axis_y = 0.0f;
        s_has_sample = true;
    } else {
        float prev_gy = s_gy;
        float prev_gz = s_gz;
        float gravity_delta = sqrtf((gx - s_gx) * (gx - s_gx) +
                                    (gy - s_gy) * (gy - s_gy) +
                                    (gz - s_gz) * (gz - s_gz));
        if (gravity_delta < APP_MOTION_STILL_GRAVITY_DELTA && gyro_mag < APP_MOTION_GYRO_BIAS_MAX_DPS) {
            s_gyro_bias_x += (gyro_x_dps - s_gyro_bias_x) * APP_MOTION_GYRO_BIAS_ALPHA;
            s_gyro_bias_y += (gyro_y_dps - s_gyro_bias_y) * APP_MOTION_GYRO_BIAS_ALPHA;
            s_gyro_bias_z += (gyro_z_dps - s_gyro_bias_z) * APP_MOTION_GYRO_BIAS_ALPHA;
        }

        s_gx += (gx - s_gx) * APP_MOTION_ACCEL_ALPHA;
        s_gy += (gy - s_gy) * APP_MOTION_ACCEL_ALPHA;
        s_gz += (gz - s_gz) * APP_MOTION_ACCEL_ALPHA;
        float n = sqrtf(s_gx * s_gx + s_gy * s_gy + s_gz * s_gz);
        if (n > 0.0001f) {
            s_gx /= n;
            s_gy /= n;
            s_gz /= n;
        }

        float corrected_gyro_x = gyro_x_dps - s_gyro_bias_x;
        float corrected_gyro_y = gyro_y_dps - s_gyro_bias_y;
        float corrected_gyro_z = gyro_z_dps - s_gyro_bias_z;
        float corrected_gyro_mag = sqrtf(corrected_gyro_x * corrected_gyro_x +
                                         corrected_gyro_y * corrected_gyro_y +
                                         corrected_gyro_z * corrected_gyro_z);

        float dgy = s_gy - prev_gy;
        float dgz = s_gz - prev_gz;
        float accel_motion_x = -with_deadband(dgy, APP_MOTION_LOOK_ACCEL_DEADBAND) *
                               APP_MOTION_LOOK_ACCEL_SCALE_X;
        float accel_motion_y = -with_deadband(dgz, APP_MOTION_LOOK_ACCEL_DEADBAND) *
                               APP_MOTION_LOOK_ACCEL_SCALE_Y;
        float gyro_motion_x = with_deadband(corrected_gyro_y, APP_MOTION_LOOK_GYRO_DEADBAND_DPS) *
                              dt_s / APP_MOTION_LOOK_GYRO_FULL_AXIS_DEG;
        float gyro_motion_y = -with_deadband(corrected_gyro_x, APP_MOTION_LOOK_GYRO_DEADBAND_DPS) *
                              dt_s / APP_MOTION_LOOK_GYRO_FULL_AXIS_DEG;
        float motion_x = with_deadband(accel_motion_x + gyro_motion_x, APP_MOTION_LOOK_IMPULSE_DEADBAND);
        float motion_y = with_deadband(accel_motion_y + gyro_motion_y, APP_MOTION_LOOK_IMPULSE_DEADBAND);
        float plane_strength = sqrtf(s_gx * s_gx + s_gy * s_gy);
        if (plane_strength < APP_MOTION_ROLL_GRAVITY_MIN_STRENGTH) {
            motion_x *= APP_MOTION_FLAT_LOOK_SUPPRESS;
            motion_y *= APP_MOTION_FLAT_LOOK_SUPPRESS;
        }
        s_axis_x = app_face_clampf(s_axis_x + motion_x, -APP_MOTION_LOOK_LIMIT_X, APP_MOTION_LOOK_LIMIT_X);
        s_axis_y = app_face_clampf(s_axis_y + motion_y, -APP_MOTION_LOOK_LIMIT_Y, APP_MOTION_LOOK_LIMIT_Y);
        float look_decay = corrected_gyro_mag < APP_MOTION_ROLL_STILL_DPS
                               ? APP_MOTION_LOOK_DECAY_STILL
                               : APP_MOTION_LOOK_DECAY_MOVING;
        s_axis_x += (0.0f - s_axis_x) * look_decay;
        s_axis_y += (0.0f - s_axis_y) * look_decay;

        float gyro_roll_dps = fabsf(corrected_gyro_z) >= APP_MOTION_ROLL_GYRO_DEADBAND_DPS ? corrected_gyro_z : 0.0f;
        s_axis_z = app_face_wrap_unit(s_axis_z + roll_sign() * gyro_roll_dps * dt_s / 180.0f);

        float filtered_gravity_roll = 0.0f;
        float filtered_plane_strength = 0.0f;
        if (roll_from_gravity(s_gx, s_gy, &filtered_gravity_roll, &filtered_plane_strength)) {
            float gravity_blend = corrected_gyro_mag < APP_MOTION_ROLL_STILL_DPS
                                      ? APP_MOTION_ROLL_GRAVITY_BLEND_STILL
                                      : APP_MOTION_ROLL_GRAVITY_BLEND_MOVING;
            float roll_delta = app_face_shortest_unit_delta(s_axis_z, filtered_gravity_roll);
            if (fabsf(roll_delta) > APP_MOTION_ROLL_FAST_ERROR && gravity_blend < APP_MOTION_ROLL_GRAVITY_BLEND_FAST) {
                gravity_blend = APP_MOTION_ROLL_GRAVITY_BLEND_FAST;
            }
            s_axis_z = app_face_wrap_unit(s_axis_z + roll_delta * gravity_blend);
        } else if (corrected_gyro_mag < APP_MOTION_ROLL_STILL_DPS) {
            s_axis_z = app_face_wrap_unit(s_axis_z + app_face_shortest_unit_delta(s_axis_z, 0.0f) *
                                                       APP_MOTION_ROLL_UNRELIABLE_DECAY);
        }

#if CONFIG_APP_MOTION_LOG_ENABLED
        log_motion_x = motion_x;
        log_motion_y = motion_y;
#endif
    }

#if CONFIG_APP_MOTION_LOG_ENABLED
    float display_gravity_roll = 0.0f;
    float display_plane_strength = 0.0f;
    bool display_roll_reliable = roll_from_gravity(s_gx, s_gy, &display_gravity_roll, &display_plane_strength);
    float corrected_log_x = gyro_x_dps - s_gyro_bias_x;
    float corrected_log_y = gyro_y_dps - s_gyro_bias_y;
    float corrected_log_z = gyro_z_dps - s_gyro_bias_z;
    float corrected_log_mag = sqrtf(corrected_log_x * corrected_log_x +
                                    corrected_log_y * corrected_log_y +
                                    corrected_log_z * corrected_log_z);
    const char *pose_mode = "stable";
    if (!display_roll_reliable) {
        pose_mode = "flat";
    } else if (corrected_log_mag > APP_MOTION_POSE_MOVING_DPS ||
               fabsf(log_motion_x) + fabsf(log_motion_y) > APP_MOTION_POSE_ACCEL_DELTA) {
        pose_mode = "moving";
    }

    if (s_last_log_us == 0 ||
        (uint64_t)(now_us - s_last_log_us) >= (uint64_t)CONFIG_APP_MOTION_LOG_INTERVAL_MS * 1000ULL) {
        s_last_log_us = now_us;
        should_log = true;
        log_gx = s_gx;
        log_gy = s_gy;
        log_gz = s_gz;
        log_axis_x = s_axis_x;
        log_axis_y = s_axis_y;
        log_axis_z = s_axis_z;
        log_gravity_roll = display_gravity_roll;
        log_plane_strength = display_plane_strength;
        log_roll_reliable = display_roll_reliable;
        log_pose_mode = pose_mode;
        log_dt_ms = dt_s * 1000.0f;
        log_bias_x = s_gyro_bias_x;
        log_bias_y = s_gyro_bias_y;
        log_bias_z = s_gyro_bias_z;
        log_corrected_gyro_x = corrected_log_x;
        log_corrected_gyro_y = corrected_log_y;
        log_corrected_gyro_z = corrected_log_z;
    }
#endif
    portEXIT_CRITICAL(&s_motion_lock);

#if CONFIG_APP_MOTION_LOG_ENABLED
    if (should_log) {
        ESP_LOGI(TAG,
                 "IMU acc=(%.2f,%.2f,%.2f)g gravity=(%.2f,%.2f,%.2f) dom=%s plane=%.2f roll_ok=%d pose=%s gyro=(%.1f,%.1f,%.1f)dps bias=(%.1f,%.1f,%.1f)dps corr=(%.1f,%.1f,%.1f)dps look=(%.2f,%.2f) roll=%.2f impulse=(%.3f,%.3f) grav_roll=%.2f dt=%.1fms",
                 (double)accel_x_g,
                 (double)accel_y_g,
                 (double)accel_z_g,
                 (double)log_gx,
                 (double)log_gy,
                 (double)log_gz,
                 dominant_gravity_axis(log_gx, log_gy, log_gz),
                 (double)log_plane_strength,
                 log_roll_reliable ? 1 : 0,
                 log_pose_mode,
                 (double)gyro_x_dps,
                 (double)gyro_y_dps,
                 (double)gyro_z_dps,
                 (double)log_bias_x,
                 (double)log_bias_y,
                 (double)log_bias_z,
                 (double)log_corrected_gyro_x,
                 (double)log_corrected_gyro_y,
                 (double)log_corrected_gyro_z,
                 (double)log_axis_x,
                 (double)log_axis_y,
                 (double)log_axis_z,
                 (double)log_motion_x,
                 (double)log_motion_y,
                 (double)log_gravity_roll,
                 (double)log_dt_ms);
    }
#endif
}

void app_motion_update_from_accel(float accel_x_g, float accel_y_g, float accel_z_g)
{
    app_motion_update_from_imu(accel_x_g, accel_y_g, accel_z_g, 0.0f, 0.0f, 0.0f);
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
