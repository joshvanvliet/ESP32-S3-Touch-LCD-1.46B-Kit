#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    APP_FACE_IDLE = 0,
    APP_FACE_CURIOUS,
    APP_FACE_LISTENING,
    APP_FACE_THINKING,
    APP_FACE_SPEAKING,
    APP_FACE_HAPPY,
    APP_FACE_SUSPICIOUS,
    APP_FACE_SLEEPY,
    APP_FACE_ANGRY,
    APP_FACE_SURPRISED,
    APP_FACE_ERROR,
    APP_FACE_MODE_COUNT,
} app_face_mode_t;

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} app_face_rgb_t;

typedef struct {
    float eye_w;
    float eye_h;
    float sep;
    float y;
    float open;
    float round_eye;
    float slant;
    float smile;
    float lid;
    float pinch;
    float concave_top;
    float valley;
    float pupil_size;
    float sparkle;
    float cheek;
    float side_eye;
    float glance;
    float eye_tilt;
    float asymmetry;
    float think_dots;
    float nod;
    float speaking_bars;
    float bounce;
    float pulse;
    float bob;
    float shell;
    float shell_tint;
} app_face_mood_numbers_t;

typedef struct {
    const char *name;
    bool ambient_gaze;
    app_face_rgb_t iris;
    app_face_rgb_t pupil;
    app_face_mood_numbers_t v;
} app_face_mode_style_t;

typedef struct {
    uint16_t logical_width;
    uint16_t logical_height;
    uint16_t render_width;
    uint16_t render_height;
    uint16_t fps;
    float display_center_x;
    float display_center_y;
    float face_center_x;
    float face_center_y;
    float display_radius;
    float sphere_radius;
    float eye_width_scale;
    float eye_height_scale;
    float eye_separation_scale;
    float patch_scale;
    float gaze_center_shift_x;
    float gaze_center_shift_y;
    float max_horizontal_edge;
    float fit_horizontal_edge;
    float fit_vertical_half;
    float projected_eye_edge_margin;
    float projected_eye_visibility_floor;
    float calm_center_gap;
    float concave_center_gap;
} app_face_geometry_config_t;

typedef struct {
    uint32_t transition_ms;
    uint32_t initial_blink_delay_ms;
    uint32_t user_quiet_ms;
    uint32_t user_release_ms;
    uint32_t gaze_move_out_ms;
    uint32_t gaze_hold_min_ms;
    uint32_t gaze_hold_max_ms;
    uint32_t gaze_return_ms;
    uint32_t gaze_gap_min_ms;
    uint32_t gaze_gap_max_ms;
    float gaze_max_total_x;
    float gaze_max_total_y;
    float drift_max_x;
    float drift_max_y;
    float tilt_vector_limit;
    float drift_speed_base;
    float drift_speed_energy;
    float blink_close;
    float blink_hold;
    float blink_open;
    float blink_minimum_open;
    float blink_randomness;
    float nod_frequency_base;
    float nod_frequency_energy;
    uint32_t click_pulse_ms;
} app_face_motion_config_t;

typedef struct {
    float gaze_ease_low;
    float gaze_ease_high;
    float pulse_low;
    float pulse_high;
    float bounce_low;
    float bounce_high;
    float breathing_low;
    float breathing_high;
    float blink_interval_low_ms;
    float blink_interval_high_ms;
    float blink_duration_low_ms;
    float blink_duration_high_ms;
} app_face_energy_config_t;

typedef struct {
    const char *profile_name;
    app_face_geometry_config_t geometry;
    app_face_motion_config_t motion;
    app_face_energy_config_t energy;
} app_face_config_t;

extern const app_face_config_t APP_FACE_DEFAULT_CONFIG;
extern const app_face_mode_style_t APP_FACE_MODE_STYLES[APP_FACE_MODE_COUNT];

const char *app_face_mode_name(app_face_mode_t mode);
app_face_mode_t app_face_mode_from_name(const char *name, app_face_mode_t fallback);
app_face_rgb_t app_face_rgb_lerp(app_face_rgb_t from, app_face_rgb_t to, float t);
app_face_mood_numbers_t app_face_mood_lerp(const app_face_mood_numbers_t *from,
                                           const app_face_mood_numbers_t *to,
                                           float t);
