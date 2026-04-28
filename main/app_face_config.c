#include "app_face_config.h"

#include <string.h>

#include "app_face_math.h"

#define RGB(hex) \
    { .r = (uint8_t)(((hex) >> 16) & 0xFF), .g = (uint8_t)(((hex) >> 8) & 0xFF), .b = (uint8_t)((hex) & 0xFF) }

const app_face_config_t APP_FACE_DEFAULT_CONFIG = {
    .profile_name = "esp32-s3-412",
    .geometry = {
        .logical_width = 240,
        .logical_height = 240,
        .render_width = 412,
        .render_height = 412,
        .fps = 60,
        .display_center_x = 0.5f,
        .display_center_y = 0.5f,
        .face_center_x = 0.5f,
        .face_center_y = 0.46f,
        .display_radius = 0.46f,
        .sphere_radius = 0.82f,
        .eye_width_scale = 1.95f,
        .eye_height_scale = 1.52f,
        .eye_separation_scale = 1.24f,
        .patch_scale = 0.74f,
        .gaze_center_shift_x = 0.16f,
        .gaze_center_shift_y = 0.08f,
        .max_horizontal_edge = 0.82f,
        .fit_horizontal_edge = 0.76f,
        .fit_vertical_half = 0.58f,
        .projected_eye_edge_margin = 0.025f,
        .projected_eye_visibility_floor = 0.18f,
        .calm_center_gap = 0.20f,
        .concave_center_gap = 0.16f,
    },
    .motion = {
        .transition_ms = 820,
        .initial_blink_delay_ms = 1200,
        .user_quiet_ms = 2600,
        .user_release_ms = 1100,
        .gaze_move_out_ms = 620,
        .gaze_hold_min_ms = 680,
        .gaze_hold_max_ms = 1080,
        .gaze_return_ms = 820,
        .gaze_gap_min_ms = 3400,
        .gaze_gap_max_ms = 7200,
        .gaze_max_total_x = 0.27f,
        .gaze_max_total_y = 0.095f,
        .drift_max_x = 0.12f,
        .drift_max_y = 0.045f,
        .tilt_vector_limit = 1.0f,
        .drift_speed_base = 0.26f,
        .drift_speed_energy = 0.10f,
        .blink_close = 0.42f,
        .blink_hold = 0.12f,
        .blink_open = 0.72f,
        .blink_minimum_open = 0.030f,
        .blink_randomness = 0.70f,
        .nod_frequency_base = 1.75f,
        .nod_frequency_energy = 0.90f,
        .energy_response_hz = 3.2f,
        .squint_response_hz = 2.8f,
        .click_pulse_ms = 620,
    },
    .energy = {
        .gaze_ease_low = 0.72f,
        .gaze_ease_high = 1.65f,
        .pulse_low = 0.28f,
        .pulse_high = 1.32f,
        .bounce_low = 0.12f,
        .bounce_high = 1.08f,
        .breathing_low = 0.003f,
        .breathing_high = 0.028f,
        .blink_interval_low_ms = 6200.0f,
        .blink_interval_high_ms = 2600.0f,
        .blink_duration_low_ms = 260.0f,
        .blink_duration_high_ms = 145.0f,
    },
};

const app_face_mode_style_t APP_FACE_MODE_STYLES[APP_FACE_MODE_COUNT] = {
    [APP_FACE_IDLE] = {
        .name = "idle",
        .ambient_gaze = true,
        .iris = RGB(0x6FFFEE),
        .pupil = RGB(0x168F99),
        .v = {.eye_w = 0.36f, .eye_h = 0.46f, .sep = 0.39f, .y = 0.01f, .open = 1.0f, .round_eye = 1.0f, .pupil_size = 1.0f, .sparkle = 0.62f, .cheek = 0.48f, .eye_tilt = -0.02f, .pulse = 0.18f, .bob = 0.75f, .shell_tint = 0.18f},
    },
    [APP_FACE_CURIOUS] = {
        .name = "curious",
        .ambient_gaze = true,
        .iris = RGB(0x73FFF1),
        .pupil = RGB(0x129AA5),
        .v = {.eye_w = 0.34f, .eye_h = 0.44f, .sep = 0.41f, .y = -0.035f, .open = 1.0f, .round_eye = 1.0f, .lid = -0.02f, .pupil_size = 0.82f, .sparkle = 0.76f, .cheek = 0.34f, .side_eye = 0.42f, .eye_tilt = 0.16f, .asymmetry = 0.9f, .bounce = 0.08f, .pulse = 0.38f, .bob = 1.05f, .shell = -0.02f, .shell_tint = 0.22f},
    },
    [APP_FACE_LISTENING] = {
        .name = "listening",
        .ambient_gaze = false,
        .iris = RGB(0x74FFF1),
        .pupil = RGB(0x119EAA),
        .v = {.eye_w = 0.35f, .eye_h = 0.47f, .sep = 0.42f, .y = -0.055f, .open = 1.0f, .round_eye = 1.0f, .lid = -0.10f, .pupil_size = 0.58f, .sparkle = 0.22f, .cheek = 0.12f, .nod = 0.38f, .bounce = 0.04f, .pulse = 0.34f, .bob = 0.46f, .shell = -0.03f, .shell_tint = 0.24f},
    },
    [APP_FACE_THINKING] = {
        .name = "thinking",
        .ambient_gaze = true,
        .iris = RGB(0x8AF7FF),
        .pupil = RGB(0x176D8D),
        .v = {.eye_w = 0.35f, .eye_h = 0.45f, .sep = 0.43f, .y = -0.03f, .open = 0.70f, .round_eye = 1.0f, .slant = 0.02f, .lid = 0.16f, .valley = 0.18f, .pupil_size = 0.48f, .sparkle = 0.35f, .cheek = 0.08f, .side_eye = -0.50f, .eye_tilt = -0.16f, .think_dots = 1.0f, .bounce = 0.04f, .pulse = 0.30f, .bob = 0.62f, .shell = 0.04f, .shell_tint = 0.12f},
    },
    [APP_FACE_SPEAKING] = {
        .name = "speaking",
        .ambient_gaze = true,
        .iris = RGB(0x67FFF1),
        .pupil = RGB(0x118B99),
        .v = {.eye_w = 0.36f, .eye_h = 0.46f, .sep = 0.41f, .open = 0.92f, .round_eye = 1.0f, .smile = 0.08f, .pupil_size = 0.78f, .sparkle = 0.28f, .cheek = 0.34f, .eye_tilt = 0.02f, .speaking_bars = 1.0f, .bounce = 0.72f, .pulse = 0.92f, .bob = 1.10f, .shell_tint = 0.20f},
    },
    [APP_FACE_HAPPY] = {
        .name = "happy",
        .ambient_gaze = true,
        .iris = RGB(0x71FFDC),
        .pupil = RGB(0x148F81),
        .v = {.eye_w = 0.52f, .eye_h = 0.34f, .sep = 0.45f, .y = 0.045f, .open = 0.50f, .smile = 1.0f, .pupil_size = 0.08f, .cheek = 1.0f, .eye_tilt = -0.03f, .pulse = 0.48f, .bob = 1.35f, .shell = -0.04f, .shell_tint = 0.24f},
    },
    [APP_FACE_SUSPICIOUS] = {
        .name = "suspicious",
        .ambient_gaze = false,
        .iris = RGB(0xA970FF),
        .pupil = RGB(0xFFF0A3),
        .v = {.eye_w = 0.43f, .eye_h = 0.36f, .sep = 0.46f, .y = -0.03f, .open = 0.42f, .round_eye = 0.24f, .slant = -0.22f, .smile = -0.12f, .lid = 0.38f, .pinch = -0.5f, .valley = 0.72f, .pupil_size = 0.28f, .glance = 1.0f, .eye_tilt = -0.06f, .pulse = 0.18f, .bob = 0.32f, .shell = 0.08f, .shell_tint = 0.04f},
    },
    [APP_FACE_SLEEPY] = {
        .name = "sleepy",
        .ambient_gaze = false,
        .iris = RGB(0xB38CFF),
        .pupil = RGB(0xFFF2A6),
        .v = {.eye_w = 0.43f, .eye_h = 0.24f, .sep = 0.44f, .y = 0.05f, .open = 0.28f, .slant = 0.10f, .smile = -0.04f, .lid = 0.48f, .valley = 0.26f, .pupil_size = 0.18f, .sparkle = 0.22f, .eye_tilt = 0.04f, .pulse = 0.10f, .bob = 0.18f, .shell = 0.10f, .shell_tint = 0.08f},
    },
    [APP_FACE_ANGRY] = {
        .name = "angry",
        .ambient_gaze = false,
        .iris = RGB(0xA869FF),
        .pupil = RGB(0xFFE250),
        .v = {.eye_w = 0.46f, .eye_h = 0.36f, .sep = 0.46f, .y = 0.01f, .open = 0.80f, .slant = 0.08f, .smile = -0.20f, .lid = 0.46f, .pinch = 1.34f, .concave_top = 1.0f, .pupil_size = 0.30f, .eye_tilt = 0.03f, .pulse = 0.18f, .bob = 0.18f, .shell = 0.09f, .shell_tint = 0.02f},
    },
    [APP_FACE_SURPRISED] = {
        .name = "surprised",
        .ambient_gaze = true,
        .iris = RGB(0x80FFF4),
        .pupil = RGB(0x1595A0),
        .v = {.eye_w = 0.43f, .eye_h = 0.92f, .sep = 0.43f, .y = -0.055f, .open = 1.0f, .lid = -0.18f, .pupil_size = 0.42f, .sparkle = 0.90f, .cheek = 0.04f, .bounce = 0.04f, .pulse = 0.78f, .bob = 1.18f, .shell = -0.05f, .shell_tint = 0.24f},
    },
    [APP_FACE_ERROR] = {
        .name = "error",
        .ambient_gaze = false,
        .iris = RGB(0xFF6B63),
        .pupil = RGB(0xFFE9B4),
        .v = {.eye_w = 0.43f, .eye_h = 0.34f, .sep = 0.46f, .y = -0.02f, .open = 0.50f, .slant = -0.64f, .smile = -0.22f, .lid = 0.24f, .pinch = 0.46f, .concave_top = 0.34f, .valley = 0.42f, .pupil_size = 0.24f, .sparkle = 0.28f, .eye_tilt = -0.08f, .pulse = 0.54f, .bob = 0.28f, .shell = -0.10f, .shell_tint = 0.02f},
    },
};

const char *app_face_mode_name(app_face_mode_t mode)
{
    if (mode < 0 || mode >= APP_FACE_MODE_COUNT) {
        return "unknown";
    }
    return APP_FACE_MODE_STYLES[mode].name;
}

app_face_mode_t app_face_mode_from_name(const char *name, app_face_mode_t fallback)
{
    if (!name) {
        return fallback;
    }
    for (int i = 0; i < APP_FACE_MODE_COUNT; i++) {
        if (strcmp(name, APP_FACE_MODE_STYLES[i].name) == 0) {
            return (app_face_mode_t)i;
        }
    }
    return fallback;
}

app_face_rgb_t app_face_rgb_lerp(app_face_rgb_t from, app_face_rgb_t to, float t)
{
    t = app_face_clampf(t, 0.0f, 1.0f);
    return (app_face_rgb_t){
        .r = (uint8_t)app_face_lerpf((float)from.r, (float)to.r, t),
        .g = (uint8_t)app_face_lerpf((float)from.g, (float)to.g, t),
        .b = (uint8_t)app_face_lerpf((float)from.b, (float)to.b, t),
    };
}

app_face_mood_numbers_t app_face_mood_lerp(const app_face_mood_numbers_t *from,
                                           const app_face_mood_numbers_t *to,
                                           float t)
{
    app_face_mood_numbers_t out;
    t = app_face_smoothf(t);
#define LERP_FIELD(field) out.field = app_face_lerpf(from->field, to->field, t)
    LERP_FIELD(eye_w);
    LERP_FIELD(eye_h);
    LERP_FIELD(sep);
    LERP_FIELD(y);
    LERP_FIELD(open);
    LERP_FIELD(round_eye);
    LERP_FIELD(slant);
    LERP_FIELD(smile);
    LERP_FIELD(lid);
    LERP_FIELD(pinch);
    LERP_FIELD(concave_top);
    LERP_FIELD(valley);
    LERP_FIELD(pupil_size);
    LERP_FIELD(sparkle);
    LERP_FIELD(cheek);
    LERP_FIELD(side_eye);
    LERP_FIELD(glance);
    LERP_FIELD(eye_tilt);
    LERP_FIELD(asymmetry);
    LERP_FIELD(think_dots);
    LERP_FIELD(nod);
    LERP_FIELD(speaking_bars);
    LERP_FIELD(bounce);
    LERP_FIELD(pulse);
    LERP_FIELD(bob);
    LERP_FIELD(shell);
    LERP_FIELD(shell_tint);
#undef LERP_FIELD
    return out;
}
