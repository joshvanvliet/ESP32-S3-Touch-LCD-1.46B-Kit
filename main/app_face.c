#include "app_face.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "Display_SPD2010.h"
#include "app_face_canvas.h"
#include "app_lcd.h"
#include "app_face_math.h"
#include "app_motion.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#ifndef CONFIG_APP_FACE_FPS
#define CONFIG_APP_FACE_FPS 60
#endif

#ifndef CONFIG_APP_FACE_PROFILE
#define CONFIG_APP_FACE_PROFILE 0
#endif

#ifndef CONFIG_APP_FACE_TX_BUFFER_BYTES
#define CONFIG_APP_FACE_TX_BUFFER_BYTES 8192
#endif

#if !CONFIG_APP_FACE_ENABLED
esp_err_t app_face_init(lv_obj_t *parent)
{
    (void)parent;
    return ESP_OK;
}
lv_obj_t *app_face_get_obj(void) { return NULL; }
void app_face_set_mode(app_face_mode_t mode) { (void)mode; }
void app_face_pulse_mode(app_face_mode_t mode, uint32_t duration_ms) { (void)mode; (void)duration_ms; }
void app_face_set_energy(float energy_0_to_1) { (void)energy_0_to_1; }
void app_face_set_axes(float axis_x, float axis_y, float axis_z) { (void)axis_x; (void)axis_y; (void)axis_z; }
void app_face_force_blink(void) {}
void app_face_tap(void) {}
void app_face_tick(uint32_t now_ms) { (void)now_ms; }
void app_face_set_protected_areas(const lv_area_t *areas, size_t count) { (void)areas; (void)count; }
void app_face_request_full_refresh(void) {}
bool app_face_get_stats(app_face_stats_t *out_stats) { (void)out_stats; return false; }
#else

static const char *TAG = "APP_FACE";

#define APP_FACE_TX_BUFFER_BYTES CONFIG_APP_FACE_TX_BUFFER_BYTES
#define APP_FACE_TX_BUFFER_COUNT 1
#define APP_FACE_MAX_DIRTY_RECTS 10
#define APP_FACE_MAX_BLIT_FRAGMENTS 16
#define APP_FACE_MAX_PROTECTED_RECTS 4
#define APP_FACE_DIRTY_MERGE_GAP 6

typedef struct {
    lv_area_t rects[APP_FACE_MAX_DIRTY_RECTS];
    size_t count;
} app_face_dirty_list_t;

typedef enum {
    APP_FACE_GAZE_IDLE,
    APP_FACE_GAZE_MOVING_OUT,
    APP_FACE_GAZE_HOLDING,
    APP_FACE_GAZE_RETURNING,
} app_face_gaze_phase_t;

typedef struct {
    float raw;
    float lively;
    float gaze_ease;
    float pulse;
    float bounce;
    float breathing;
    float blink_interval_ms;
    float blink_duration_ms;
} app_face_energy_profile_t;

typedef struct {
    float scale;
    float cx;
    float cy;
    float radius;
    float face_motion_y;
    float voice_beat;
    float eye_w;
    float eye_h;
    float local_h;
    float sep;
    float base_y;
    float patch_w;
    float patch_h;
    app_face_surface_t left;
    app_face_surface_t right;
    float look_x;
    float look_y;
} app_face_draw_layout_t;

typedef struct {
    float x;
    float y;
    float start_x;
    float start_y;
    float target_x;
    float target_y;
    app_face_gaze_phase_t phase;
    uint32_t phase_start_ms;
    uint32_t phase_end_ms;
    uint32_t next_at_ms;
} app_face_gaze_t;

typedef struct {
    bool forced;
    float value;
    uint32_t start_ms;
    uint32_t next_ms;
} app_face_blink_t;

typedef struct {
    lv_obj_t *canvas_obj;
    lv_color_t *pixels;
    lv_color_t *base_pixels;
    lv_color_t *tx_buffers[APP_FACE_TX_BUFFER_COUNT];
    app_face_canvas_t canvas;
    app_face_canvas_t base_canvas;
    app_face_mode_t previous_mode;
    app_face_mode_t target_mode;
    app_face_mode_t base_mode;
    app_face_mode_t pulse_return_mode;
    bool pulse_active;
    bool base_dirty;
    bool has_prev_dynamic_list;
    bool full_refresh_requested;
    app_face_dirty_list_t prev_dynamic_list;
    lv_area_t protected_areas[APP_FACE_MAX_PROTECTED_RECTS];
    size_t protected_area_count;
    uint8_t tx_index;
    uint32_t pulse_until_ms;
    uint32_t transition_start_ms;
    uint32_t last_frame_ms;
    uint32_t last_update_ms;
    uint32_t stats_window_start_ms;
    uint32_t stats_rendered_frames;
    uint32_t stats_blit_frames;
    uint32_t stats_skipped_frames;
    uint32_t stats_dirty_pixels;
    uint32_t stats_packed_bytes;
    app_face_stats_t stats;
    float time_s;
    float energy;
    float target_energy;
    float axis_x;
    float axis_y;
    float axis_z;
    float click_pulse;
    float micro_squint;
    app_face_gaze_t gaze;
    app_face_blink_t blink;
    uint32_t rng;
    bool initialized;
} app_face_runtime_t;

static app_face_runtime_t s_face;

static uint32_t elapsed_ms(uint32_t now, uint32_t then)
{
    return (uint32_t)(now - then);
}

static uint32_t rng_next(void)
{
    s_face.rng = s_face.rng * 1664525u + 1013904223u;
    return s_face.rng;
}

static float rng_float(void)
{
    return (float)(rng_next() & 0x00FFFFFFu) / (float)0x01000000u;
}

static uint32_t rng_range_ms(uint32_t min_ms, uint32_t max_ms)
{
    if (max_ms <= min_ms) {
        return min_ms;
    }
    return min_ms + (uint32_t)(rng_float() * (float)(max_ms - min_ms));
}

static uint8_t alpha_from_float(float alpha)
{
    return (uint8_t)(app_face_clampf(alpha, 0.0f, 1.0f) * 255.0f);
}

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static void area_reset(lv_area_t *area, int width, int height)
{
    area->x1 = width;
    area->y1 = height;
    area->x2 = -1;
    area->y2 = -1;
}

static bool area_is_empty(const lv_area_t *area)
{
    return area->x1 > area->x2 || area->y1 > area->y2;
}

static void area_include_rect(lv_area_t *area, float x1, float y1, float x2, float y2)
{
    int ix1 = (int)floorf(fminf(x1, x2));
    int iy1 = (int)floorf(fminf(y1, y2));
    int ix2 = (int)ceilf(fmaxf(x1, x2));
    int iy2 = (int)ceilf(fmaxf(y1, y2));
    if (area_is_empty(area)) {
        area->x1 = ix1;
        area->y1 = iy1;
        area->x2 = ix2;
        area->y2 = iy2;
        return;
    }
    area->x1 = area->x1 < ix1 ? area->x1 : ix1;
    area->y1 = area->y1 < iy1 ? area->y1 : iy1;
    area->x2 = area->x2 > ix2 ? area->x2 : ix2;
    area->y2 = area->y2 > iy2 ? area->y2 : iy2;
}

static void area_include_area(lv_area_t *area, const lv_area_t *other)
{
    if (area_is_empty(other)) {
        return;
    }
    area_include_rect(area, other->x1, other->y1, other->x2, other->y2);
}

static void area_inflate_clip(lv_area_t *area, int margin, int width, int height)
{
    if (area_is_empty(area)) {
        area->x1 = 0;
        area->y1 = 0;
        area->x2 = width - 1;
        area->y2 = height - 1;
        return;
    }
    area->x1 = clamp_int(area->x1 - margin, 0, width - 1);
    area->y1 = clamp_int(area->y1 - margin, 0, height - 1);
    area->x2 = clamp_int(area->x2 + margin, 0, width - 1);
    area->y2 = clamp_int(area->y2 + margin, 0, height - 1);
    area->x1 = (area->x1 >> 2) << 2;
    area->x2 = clamp_int(((area->x2 >> 2) << 2) + 3, 0, width - 1);
}

static bool area_clip_to_bounds(lv_area_t *area, int width, int height)
{
    if (area_is_empty(area)) {
        return false;
    }
    area->x1 = clamp_int(area->x1, 0, width - 1);
    area->y1 = clamp_int(area->y1, 0, height - 1);
    area->x2 = clamp_int(area->x2, 0, width - 1);
    area->y2 = clamp_int(area->y2, 0, height - 1);
    area->x1 = (area->x1 >> 2) << 2;
    area->x2 = clamp_int(((area->x2 >> 2) << 2) + 3, 0, width - 1);
    return !area_is_empty(area);
}

static int area_width(const lv_area_t *area)
{
    return area_is_empty(area) ? 0 : (int)(area->x2 - area->x1 + 1);
}

static int area_height(const lv_area_t *area)
{
    return area_is_empty(area) ? 0 : (int)(area->y2 - area->y1 + 1);
}

static uint32_t area_pixel_count(const lv_area_t *area)
{
    return (uint32_t)area_width(area) * (uint32_t)area_height(area);
}

static bool area_intersects(const lv_area_t *a, const lv_area_t *b)
{
    return !area_is_empty(a) && !area_is_empty(b) &&
           a->x1 <= b->x2 && a->x2 >= b->x1 && a->y1 <= b->y2 && a->y2 >= b->y1;
}

static bool area_near_or_overlaps(const lv_area_t *a, const lv_area_t *b, int gap)
{
    if (area_is_empty(a) || area_is_empty(b)) {
        return false;
    }
    return a->x1 <= b->x2 + gap && a->x2 + gap >= b->x1 &&
           a->y1 <= b->y2 + gap && a->y2 + gap >= b->y1;
}

static void dirty_list_reset(app_face_dirty_list_t *list)
{
    list->count = 0;
}

static void dirty_list_add_area(app_face_dirty_list_t *list, lv_area_t area, int width, int height)
{
    if (!area_clip_to_bounds(&area, width, height)) {
        return;
    }

    for (size_t i = 0; i < list->count; i++) {
        if (area_near_or_overlaps(&list->rects[i], &area, APP_FACE_DIRTY_MERGE_GAP)) {
            area_include_area(&list->rects[i], &area);
            area_clip_to_bounds(&list->rects[i], width, height);
            return;
        }
    }

    if (list->count < APP_FACE_MAX_DIRTY_RECTS) {
        list->rects[list->count++] = area;
        return;
    }

    size_t smallest = 0;
    uint32_t smallest_pixels = UINT32_MAX;
    for (size_t i = 0; i < list->count; i++) {
        uint32_t pixels = area_pixel_count(&list->rects[i]);
        if (pixels < smallest_pixels) {
            smallest_pixels = pixels;
            smallest = i;
        }
    }
    area_include_area(&list->rects[smallest], &area);
    area_clip_to_bounds(&list->rects[smallest], width, height);
}

static void dirty_list_add_list(app_face_dirty_list_t *dst,
                                const app_face_dirty_list_t *src,
                                int width,
                                int height)
{
    for (size_t i = 0; i < src->count; i++) {
        dirty_list_add_area(dst, src->rects[i], width, height);
    }
}

static void schedule_next_gaze(uint32_t now_ms)
{
    const app_face_motion_config_t *motion = &APP_FACE_DEFAULT_CONFIG.motion;
    s_face.gaze.next_at_ms = now_ms + rng_range_ms(motion->gaze_gap_min_ms, motion->gaze_gap_max_ms);
}

static void reset_gaze(uint32_t now_ms)
{
    s_face.gaze.x = 0.0f;
    s_face.gaze.y = 0.0f;
    s_face.gaze.start_x = 0.0f;
    s_face.gaze.start_y = 0.0f;
    s_face.gaze.target_x = 0.0f;
    s_face.gaze.target_y = 0.0f;
    s_face.gaze.phase = APP_FACE_GAZE_IDLE;
    s_face.gaze.phase_start_ms = now_ms;
    s_face.gaze.phase_end_ms = now_ms;
}

static void begin_gaze_move(uint32_t now_ms)
{
    static const app_face_vec2_t targets[] = {
        {.x = -0.22f, .y = -0.045f},
        {.x = 0.21f, .y = -0.035f},
        {.x = -0.18f, .y = 0.06f},
        {.x = 0.17f, .y = 0.05f},
    };
    const app_face_motion_config_t *motion = &APP_FACE_DEFAULT_CONFIG.motion;
    app_face_vec2_t target = targets[rng_next() % (sizeof(targets) / sizeof(targets[0]))];
    s_face.gaze.start_x = s_face.gaze.x;
    s_face.gaze.start_y = s_face.gaze.y;
    s_face.gaze.target_x = target.x;
    s_face.gaze.target_y = target.y;
    s_face.gaze.phase = APP_FACE_GAZE_MOVING_OUT;
    s_face.gaze.phase_start_ms = now_ms;
    s_face.gaze.phase_end_ms = now_ms + motion->gaze_move_out_ms;
}

static void begin_gaze_hold(uint32_t now_ms)
{
    const app_face_motion_config_t *motion = &APP_FACE_DEFAULT_CONFIG.motion;
    s_face.gaze.x = s_face.gaze.target_x;
    s_face.gaze.y = s_face.gaze.target_y;
    s_face.gaze.phase = APP_FACE_GAZE_HOLDING;
    s_face.gaze.phase_start_ms = now_ms;
    s_face.gaze.phase_end_ms = now_ms + rng_range_ms(motion->gaze_hold_min_ms, motion->gaze_hold_max_ms);
}

static void begin_gaze_return(uint32_t now_ms)
{
    const app_face_motion_config_t *motion = &APP_FACE_DEFAULT_CONFIG.motion;
    s_face.gaze.start_x = s_face.gaze.x;
    s_face.gaze.start_y = s_face.gaze.y;
    s_face.gaze.target_x = 0.0f;
    s_face.gaze.target_y = 0.0f;
    s_face.gaze.phase = APP_FACE_GAZE_RETURNING;
    s_face.gaze.phase_start_ms = now_ms;
    s_face.gaze.phase_end_ms = now_ms + motion->gaze_return_ms;
}

static void sample_gaze_motion(uint32_t now_ms)
{
    uint32_t span = s_face.gaze.phase_end_ms - s_face.gaze.phase_start_ms;
    if (span == 0) {
        span = 1;
    }
    float t = app_face_min_jerk((float)elapsed_ms(now_ms, s_face.gaze.phase_start_ms) / (float)span);
    s_face.gaze.x = app_face_lerpf(s_face.gaze.start_x, s_face.gaze.target_x, t);
    s_face.gaze.y = app_face_lerpf(s_face.gaze.start_y, s_face.gaze.target_y, t);
}

static void update_gaze(uint32_t now_ms, bool allowed)
{
    if (!allowed) {
        reset_gaze(now_ms);
        schedule_next_gaze(now_ms);
        return;
    }
    if (s_face.gaze.phase == APP_FACE_GAZE_IDLE && elapsed_ms(now_ms, s_face.gaze.next_at_ms) < 0x80000000u) {
        begin_gaze_move(now_ms);
    }
    if (s_face.gaze.phase == APP_FACE_GAZE_MOVING_OUT) {
        sample_gaze_motion(now_ms);
        if (elapsed_ms(now_ms, s_face.gaze.phase_end_ms) < 0x80000000u) {
            begin_gaze_hold(now_ms);
        }
    } else if (s_face.gaze.phase == APP_FACE_GAZE_HOLDING) {
        s_face.gaze.x = s_face.gaze.target_x;
        s_face.gaze.y = s_face.gaze.target_y;
        if (elapsed_ms(now_ms, s_face.gaze.phase_end_ms) < 0x80000000u) {
            begin_gaze_return(now_ms);
        }
    } else if (s_face.gaze.phase == APP_FACE_GAZE_RETURNING) {
        sample_gaze_motion(now_ms);
        if (elapsed_ms(now_ms, s_face.gaze.phase_end_ms) < 0x80000000u) {
            reset_gaze(now_ms);
            schedule_next_gaze(now_ms);
        }
    }
}

static app_face_energy_profile_t energy_profile(void)
{
    const app_face_energy_config_t *cfg = &APP_FACE_DEFAULT_CONFIG.energy;
    float e = app_face_clampf(s_face.energy, 0.0f, 1.0f);
    float lively = app_face_smoothf(e);
    return (app_face_energy_profile_t){
        .raw = e,
        .lively = lively,
        .gaze_ease = app_face_lerpf(cfg->gaze_ease_low, cfg->gaze_ease_high, lively),
        .pulse = app_face_lerpf(cfg->pulse_low, cfg->pulse_high, lively),
        .bounce = app_face_lerpf(cfg->bounce_low, cfg->bounce_high, lively),
        .breathing = app_face_lerpf(cfg->breathing_low, cfg->breathing_high, lively),
        .blink_interval_ms = app_face_lerpf(cfg->blink_interval_low_ms, cfg->blink_interval_high_ms, lively),
        .blink_duration_ms = app_face_lerpf(cfg->blink_duration_low_ms, cfg->blink_duration_high_ms, lively),
    };
}

static void set_transition_target(app_face_mode_t mode, uint32_t now_ms)
{
    if (mode < 0 || mode >= APP_FACE_MODE_COUNT || s_face.target_mode == mode) {
        return;
    }
    s_face.previous_mode = s_face.target_mode;
    s_face.target_mode = mode;
    s_face.transition_start_ms = now_ms;
    s_face.click_pulse = 1.0f;
    s_face.base_dirty = true;
}

static void current_style(uint32_t now_ms,
                          app_face_mood_numbers_t *mood,
                          app_face_rgb_t *iris,
                          app_face_rgb_t *pupil)
{
    const app_face_mode_style_t *from = &APP_FACE_MODE_STYLES[s_face.previous_mode];
    const app_face_mode_style_t *to = &APP_FACE_MODE_STYLES[s_face.target_mode];
    float t = (float)elapsed_ms(now_ms, s_face.transition_start_ms) / (float)APP_FACE_DEFAULT_CONFIG.motion.transition_ms;
    t = app_face_smoothf(t);
    *mood = app_face_mood_lerp(&from->v, &to->v, t);
    *iris = app_face_rgb_lerp(from->iris, to->iris, t);
    *pupil = app_face_rgb_lerp(from->pupil, to->pupil, t);
}

static app_face_orientation_t orientation3d(const app_face_mood_numbers_t *mood, float nod)
{
    const app_face_motion_config_t *motion = &APP_FACE_DEFAULT_CONFIG.motion;
    float axis_x = s_face.axis_x;
    float axis_y = s_face.axis_y;
    float tilt_len = sqrtf(axis_x * axis_x + axis_y * axis_y);
    float tilt_limit = fmaxf(0.1f, motion->tilt_vector_limit);
    if (tilt_len > tilt_limit) {
        float fit = tilt_limit / tilt_len;
        axis_x *= fit;
        axis_y *= fit;
    }

    float yaw_input = tanhf(axis_x * 1.35f) / tanhf(1.35f);
    return (app_face_orientation_t){
        .yaw = yaw_input * 0.84f,
        .pitch = -axis_y * 0.86f + nod * 0.22f,
        .roll = s_face.axis_z * APP_FACE_PI + mood->eye_tilt * 0.12f,
    };
}

static void draw_static_screen(app_face_canvas_t *canvas)
{
    const app_face_geometry_config_t *geo = &APP_FACE_DEFAULT_CONFIG.geometry;
    float w = (float)canvas->width;
    float h = (float)canvas->height;
    float scale = fminf(w, h);
    float cx = w * 0.5f;
    float cy = h * 0.5f;
    float radius = scale * geo->display_radius;
    app_face_rgb_t screen = {.r = 3, .g = 5, .b = 9};
    app_face_rgb_t inner = {.r = 5, .g = 8, .b = 13};

    app_face_canvas_clear(canvas, (app_face_rgb_t){.r = 1, .g = 2, .b = 5});
    app_face_canvas_fill_ellipse(canvas, cx, cy, radius, radius, 0.0f, screen, 255);
    app_face_canvas_set_clip_circle(canvas, cx, cy, radius - 2.0f);
    app_face_canvas_fill_rect(canvas, (int)(cx - radius), (int)(cy - radius), (int)(radius * 2.0f), (int)(radius * 2.0f), inner, 255);
    app_face_canvas_clear_clip(canvas);
}

static void transformed_point(const app_face_surface_t *surface,
                              float cx,
                              float cy,
                              float local_x,
                              float local_y,
                              app_face_vec2_t *out)
{
    out->x = cx + surface->x + surface->ux * local_x + surface->vx * local_y;
    out->y = cy + surface->y + surface->uy * local_x + surface->vy * local_y;
}

static float projected_radial_half(const app_face_surface_t *surface, float local_w, float local_h)
{
    float center_dist = sqrtf(surface->x * surface->x + surface->y * surface->y);
    if (center_dist < 0.0001f) {
        return fmaxf(local_w, local_h) * 0.5f;
    }

    float radial_x = surface->x / center_dist;
    float radial_y = surface->y / center_dist;
    float u_radial = surface->ux * radial_x + surface->uy * radial_y;
    float v_radial = surface->vx * radial_x + surface->vy * radial_y;
    return (fabsf(u_radial) * local_w + fabsf(v_radial) * local_h) * 0.54f;
}

static void keep_surface_inside_display(app_face_surface_t *surface, float display_radius, float local_w, float local_h)
{
    const app_face_geometry_config_t *geo = &APP_FACE_DEFAULT_CONFIG.geometry;
    float center_dist = sqrtf(surface->x * surface->x + surface->y * surface->y);
    if (center_dist < 0.0001f) {
        return;
    }

    float radial_half = projected_radial_half(surface, local_w, local_h);
    float safe_edge = display_radius * (1.0f - geo->projected_eye_edge_margin);
    float max_center_dist = fmaxf(display_radius * 0.12f, safe_edge - radial_half);
    if (center_dist > max_center_dist) {
        float fit = max_center_dist / center_dist;
        surface->x *= fit;
        surface->y *= fit;
        center_dist = max_center_dist;
    }

    if (surface->z > -0.10f) {
        surface->visibility = fmaxf(surface->visibility, geo->projected_eye_visibility_floor);
    }
}

static void rotate_local(float *x, float *y, float angle)
{
    float c = cosf(angle);
    float s = sinf(angle);
    float rx = *x * c - *y * s;
    float ry = *x * s + *y * c;
    *x = rx;
    *y = ry;
}

static size_t build_eye_shape(const app_face_mood_numbers_t *mood,
                              int side,
                              float w,
                              float h,
                              float close,
                              app_face_vec2_t *local,
                              size_t cap)
{
    if (cap < 48) {
        return 0;
    }
    float round_eye = app_face_smoothf(app_face_clampf(mood->round_eye, 0.0f, 1.0f));
    float expressive_open = app_face_lerpf(mood->open, 1.0f, round_eye * 0.95f);
    float open = app_face_clampf(expressive_open * s_face.blink.value * (1.0f - close * 0.86f), 0.03f, 1.08f);
    float hh = h * open * 0.5f;
    float hw = w * 0.5f;
    float angle = mood->eye_tilt * 0.55f;
    float smile = app_face_smoothf(app_face_clampf(mood->smile, 0.0f, 1.0f));
    float concave = app_face_smoothf(app_face_clampf(mood->concave_top, 0.0f, 1.0f));

    size_t n = 0;
    if (smile > 0.38f) {
        for (int i = 0; i < 24; i++) {
            float t = (float)i / 23.0f;
            float x = -hw + t * w;
            float y = -hh * 0.08f + sinf(t * APP_FACE_PI) * hh * (1.05f + smile * 0.2f);
            rotate_local(&x, &y, angle);
            local[n++] = (app_face_vec2_t){.x = x, .y = y};
        }
        for (int i = 23; i >= 0; i--) {
            float t = (float)i / 23.0f;
            float x = -hw + t * w;
            float y = -hh * 0.05f + sinf(t * APP_FACE_PI) * hh * (0.54f + smile * 0.08f);
            rotate_local(&x, &y, angle);
            local[n++] = (app_face_vec2_t){.x = x, .y = y};
        }
        return n;
    }

    if (concave > 0.30f) {
        float inward = side < 0 ? 1.0f : -1.0f;
        for (int i = 0; i < 24; i++) {
            float t = (float)i / 23.0f;
            float x = -hw + t * w;
            float center_dip = sinf(t * APP_FACE_PI) * hh * (0.82f * concave);
            float tilt = (t - 0.5f) * mood->pinch * inward * hh * 0.55f;
            float y = -hh * (0.80f - concave * 0.22f) + center_dip + tilt + mood->lid * h * 0.04f;
            rotate_local(&x, &y, angle);
            local[n++] = (app_face_vec2_t){.x = x, .y = y};
        }
        for (int i = 23; i >= 0; i--) {
            float t = (float)i / 23.0f;
            float x = -hw + t * w;
            float y = hh * (0.72f + 0.14f * concave) + sinf(t * APP_FACE_PI) * hh * 0.20f;
            rotate_local(&x, &y, angle);
            local[n++] = (app_face_vec2_t){.x = x, .y = y};
        }
        return n;
    }

    float lid = mood->lid * h * 0.07f;
    float slant = mood->slant * (float)(-side) * h * 0.18f;
    for (int i = 0; i < 48; i++) {
        float a = ((float)i / 48.0f) * APP_FACE_TAU;
        float x = cosf(a) * hw;
        float y = sinf(a) * hh + lid + (x / fmaxf(1.0f, hw)) * slant;
        rotate_local(&x, &y, angle);
        local[n++] = (app_face_vec2_t){.x = x, .y = y};
    }
    return n;
}

static void draw_eye(app_face_canvas_t *canvas,
                     float cx,
                     float cy,
                     const app_face_surface_t *surface,
                     int side,
                     const app_face_mood_numbers_t *mood,
                     app_face_rgb_t iris,
                     app_face_rgb_t pupil,
                     float eye_w,
                     float eye_h,
                     float look_x,
                     float look_y,
                     float scale)
{
    if (surface->visibility <= 0.025f) {
        return;
    }
    float round_eye = app_face_smoothf(app_face_clampf(mood->round_eye, 0.0f, 1.0f));
    float local_h = app_face_lerpf(eye_h, eye_w, round_eye);
    float close = (s_face.micro_squint + fmaxf(0.0f, mood->lid) * 0.12f) * (1.0f - round_eye * 0.65f);
    float smile = app_face_smoothf(app_face_clampf(mood->smile, 0.0f, 1.0f));
    float concave = app_face_smoothf(app_face_clampf(mood->concave_top, 0.0f, 1.0f));
    float expressive_open = app_face_lerpf(mood->open, 1.0f, round_eye * 0.95f);
    float open = app_face_clampf(expressive_open * s_face.blink.value * (1.0f - close * 0.86f), 0.03f, 1.08f);
    bool use_fast_ellipse = round_eye > 0.72f && smile < 0.18f && concave < 0.20f && fabsf(mood->slant) < 0.18f;
    if (use_fast_ellipse) {
        float angle = mood->eye_tilt * 0.55f;
        float c = cosf(angle);
        float s = sinf(angle);
        float ux = surface->ux * c + surface->vx * s;
        float uy = surface->uy * c + surface->vy * s;
        float vx = -surface->ux * s + surface->vx * c;
        float vy = -surface->uy * s + surface->vy * c;
        app_face_canvas_fill_transformed_ellipse(canvas,
                                                 cx + surface->x,
                                                 cy + surface->y,
                                                 ux,
                                                 uy,
                                                 vx,
                                                 vy,
                                                 eye_w * 0.5f,
                                                 local_h * open * 0.5f,
                                                 iris,
                                                 alpha_from_float(surface->visibility));
    } else {
        app_face_vec2_t local[APP_FACE_MAX_POLY_POINTS];
        app_face_vec2_t screen[APP_FACE_MAX_POLY_POINTS];
        size_t n = build_eye_shape(mood, side, eye_w, local_h, close, local, APP_FACE_MAX_POLY_POINTS);
        for (size_t i = 0; i < n; i++) {
            transformed_point(surface, cx, cy, local[i].x, local[i].y, &screen[i]);
        }
        app_face_canvas_fill_polygon(canvas, screen, n, iris, alpha_from_float(surface->visibility));
    }

    float smile_pupil_fade = 1.0f - app_face_smoothf((mood->smile - 0.24f) / 0.62f);
    if (smile_pupil_fade <= 0.05f) {
        return;
    }

    float pupil_ratio = 0.15f + mood->pupil_size * 0.22f;
    float pupil_w = eye_w * pupil_ratio;
    float base_pupil_h = local_h * app_face_clampf(app_face_lerpf(mood->open, 1.0f, round_eye * 0.95f), 0.12f, 1.0f) *
                         (0.14f + mood->pupil_size * 0.15f);
    float round_pupil_h = local_h * pupil_ratio * 0.92f;
    float pupil_h = app_face_lerpf(base_pupil_h, round_pupil_h, round_eye);
    float px = app_face_clampf(look_x * eye_w * 0.025f + mood->side_eye * eye_w * 0.08f - side * eye_w * 0.018f,
                               -eye_w * 0.20f,
                               eye_w * 0.20f);
    float py = app_face_clampf(look_y * local_h * 0.025f - local_h * 0.016f, -local_h * 0.15f, local_h * 0.15f);
    float alpha = surface->visibility * smile_pupil_fade;
    app_face_canvas_fill_transformed_ellipse(canvas,
                                             cx + surface->x + surface->ux * px + surface->vx * py,
                                             cy + surface->y + surface->uy * px + surface->vy * py,
                                             surface->ux,
                                             surface->uy,
                                             surface->vx,
                                             surface->vy,
                                             pupil_w,
                                             pupil_h,
                                             pupil,
                                             alpha_from_float(alpha));
    if (mood->sparkle > 0.05f && mood->concave_top < 0.35f) {
        app_face_canvas_fill_transformed_ellipse(canvas,
                                                 cx + surface->x + surface->ux * (px - side * pupil_w * 0.45f) + surface->vx * (py - pupil_h * 0.35f),
                                                 cy + surface->y + surface->uy * (px - side * pupil_w * 0.45f) + surface->vy * (py - pupil_h * 0.35f),
                                                 surface->ux,
                                                 surface->uy,
                                                 surface->vx,
                                                 surface->vy,
                                                 fmaxf(1.3f, scale * 0.006f * mood->sparkle),
                                                 fmaxf(1.3f, scale * 0.006f * mood->sparkle),
                                                 pupil,
                                                 alpha_from_float(alpha * mood->sparkle * 0.55f));
    }
}

static void compute_draw_layout(const app_face_canvas_t *canvas,
                                const app_face_mood_numbers_t *mood,
                                const app_face_energy_profile_t *energy,
                                app_face_draw_layout_t *layout)
{
    const app_face_geometry_config_t *geo = &APP_FACE_DEFAULT_CONFIG.geometry;
    float w = (float)canvas->width;
    float h = (float)canvas->height;
    float scale = fminf(w, h);
    float cx = w * 0.5f;
    float cy = h * 0.5f;
    float radius = scale * geo->display_radius;
    float nod = mood->nod * sinf(s_face.time_s * (APP_FACE_DEFAULT_CONFIG.motion.nod_frequency_base +
                                                  energy->raw * APP_FACE_DEFAULT_CONFIG.motion.nod_frequency_energy));
    float voice = mood->bounce * energy->bounce;
    float voice_beat = app_face_clampf(sinf(s_face.time_s * 11.8f) * sinf(s_face.time_s * 7.1f + 2.0f) * voice, -0.82f, 0.82f);
    float eye_breath_y = sinf(s_face.time_s * 3.3f) * scale * 0.006f * mood->pulse * energy->pulse;
    float face_motion_y = sinf(s_face.time_s * 2.1f) * scale * 0.003f * mood->bob + nod * scale * 0.008f;
    app_face_orientation_t orientation = orientation3d(mood, nod);
    float sphere_radius = radius * geo->sphere_radius;

    float drift_phase = s_face.time_s * (APP_FACE_DEFAULT_CONFIG.motion.drift_speed_base +
                                         energy->raw * APP_FACE_DEFAULT_CONFIG.motion.drift_speed_energy);
    float drift_x = (sinf(drift_phase * 0.91f) + sinf(drift_phase * 0.37f + 1.6f) * 0.36f) *
                    app_face_lerpf(APP_FACE_DEFAULT_CONFIG.motion.drift_max_x * 0.42f,
                                   APP_FACE_DEFAULT_CONFIG.motion.drift_max_x,
                                   energy->lively);
    float drift_y = sinf(drift_phase * 0.73f + 2.2f) *
                    app_face_lerpf(APP_FACE_DEFAULT_CONFIG.motion.drift_max_y * 0.36f,
                                   APP_FACE_DEFAULT_CONFIG.motion.drift_max_y,
                                   energy->lively);
    float suspicious_glance = mood->glance * sinf(s_face.time_s * (1.45f + energy->raw * 0.55f));
    float gaze_x = mood->glance > 0.05f ? suspicious_glance : drift_x + s_face.gaze.x;
    float gaze_y = mood->glance > 0.05f ? 0.0f : drift_y + s_face.gaze.y;
    gaze_x = app_face_clampf(gaze_x, -APP_FACE_DEFAULT_CONFIG.motion.gaze_max_total_x, APP_FACE_DEFAULT_CONFIG.motion.gaze_max_total_x);
    gaze_y = app_face_clampf(gaze_y, -APP_FACE_DEFAULT_CONFIG.motion.gaze_max_total_y, APP_FACE_DEFAULT_CONFIG.motion.gaze_max_total_y);

    float eye_w = radius * mood->eye_w * geo->eye_width_scale * (1.0f + voice_beat * 0.04f);
    float eye_h = radius * mood->eye_h * geo->eye_height_scale * (1.0f + voice_beat * 0.13f);
    float sep = radius * mood->sep * geo->eye_separation_scale;
    float projected_half = fmaxf(eye_w, eye_h) * 0.5f;
    float available = radius * geo->fit_horizontal_edge;
    float min_gap = radius * (mood->concave_top > 0.4f ? geo->concave_center_gap : geo->calm_center_gap);
    if (sep + projected_half > available) {
        float fit = app_face_clampf((available - sep * 0.18f) / fmaxf(1.0f, sep + projected_half), 0.66f, 1.0f);
        eye_w *= fit;
        eye_h *= fit;
        projected_half *= fit;
    }
    sep = fmaxf(sep, projected_half + min_gap * 0.5f);
    float max_shift_x = fmaxf(0.0f, radius * geo->max_horizontal_edge - (sep + projected_half));
    float max_shift_y = fmaxf(0.0f, radius * geo->fit_vertical_half - eye_h * 0.5f);
    float base_y = app_face_clampf(radius * mood->y + eye_breath_y + voice_beat * scale * 0.012f + face_motion_y, -max_shift_y, max_shift_y);
    float gaze_offset_x = app_face_clampf(gaze_x * radius * geo->gaze_center_shift_x, -max_shift_x, max_shift_x);
    float gaze_offset_y = app_face_clampf(gaze_y * radius * geo->gaze_center_shift_y, -max_shift_y, max_shift_y);
    float round_eye = app_face_smoothf(mood->round_eye);
    float local_h = app_face_lerpf(eye_h, eye_w, round_eye);
    float patch_w = eye_w * geo->patch_scale;
    float patch_h = local_h * geo->patch_scale;

    layout->scale = scale;
    layout->cx = cx;
    layout->cy = cy;
    layout->radius = radius;
    layout->face_motion_y = face_motion_y;
    layout->voice_beat = voice_beat;
    layout->eye_w = eye_w;
    layout->eye_h = eye_h;
    layout->local_h = local_h;
    layout->sep = sep;
    layout->base_y = base_y;
    layout->patch_w = patch_w;
    layout->patch_h = patch_h;
    layout->left = app_face_eye_surface(-sep + gaze_offset_x, base_y + gaze_offset_y, patch_w, patch_h, sphere_radius, radius, orientation);
    layout->right = app_face_eye_surface(sep + gaze_offset_x, base_y + gaze_offset_y, patch_w, patch_h, sphere_radius, radius, orientation);
    keep_surface_inside_display(&layout->left, radius - 2.0f, eye_w, local_h);
    keep_surface_inside_display(&layout->right, radius - 2.0f, eye_w, local_h);
    layout->look_x = fabsf(s_face.axis_x) + fabsf(s_face.axis_y) > 0.08f ? s_face.axis_x : 0.0f;
    layout->look_y = app_face_clampf((fabsf(s_face.axis_x) + fabsf(s_face.axis_y) > 0.08f ? s_face.axis_y : 0.0f) -
                                         mood->think_dots * 0.28f,
                                     -1.0f,
                                     1.0f);
}

static void draw_screen(app_face_canvas_t *canvas,
                        uint32_t now_ms,
                        const app_face_mood_numbers_t *mood,
                        app_face_rgb_t iris,
                        app_face_rgb_t pupil,
                        const app_face_energy_profile_t *energy,
                        const app_face_draw_layout_t *layout)
{
    (void)now_ms;
    float scale = layout->scale;
    float cx = layout->cx;
    float cy = layout->cy;
    float radius = layout->radius;
    float face_motion_y = layout->face_motion_y;
    float eye_w = layout->eye_w;
    float eye_h = layout->eye_h;
    const app_face_surface_t *left = &layout->left;
    const app_face_surface_t *right = &layout->right;
    float look_x = layout->look_x;
    float look_y = layout->look_y;
    app_face_canvas_set_clip_circle(canvas, cx, cy, radius - 2.0f);

    if (left->z <= right->z) {
        draw_eye(canvas, cx, cy, left, -1, mood, iris, pupil, eye_w, eye_h, look_x, look_y, scale);
        draw_eye(canvas, cx, cy, right, 1, mood, iris, pupil, eye_w, eye_h, look_x, look_y, scale);
    } else {
        draw_eye(canvas, cx, cy, right, 1, mood, iris, pupil, eye_w, eye_h, look_x, look_y, scale);
        draw_eye(canvas, cx, cy, left, -1, mood, iris, pupil, eye_w, eye_h, look_x, look_y, scale);
    }

    if (mood->think_dots > 0.05f) {
        for (int i = 0; i < 3; i++) {
            float phase = s_face.time_s * (2.4f + energy->raw * 2.2f) + (float)i * 1.1f;
            float dot_x = cx + radius * (-0.18f + (float)i * 0.18f);
            float dot_y = cy - radius * 0.52f + face_motion_y + sinf(phase) * radius * 0.015f;
            app_face_canvas_fill_ellipse(canvas, dot_x, dot_y, radius * (0.025f + (float)i * 0.004f), radius * (0.025f + (float)i * 0.004f), 0.0f, iris,
                                         alpha_from_float((0.46f + 0.16f * energy->lively) * mood->think_dots));
        }
    }

    if (mood->speaking_bars > 0.08f) {
        int count = 7;
        float total = radius * 0.56f;
        float start = cx - total * 0.5f;
        for (int i = 0; i < count; i++) {
            float amp = fabsf(sinf(s_face.time_s * (7.0f + energy->raw * 18.0f) + (float)i * 0.72f)) *
                            radius * (0.05f + 0.13f * energy->lively) +
                        radius * 0.02f;
            float x = start + (float)i * (total / (float)(count - 1));
            app_face_canvas_fill_ellipse(canvas, x, cy + radius * 0.50f + face_motion_y, radius * 0.018f, amp * 0.5f, 0.0f, iris,
                                         alpha_from_float((0.26f + 0.38f * energy->lively) * mood->speaking_bars));
        }
    }

    app_face_canvas_clear_clip(canvas);
}

static bool surface_bounds_area(const app_face_canvas_t *canvas,
                                const app_face_draw_layout_t *layout,
                                const app_face_surface_t *surface,
                                float local_w,
                                float local_h,
                                float margin,
                                lv_area_t *out)
{
    if (surface->visibility <= 0.01f) {
        area_reset(out, canvas->width, canvas->height);
        return false;
    }
    float half_x = (fabsf(surface->ux) * local_w + fabsf(surface->vx) * local_h) * 0.58f + margin;
    float half_y = (fabsf(surface->uy) * local_w + fabsf(surface->vy) * local_h) * 0.58f + margin;
    float cx = layout->cx + surface->x;
    float cy = layout->cy + surface->y;
    area_reset(out, canvas->width, canvas->height);
    area_include_rect(out, cx - half_x, cy - half_y, cx + half_x, cy + half_y);
    area_inflate_clip(out, (int)(layout->scale * 0.018f) + 6, canvas->width, canvas->height);
    return !area_is_empty(out);
}

static lv_area_t display_dirty_area(const app_face_canvas_t *canvas, const app_face_draw_layout_t *layout)
{
    float display_margin = 4.0f;
    lv_area_t area = {
        .x1 = (lv_coord_t)floorf(layout->cx - layout->radius - display_margin),
        .y1 = (lv_coord_t)floorf(layout->cy - layout->radius - display_margin),
        .x2 = (lv_coord_t)ceilf(layout->cx + layout->radius + display_margin),
        .y2 = (lv_coord_t)ceilf(layout->cy + layout->radius + display_margin),
    };
    area_clip_to_bounds(&area, canvas->width, canvas->height);
    return area;
}

static bool clip_area_to_area(lv_area_t *area, const lv_area_t *clip, int width, int height)
{
    if (area_is_empty(area) || area_is_empty(clip)) {
        return false;
    }
    area->x1 = clamp_int(area->x1, clip->x1, clip->x2);
    area->y1 = clamp_int(area->y1, clip->y1, clip->y2);
    area->x2 = clamp_int(area->x2, clip->x1, clip->x2);
    area->y2 = clamp_int(area->y2, clip->y1, clip->y2);
    return area_clip_to_bounds(area, width, height);
}

static void estimate_dynamic_list(const app_face_canvas_t *canvas,
                                  const app_face_mood_numbers_t *mood,
                                  const app_face_energy_profile_t *energy,
                                  const app_face_draw_layout_t *layout,
                                  app_face_dirty_list_t *list)
{
    dirty_list_reset(list);
    lv_area_t display_area = display_dirty_area(canvas, layout);
    float dynamic_margin = layout->scale * 0.038f;
    lv_area_t area;

    if (surface_bounds_area(canvas, layout, &layout->left, layout->eye_w, layout->local_h, dynamic_margin, &area) &&
        clip_area_to_area(&area, &display_area, canvas->width, canvas->height)) {
        dirty_list_add_area(list, area, canvas->width, canvas->height);
    }

    if (surface_bounds_area(canvas, layout, &layout->right, layout->eye_w, layout->local_h, dynamic_margin, &area) &&
        clip_area_to_area(&area, &display_area, canvas->width, canvas->height)) {
        dirty_list_add_area(list, area, canvas->width, canvas->height);
    }

    if (mood->think_dots > 0.05f) {
        area_reset(&area, canvas->width, canvas->height);
        area_include_rect(&area,
                          layout->cx - layout->radius * 0.28f,
                          layout->cy - layout->radius * 0.62f + layout->face_motion_y,
                          layout->cx + layout->radius * 0.28f,
                          layout->cy - layout->radius * 0.42f + layout->face_motion_y);
        area_inflate_clip(&area, 6, canvas->width, canvas->height);
        if (clip_area_to_area(&area, &display_area, canvas->width, canvas->height)) {
            dirty_list_add_area(list, area, canvas->width, canvas->height);
        }
    }

    if (mood->speaking_bars > 0.08f) {
        area_reset(&area, canvas->width, canvas->height);
        float amp = layout->radius * (0.06f + 0.15f * energy->lively);
        area_include_rect(&area,
                          layout->cx - layout->radius * 0.34f,
                          layout->cy + layout->radius * 0.50f + layout->face_motion_y - amp,
                          layout->cx + layout->radius * 0.34f,
                          layout->cy + layout->radius * 0.50f + layout->face_motion_y + amp);
        area_inflate_clip(&area, 6, canvas->width, canvas->height);
        if (clip_area_to_area(&area, &display_area, canvas->width, canvas->height)) {
            dirty_list_add_area(list, area, canvas->width, canvas->height);
        }
    }
}

static void fragment_list_add(lv_area_t *fragments, size_t *count, const lv_area_t *area)
{
    if (area_is_empty(area) || *count >= APP_FACE_MAX_BLIT_FRAGMENTS) {
        return;
    }
    fragments[(*count)++] = *area;
}

static size_t subtract_protected_areas(const lv_area_t *area, lv_area_t *out_fragments, size_t max_fragments)
{
    if (area_is_empty(area) || max_fragments == 0) {
        return 0;
    }

    lv_area_t fragments[APP_FACE_MAX_BLIT_FRAGMENTS];
    size_t fragment_count = 1;
    fragments[0] = *area;

    for (size_t p = 0; p < s_face.protected_area_count; p++) {
        lv_area_t protect = s_face.protected_areas[p];
        area_inflate_clip(&protect, 2, s_face.canvas.width, s_face.canvas.height);
        if (area_is_empty(&protect)) {
            continue;
        }

        lv_area_t next[APP_FACE_MAX_BLIT_FRAGMENTS];
        size_t next_count = 0;
        for (size_t i = 0; i < fragment_count; i++) {
            lv_area_t src = fragments[i];
            if (!area_intersects(&src, &protect)) {
                fragment_list_add(next, &next_count, &src);
                continue;
            }

            lv_area_t top = src;
            top.y2 = protect.y1 - 1;
            fragment_list_add(next, &next_count, &top);

            lv_area_t bottom = src;
            bottom.y1 = protect.y2 + 1;
            fragment_list_add(next, &next_count, &bottom);

            lv_area_t left = src;
            left.y1 = fmax(src.y1, protect.y1);
            left.y2 = fmin(src.y2, protect.y2);
            left.x2 = protect.x1 - 1;
            fragment_list_add(next, &next_count, &left);

            lv_area_t right = src;
            right.y1 = fmax(src.y1, protect.y1);
            right.y2 = fmin(src.y2, protect.y2);
            right.x1 = protect.x2 + 1;
            fragment_list_add(next, &next_count, &right);
        }
        memcpy(fragments, next, next_count * sizeof(fragments[0]));
        fragment_count = next_count;
        if (fragment_count == 0) {
            break;
        }
    }

    size_t copied = fragment_count < max_fragments ? fragment_count : max_fragments;
    memcpy(out_fragments, fragments, copied * sizeof(out_fragments[0]));
    return copied;
}

static esp_err_t blit_area_from_framebuffer(const lv_area_t *area,
                                            uint32_t *packed_bytes,
                                            uint32_t *lcd_wait_us)
{
    if (area_is_empty(area) || !s_face.tx_buffers[0]) {
        return ESP_OK;
    }

    const int width = area_width(area);
    if (width <= 0 || area_height(area) <= 0) {
        return ESP_OK;
    }

    const int pixels_per_buffer = APP_FACE_TX_BUFFER_BYTES / (int)sizeof(lv_color_t);
    int rows_per_chunk = pixels_per_buffer / width;
    if (rows_per_chunk < 1) {
        rows_per_chunk = 1;
    }

    for (int y = area->y1; y <= area->y2; y += rows_per_chunk) {
        int rows = area->y2 - y + 1;
        if (rows > rows_per_chunk) {
            rows = rows_per_chunk;
        }

        int64_t wait_start = esp_timer_get_time();
        esp_err_t err = app_lcd_wait_idle(UINT32_MAX);
        if (lcd_wait_us) {
            *lcd_wait_us += (uint32_t)(esp_timer_get_time() - wait_start);
        }
        if (err != ESP_OK) {
            return err;
        }

        lv_color_t *tx = s_face.tx_buffers[s_face.tx_index % APP_FACE_TX_BUFFER_COUNT];
        s_face.tx_index++;
        size_t row_bytes = (size_t)width * sizeof(lv_color_t);
        for (int row = 0; row < rows; row++) {
            memcpy(&tx[row * width],
                   &s_face.pixels[(y + row) * s_face.canvas.width + area->x1],
                   row_bytes);
        }

        if (packed_bytes) {
            *packed_bytes += (uint32_t)(row_bytes * (size_t)rows);
        }
        err = app_lcd_blit_rect_async(area->x1, y, width, rows, tx, NULL, NULL);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

static esp_err_t blit_dirty_area(const lv_area_t *area,
                                 uint32_t *dirty_pixels,
                                 uint32_t *packed_bytes,
                                 uint32_t *lcd_wait_us)
{
    lv_area_t fragments[APP_FACE_MAX_BLIT_FRAGMENTS];
    size_t count = subtract_protected_areas(area, fragments, APP_FACE_MAX_BLIT_FRAGMENTS);
    esp_err_t final_err = ESP_OK;
    for (size_t i = 0; i < count; i++) {
        lv_area_t fragment = fragments[i];
        if (!area_clip_to_bounds(&fragment, s_face.canvas.width, s_face.canvas.height)) {
            continue;
        }
        if (dirty_pixels) {
            *dirty_pixels += area_pixel_count(&fragment);
        }
        esp_err_t err = blit_area_from_framebuffer(&fragment, packed_bytes, lcd_wait_us);
        if (err != ESP_OK) {
            final_err = err;
        }
    }
    return final_err;
}

static void update_stats(uint32_t now_ms,
                         uint32_t render_time_us,
                         uint32_t dirty_pixels,
                         uint32_t packed_bytes,
                         uint32_t lcd_wait_us,
                         bool blitted)
{
    s_face.stats.target_fps = CONFIG_APP_FACE_FPS;
    s_face.stats.render_time_us = render_time_us;
    s_face.stats.dirty_pixels = dirty_pixels;
    s_face.stats.packed_bytes = packed_bytes;
    s_face.stats.lcd_wait_time_us = lcd_wait_us;

    s_face.stats_rendered_frames++;
    if (blitted) {
        s_face.stats_blit_frames++;
    }
    s_face.stats_dirty_pixels += dirty_pixels;
    s_face.stats_packed_bytes += packed_bytes;

    uint32_t elapsed = elapsed_ms(now_ms, s_face.stats_window_start_ms);
    if (elapsed >= 1000u) {
        s_face.stats.rendered_fps = (s_face.stats_rendered_frames * 1000u) / elapsed;
        s_face.stats.blit_fps = (s_face.stats_blit_frames * 1000u) / elapsed;
        s_face.stats.skipped_frames = s_face.stats_skipped_frames;
#if CONFIG_APP_FACE_PROFILE
        ESP_LOGI(TAG,
                 "face target=%lu render=%lu blit=%lu render_us=%lu dirty=%lu packed=%lu wait_us=%lu skipped=%lu",
                 (unsigned long)s_face.stats.target_fps,
                 (unsigned long)s_face.stats.rendered_fps,
                 (unsigned long)s_face.stats.blit_fps,
                 (unsigned long)s_face.stats.render_time_us,
                 (unsigned long)s_face.stats.dirty_pixels,
                 (unsigned long)s_face.stats.packed_bytes,
                 (unsigned long)s_face.stats.lcd_wait_time_us,
                 (unsigned long)s_face.stats.skipped_frames);
#endif
        s_face.stats_window_start_ms = now_ms;
        s_face.stats_rendered_frames = 0;
        s_face.stats_blit_frames = 0;
        s_face.stats_skipped_frames = 0;
        s_face.stats_dirty_pixels = 0;
        s_face.stats_packed_bytes = 0;
    }
}

static void update_runtime(uint32_t now_ms)
{
    uint32_t dt_ms = elapsed_ms(now_ms, s_face.last_update_ms);
    if (dt_ms > 50) {
        dt_ms = 50;
    }
    s_face.last_update_ms = now_ms;
    float seconds = (float)dt_ms / 1000.0f;
    s_face.time_s += seconds;

#if CONFIG_APP_FACE_USE_IMU
    float ax;
    float ay;
    float az;
    if (app_motion_get_axes(&ax, &ay, &az)) {
        app_face_set_axes(ax, ay, az);
    }
#endif

    s_face.energy += (s_face.target_energy - s_face.energy) * (1.0f - powf(0.001f, seconds));
    app_face_energy_profile_t energy = energy_profile();

    if (s_face.pulse_active && elapsed_ms(now_ms, s_face.pulse_until_ms) < 0x80000000u) {
        s_face.pulse_active = false;
        set_transition_target(s_face.pulse_return_mode, now_ms);
    }

    if (!s_face.blink.forced && elapsed_ms(now_ms, s_face.blink.next_ms) < 0x80000000u) {
        app_face_force_blink();
    }
    if (s_face.blink.forced) {
        float sleepy = s_face.target_mode == APP_FACE_SLEEPY ? 1.45f : 1.0f;
        float close_ms = energy.blink_duration_ms * APP_FACE_DEFAULT_CONFIG.motion.blink_close * sleepy;
        float hold_ms = energy.blink_duration_ms * APP_FACE_DEFAULT_CONFIG.motion.blink_hold * sleepy;
        float open_ms = energy.blink_duration_ms * APP_FACE_DEFAULT_CONFIG.motion.blink_open * sleepy;
        float t = (float)elapsed_ms(now_ms, s_face.blink.start_ms);
        if (t < close_ms) {
            s_face.blink.value = 1.0f - app_face_smoothf(t / close_ms);
        } else if (t < close_ms + hold_ms) {
            s_face.blink.value = APP_FACE_DEFAULT_CONFIG.motion.blink_minimum_open;
        } else if (t < close_ms + hold_ms + open_ms) {
            s_face.blink.value = app_face_smoothf((t - close_ms - hold_ms) / open_ms);
        } else {
            s_face.blink.value = 1.0f;
            s_face.blink.forced = false;
            float nervous = s_face.target_mode == APP_FACE_ERROR ? 0.56f : 1.0f;
            s_face.blink.next_ms = now_ms + (uint32_t)(energy.blink_interval_ms * nervous +
                                                       rng_float() * energy.blink_interval_ms * APP_FACE_DEFAULT_CONFIG.motion.blink_randomness);
        }
    }

    float squint_target = 0.0f;
    if (s_face.target_mode == APP_FACE_SUSPICIOUS || s_face.target_mode == APP_FACE_ANGRY) {
        squint_target = 0.18f + 0.05f * sinf(s_face.time_s * 2.1f);
    } else if (s_face.target_mode == APP_FACE_SLEEPY) {
        squint_target = 0.10f;
    }
    s_face.micro_squint += (squint_target - s_face.micro_squint) * (1.0f - powf(0.008f, seconds));
    if (s_face.click_pulse > 0.0f) {
        s_face.click_pulse = fmaxf(0.0f, s_face.click_pulse - (float)dt_ms / (float)APP_FACE_DEFAULT_CONFIG.motion.click_pulse_ms);
    }
}

static void render(uint32_t now_ms)
{
    int64_t render_start_us = esp_timer_get_time();
    app_face_mood_numbers_t mood;
    app_face_rgb_t iris;
    app_face_rgb_t pupil;
    current_style(now_ms, &mood, &iris, &pupil);

    bool full_refresh = s_face.base_dirty || s_face.full_refresh_requested || !s_face.has_prev_dynamic_list;
    if (s_face.base_dirty) {
        draw_static_screen(&s_face.base_canvas);
        s_face.base_dirty = false;
    }
    app_face_energy_profile_t energy = energy_profile();
    bool ambient_allowed = APP_FACE_MODE_STYLES[s_face.target_mode].ambient_gaze &&
                           fabsf(s_face.axis_x) + fabsf(s_face.axis_y) < 0.08f;
    update_gaze(now_ms, ambient_allowed);

    app_face_draw_layout_t layout;
    compute_draw_layout(&s_face.canvas, &mood, &energy, &layout);

    app_face_dirty_list_t current_list;
    estimate_dynamic_list(&s_face.canvas, &mood, &energy, &layout, &current_list);

    app_face_dirty_list_t dirty_list;
    dirty_list_reset(&dirty_list);
    if (full_refresh) {
        dirty_list_add_area(&dirty_list, display_dirty_area(&s_face.canvas, &layout), s_face.canvas.width, s_face.canvas.height);
    } else {
        dirty_list_add_list(&dirty_list, &current_list, s_face.canvas.width, s_face.canvas.height);
        dirty_list_add_list(&dirty_list, &s_face.prev_dynamic_list, s_face.canvas.width, s_face.canvas.height);
    }

    uint32_t dirty_pixels = 0;
    uint32_t packed_bytes = 0;
    uint32_t lcd_wait_us = 0;
    bool blitted = false;

    if (s_face.base_pixels) {
        for (size_t i = 0; i < dirty_list.count; i++) {
            lv_area_t area = dirty_list.rects[i];
            if (!area_clip_to_bounds(&area, s_face.canvas.width, s_face.canvas.height)) {
                continue;
            }
            app_face_canvas_restore_region(&s_face.canvas,
                                           &s_face.base_canvas,
                                           area.x1,
                                           area.y1,
                                           area.x2,
                                           area.y2);
            app_face_canvas_set_clip_rect(&s_face.canvas, area.x1, area.y1, area.x2, area.y2);
            draw_screen(&s_face.canvas, now_ms, &mood, iris, pupil, &energy, &layout);
        }
    } else {
        app_face_canvas_clear(&s_face.canvas, (app_face_rgb_t){.r = 1, .g = 2, .b = 5});
        draw_screen(&s_face.canvas, now_ms, &mood, iris, pupil, &energy, &layout);
    }

    for (size_t i = 0; i < dirty_list.count; i++) {
        esp_err_t err = blit_dirty_area(&dirty_list.rects[i], &dirty_pixels, &packed_bytes, &lcd_wait_us);
        if (err == ESP_OK) {
            blitted = true;
        } else {
            ESP_LOGW(TAG, "Face blit failed: %s", esp_err_to_name(err));
        }
    }

    s_face.prev_dynamic_list = current_list;
    s_face.has_prev_dynamic_list = true;
    s_face.full_refresh_requested = false;

    uint32_t render_time_us = (uint32_t)(esp_timer_get_time() - render_start_us);
    update_stats(now_ms, render_time_us, dirty_pixels, packed_bytes, lcd_wait_us, blitted);
}

esp_err_t app_face_init(lv_obj_t *parent)
{
    if (s_face.initialized) {
        return ESP_OK;
    }
    memset(&s_face, 0, sizeof(s_face));
    s_face.previous_mode = APP_FACE_IDLE;
    s_face.target_mode = APP_FACE_IDLE;
    s_face.base_mode = APP_FACE_IDLE;
    s_face.pulse_return_mode = APP_FACE_IDLE;
    s_face.energy = 0.42f;
    s_face.target_energy = 0.42f;
    s_face.blink.value = 1.0f;
    s_face.rng = 0xA511E9E5u ^ (uint32_t)lv_tick_get();
    s_face.last_frame_ms = lv_tick_get();
    s_face.last_update_ms = s_face.last_frame_ms;
    s_face.transition_start_ms = s_face.last_frame_ms;
    s_face.blink.next_ms = s_face.last_frame_ms + APP_FACE_DEFAULT_CONFIG.motion.initial_blink_delay_ms;
    reset_gaze(s_face.last_frame_ms);
    schedule_next_gaze(s_face.last_frame_ms);

    const int width = EXAMPLE_LCD_WIDTH;
    const int height = EXAMPLE_LCD_HEIGHT;
    size_t frame_bytes = (size_t)width * (size_t)height * sizeof(lv_color_t);
    s_face.pixels = heap_caps_malloc(frame_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_face.pixels) {
        ESP_LOGE(TAG, "Failed to allocate %ux%u face framebuffer", (unsigned)width, (unsigned)height);
        return ESP_ERR_NO_MEM;
    }
    s_face.base_pixels = heap_caps_malloc(frame_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_face.base_pixels) {
        ESP_LOGE(TAG, "Failed to allocate %ux%u face base framebuffer", (unsigned)width, (unsigned)height);
        heap_caps_free(s_face.pixels);
        s_face.pixels = NULL;
        return ESP_ERR_NO_MEM;
    }
    for (int i = 0; i < APP_FACE_TX_BUFFER_COUNT; i++) {
        s_face.tx_buffers[i] = heap_caps_malloc(APP_FACE_TX_BUFFER_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!s_face.tx_buffers[i]) {
            s_face.tx_buffers[i] = heap_caps_malloc(APP_FACE_TX_BUFFER_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        }
        if (!s_face.tx_buffers[i]) {
            ESP_LOGE(TAG, "Failed to allocate face LCD transfer buffer %d", i);
            for (int j = 0; j < i; j++) {
                heap_caps_free(s_face.tx_buffers[j]);
                s_face.tx_buffers[j] = NULL;
            }
            heap_caps_free(s_face.base_pixels);
            heap_caps_free(s_face.pixels);
            s_face.base_pixels = NULL;
            s_face.pixels = NULL;
            return ESP_ERR_NO_MEM;
        }
    }
    app_face_canvas_init(&s_face.canvas, s_face.pixels, width, height);
    app_face_canvas_init(&s_face.base_canvas, s_face.base_pixels, width, height);
    app_face_canvas_clear(&s_face.canvas, (app_face_rgb_t){.r = 1, .g = 2, .b = 5});
    app_face_canvas_clear(&s_face.base_canvas, (app_face_rgb_t){.r = 1, .g = 2, .b = 5});
    s_face.base_dirty = true;
    s_face.full_refresh_requested = true;
    s_face.stats_window_start_ms = s_face.last_frame_ms;

    s_face.canvas_obj = lv_obj_create(parent);
    lv_obj_set_size(s_face.canvas_obj, width, height);
    lv_obj_center(s_face.canvas_obj);
    lv_obj_set_style_bg_opa(s_face.canvas_obj, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_face.canvas_obj, 0, 0);
    lv_obj_set_style_radius(s_face.canvas_obj, 0, 0);
    lv_obj_set_style_pad_all(s_face.canvas_obj, 0, 0);
    lv_obj_clear_flag(s_face.canvas_obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_face.canvas_obj, LV_OBJ_FLAG_CLICKABLE);
    s_face.initialized = true;
    ESP_LOGI(TAG,
             "Face renderer initialized %dx%d @ %d fps direct LCD, tx=%d bytes x %d",
             width,
             height,
             CONFIG_APP_FACE_FPS,
             APP_FACE_TX_BUFFER_BYTES,
             APP_FACE_TX_BUFFER_COUNT);
    render(s_face.last_frame_ms);
    return ESP_OK;
}

lv_obj_t *app_face_get_obj(void)
{
    return s_face.canvas_obj;
}

void app_face_set_mode(app_face_mode_t mode)
{
    if (mode < 0 || mode >= APP_FACE_MODE_COUNT) {
        return;
    }
    uint32_t now_ms = lv_tick_get();
    s_face.base_mode = mode;
    s_face.pulse_return_mode = mode;
    if (!s_face.pulse_active) {
        set_transition_target(mode, now_ms);
    }
}

void app_face_pulse_mode(app_face_mode_t mode, uint32_t duration_ms)
{
    if (mode < 0 || mode >= APP_FACE_MODE_COUNT) {
        return;
    }
    uint32_t now_ms = lv_tick_get();
    s_face.pulse_active = true;
    s_face.pulse_return_mode = s_face.base_mode;
    s_face.pulse_until_ms = now_ms + duration_ms;
    set_transition_target(mode, now_ms);
}

void app_face_set_energy(float energy_0_to_1)
{
    s_face.target_energy = app_face_clampf(energy_0_to_1, 0.0f, 1.0f);
}

void app_face_set_axes(float axis_x, float axis_y, float axis_z)
{
    s_face.axis_x = app_face_clampf(axis_x, -1.0f, 1.0f);
    s_face.axis_y = app_face_clampf(axis_y, -1.0f, 1.0f);
    s_face.axis_z = app_face_clampf(axis_z, -1.0f, 1.0f);
}

void app_face_set_protected_areas(const lv_area_t *areas, size_t count)
{
    if (!s_face.initialized) {
        return;
    }

    lv_area_t next[APP_FACE_MAX_PROTECTED_RECTS];
    size_t next_count = 0;
    for (size_t i = 0; areas && i < count && next_count < APP_FACE_MAX_PROTECTED_RECTS; i++) {
        lv_area_t area = areas[i];
        if (area_clip_to_bounds(&area, s_face.canvas.width, s_face.canvas.height)) {
            next[next_count++] = area;
        }
    }

    bool changed = next_count != s_face.protected_area_count;
    for (size_t i = 0; !changed && i < next_count; i++) {
        const lv_area_t *a = &next[i];
        const lv_area_t *b = &s_face.protected_areas[i];
        changed = a->x1 != b->x1 || a->y1 != b->y1 || a->x2 != b->x2 || a->y2 != b->y2;
    }
    if (!changed) {
        return;
    }

    memcpy(s_face.protected_areas, next, next_count * sizeof(next[0]));
    s_face.protected_area_count = next_count;
    s_face.full_refresh_requested = true;
}

void app_face_request_full_refresh(void)
{
    if (!s_face.initialized) {
        return;
    }
    s_face.full_refresh_requested = true;
}

bool app_face_get_stats(app_face_stats_t *out_stats)
{
    if (!s_face.initialized || !out_stats) {
        return false;
    }
    *out_stats = s_face.stats;
    return true;
}

void app_face_force_blink(void)
{
    s_face.blink.forced = true;
    s_face.blink.start_ms = lv_tick_get();
}

void app_face_tap(void)
{
    app_face_force_blink();
    s_face.click_pulse = 1.0f;
}

void app_face_tick(uint32_t now_ms)
{
    if (!s_face.initialized) {
        return;
    }
    uint32_t min_frame_ms = 1000u / (CONFIG_APP_FACE_FPS > 0 ? CONFIG_APP_FACE_FPS : 30u);
    if (elapsed_ms(now_ms, s_face.last_frame_ms) < min_frame_ms) {
        return;
    }
    s_face.last_frame_ms = now_ms;
    update_runtime(now_ms);
    render(now_ms);
}

#endif
