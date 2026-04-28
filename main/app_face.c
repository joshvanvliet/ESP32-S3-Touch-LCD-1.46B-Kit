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

#ifndef CONFIG_APP_FACE_TX_BUFFER_COUNT
#define CONFIG_APP_FACE_TX_BUFFER_COUNT 1
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
#define APP_FACE_TX_BUFFER_COUNT CONFIG_APP_FACE_TX_BUFFER_COUNT
#define APP_FACE_MAX_DIRTY_RECTS 10
#define APP_FACE_MAX_BLIT_FRAGMENTS 16
#define APP_FACE_MAX_PROTECTED_RECTS 4
#define APP_FACE_DIRTY_MERGE_GAP 3
#define APP_FACE_PRIMITIVE_MARGIN_PX 3
#define APP_FACE_INPUT_ENERGY_SCALE 0.5f
#define APP_FACE_SPEAKING_INPUT_ENERGY_CAP 0.48f

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
    float display_cx;
    float display_cy;
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
    float ux;
    float uy;
    float vx;
    float vy;
    float visibility;
} app_face_pair_frame_t;

typedef enum {
    APP_FACE_PRIMITIVE_EYE,
    APP_FACE_PRIMITIVE_TRANSFORMED_ELLIPSE,
} app_face_primitive_type_t;

typedef struct {
    app_face_primitive_type_t type;
    lv_area_t bounds;
    app_face_surface_t surface;
    app_face_pair_frame_t frame;
    app_face_mood_numbers_t mood;
    app_face_rgb_t iris;
    app_face_rgb_t pupil;
    app_face_rgb_t color;
    float cx;
    float cy;
    float eye_w;
    float eye_h;
    float look_x;
    float look_y;
    float scale;
    float rx;
    float ry;
    uint8_t alpha;
    int side;
} app_face_render_primitive_t;

#define APP_FACE_MAX_RENDER_PRIMITIVES 14

typedef struct {
    app_face_render_primitive_t items[APP_FACE_MAX_RENDER_PRIMITIVES];
    size_t count;
} app_face_render_plan_t;

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
    uint64_t last_frame_us;
    uint32_t stats_window_start_ms;
    uint32_t stats_rendered_frames;
    uint32_t stats_blit_frames;
    uint32_t stats_skipped_frames;
    uint32_t stats_dirty_pixels;
    uint32_t stats_packed_bytes;
    uint32_t stats_dirty_rects;
    uint32_t stats_lcd_chunks;
    uint32_t stats_restore_time_us;
    uint32_t stats_draw_time_us;
    uint32_t stats_pack_time_us;
    uint32_t stats_lcd_wait_time_us;
    uint32_t stats_full_refreshes;
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

static float speaking_energy_cap(void)
{
    return APP_FACE_SPEAKING_INPUT_ENERGY_CAP * APP_FACE_INPUT_ENERGY_SCALE;
}

static void apply_speaking_energy_cap(app_face_mode_t mode)
{
    if (mode != APP_FACE_SPEAKING) {
        return;
    }
    float cap = speaking_energy_cap();
    s_face.energy = fminf(s_face.energy, cap);
    s_face.target_energy = fminf(s_face.target_energy, cap);
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

static void area_include_transformed_ellipse(lv_area_t *area,
                                             float cx,
                                             float cy,
                                             float ux,
                                             float uy,
                                             float vx,
                                             float vy,
                                             float rx,
                                             float ry,
                                             float margin)
{
    float half_x = fabsf(ux) * rx + fabsf(vx) * ry + margin;
    float half_y = fabsf(uy) * rx + fabsf(vy) * ry + margin;
    area_include_rect(area, cx - half_x, cy - half_y, cx + half_x, cy + half_y);
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

static float normalized_center(float size, float value)
{
    if (value > 0.0f && value < 1.0f) {
        return size * value;
    }
    return size * 0.5f;
}

static void draw_static_screen(app_face_canvas_t *canvas)
{
    const app_face_geometry_config_t *geo = &APP_FACE_DEFAULT_CONFIG.geometry;
    float w = (float)canvas->width;
    float h = (float)canvas->height;
    float scale = fminf(w, h);
    float cx = normalized_center(w, geo->display_center_x);
    float cy = normalized_center(h, geo->display_center_y);
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

static void keep_surface_inside_display(app_face_surface_t *surface,
                                        float display_radius,
                                        float local_w,
                                        float local_h,
                                        float face_to_display_x,
                                        float face_to_display_y)
{
    const app_face_geometry_config_t *geo = &APP_FACE_DEFAULT_CONFIG.geometry;
    float display_x = face_to_display_x + surface->x;
    float display_y = face_to_display_y + surface->y;
    float center_dist = sqrtf(display_x * display_x + display_y * display_y);
    if (center_dist < 0.0001f) {
        return;
    }

    app_face_surface_t display_surface = *surface;
    display_surface.x = display_x;
    display_surface.y = display_y;
    float radial_half = projected_radial_half(&display_surface, local_w, local_h);
    float safe_edge = display_radius * (1.0f - geo->projected_eye_edge_margin);
    float max_center_dist = fmaxf(display_radius * 0.12f, safe_edge - radial_half);
    if (center_dist > max_center_dist) {
        float fit = max_center_dist / center_dist;
        surface->x = display_x * fit - face_to_display_x;
        surface->y = display_y * fit - face_to_display_y;
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
    float blink_pupil_alpha = app_face_smooth_range(0.055f, 0.20f, open);
    pupil_h = fminf(pupil_h, local_h * open * 0.46f);
    if (pupil_h <= 0.55f || blink_pupil_alpha <= 0.02f) {
        return;
    }
    float px = app_face_clampf(look_x * eye_w * 0.025f + mood->side_eye * eye_w * 0.08f - side * eye_w * 0.018f,
                               -eye_w * 0.20f,
                               eye_w * 0.20f);
    float py = app_face_clampf(look_y * local_h * 0.025f - local_h * 0.016f, -local_h * 0.15f, local_h * 0.15f);
    float alpha = surface->visibility * smile_pupil_fade * blink_pupil_alpha;
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
    float display_cx = normalized_center(w, geo->display_center_x);
    float display_cy = normalized_center(h, geo->display_center_y);
    float cx = normalized_center(w, geo->face_center_x);
    float cy = normalized_center(h, geo->face_center_y);
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
    layout->display_cx = display_cx;
    layout->display_cy = display_cy;
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
    float face_to_display_x = cx - display_cx;
    float face_to_display_y = cy - display_cy;
    keep_surface_inside_display(&layout->left, radius - 2.0f, eye_w, local_h, face_to_display_x, face_to_display_y);
    keep_surface_inside_display(&layout->right, radius - 2.0f, eye_w, local_h, face_to_display_x, face_to_display_y);
    layout->look_x = fabsf(s_face.axis_x) + fabsf(s_face.axis_y) > 0.08f ? s_face.axis_x : 0.0f;
    layout->look_y = app_face_clampf((fabsf(s_face.axis_x) + fabsf(s_face.axis_y) > 0.08f ? s_face.axis_y : 0.0f) -
                                         mood->think_dots * 0.28f,
                                     -1.0f,
                                     1.0f);
}

static app_face_pair_frame_t eye_pair_frame(const app_face_draw_layout_t *layout)
{
    const app_face_surface_t *left = &layout->left;
    const app_face_surface_t *right = &layout->right;
    app_face_pair_frame_t frame = {
        .x = (left->x + right->x) * 0.5f,
        .y = (left->y + right->y) * 0.5f,
        .ux = (left->ux + right->ux) * 0.5f,
        .uy = (left->uy + right->uy) * 0.5f,
        .vx = (left->vx + right->vx) * 0.5f,
        .vy = (left->vy + right->vy) * 0.5f,
        .visibility = fmaxf(left->visibility, right->visibility),
    };

    float dx = right->x - left->x;
    float dy = right->y - left->y;
    float pair_len = sqrtf(dx * dx + dy * dy);
    float expected_pair_len = fmaxf(1.0f, layout->sep * 2.0f);
    float pair_scale = app_face_clampf(pair_len / expected_pair_len, 0.32f, 1.20f);

    float u_len = sqrtf(frame.ux * frame.ux + frame.uy * frame.uy);
    if (u_len < 0.18f && pair_len > 1.0f) {
        frame.ux = (dx / pair_len) * pair_scale;
        frame.uy = (dy / pair_len) * pair_scale;
        u_len = pair_scale;
    } else if (u_len < 0.0001f) {
        frame.ux = 1.0f;
        frame.uy = 0.0f;
        u_len = 1.0f;
    }

    float v_len = sqrtf(frame.vx * frame.vx + frame.vy * frame.vy);
    if (v_len < 0.18f) {
        frame.vx = -frame.uy;
        frame.vy = frame.ux;
        v_len = u_len;
    }

    float det = frame.ux * frame.vy - frame.uy * frame.vx;
    if (det < 0.0f) {
        frame.vx = -frame.vx;
        frame.vy = -frame.vy;
    }

    return frame;
}

static void pair_frame_point(const app_face_draw_layout_t *layout,
                             const app_face_pair_frame_t *frame,
                             float local_x,
                             float local_y,
                             float *out_x,
                             float *out_y)
{
    *out_x = layout->cx + frame->x + frame->ux * local_x + frame->vx * local_y;
    *out_y = layout->cy + frame->y + frame->uy * local_x + frame->vy * local_y;
}

static lv_area_t display_dirty_area(const app_face_canvas_t *canvas, const app_face_draw_layout_t *layout)
{
    float display_margin = 4.0f;
    lv_area_t area = {
        .x1 = (lv_coord_t)floorf(layout->display_cx - layout->radius - display_margin),
        .y1 = (lv_coord_t)floorf(layout->display_cy - layout->radius - display_margin),
        .x2 = (lv_coord_t)ceilf(layout->display_cx + layout->radius + display_margin),
        .y2 = (lv_coord_t)ceilf(layout->display_cy + layout->radius + display_margin),
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

static void render_plan_reset(app_face_render_plan_t *plan)
{
    plan->count = 0;
}

static bool render_plan_add(app_face_render_plan_t *plan,
                            app_face_render_primitive_t primitive,
                            const lv_area_t *display_area,
                            int width,
                            int height)
{
    if (plan->count >= APP_FACE_MAX_RENDER_PRIMITIVES ||
        !clip_area_to_area(&primitive.bounds, display_area, width, height)) {
        return false;
    }
    plan->items[plan->count++] = primitive;
    return true;
}

static bool eye_bounds_area(const app_face_canvas_t *canvas,
                            const app_face_draw_layout_t *layout,
                            const app_face_surface_t *surface,
                            int side,
                            const app_face_mood_numbers_t *mood,
                            lv_area_t *out)
{
    area_reset(out, canvas->width, canvas->height);
    if (surface->visibility <= 0.025f) {
        return false;
    }

    float round_eye = app_face_smoothf(app_face_clampf(mood->round_eye, 0.0f, 1.0f));
    float local_h = app_face_lerpf(layout->eye_h, layout->eye_w, round_eye);
    float close = (s_face.micro_squint + fmaxf(0.0f, mood->lid) * 0.12f) * (1.0f - round_eye * 0.65f);
    float smile = app_face_smoothf(app_face_clampf(mood->smile, 0.0f, 1.0f));
    float concave = app_face_smoothf(app_face_clampf(mood->concave_top, 0.0f, 1.0f));
    float expressive_open = app_face_lerpf(mood->open, 1.0f, round_eye * 0.95f);
    float open = app_face_clampf(expressive_open * s_face.blink.value * (1.0f - close * 0.86f), 0.03f, 1.08f);
    bool use_fast_ellipse = round_eye > 0.72f && smile < 0.18f && concave < 0.20f && fabsf(mood->slant) < 0.18f;
    float eye_cx = layout->cx + surface->x;
    float eye_cy = layout->cy + surface->y;
    const float margin = (float)APP_FACE_PRIMITIVE_MARGIN_PX;

    if (use_fast_ellipse) {
        float angle = mood->eye_tilt * 0.55f;
        float c = cosf(angle);
        float s = sinf(angle);
        float ux = surface->ux * c + surface->vx * s;
        float uy = surface->uy * c + surface->vy * s;
        float vx = -surface->ux * s + surface->vx * c;
        float vy = -surface->uy * s + surface->vy * c;
        area_include_transformed_ellipse(out, eye_cx, eye_cy, ux, uy, vx, vy, layout->eye_w * 0.5f, local_h * open * 0.5f, margin);
    } else {
        app_face_vec2_t local[APP_FACE_MAX_POLY_POINTS];
        app_face_vec2_t screen[APP_FACE_MAX_POLY_POINTS];
        size_t n = build_eye_shape(mood, side, layout->eye_w, local_h, close, local, APP_FACE_MAX_POLY_POINTS);
        for (size_t i = 0; i < n; i++) {
            transformed_point(surface, layout->cx, layout->cy, local[i].x, local[i].y, &screen[i]);
            area_include_rect(out, screen[i].x, screen[i].y, screen[i].x, screen[i].y);
        }
    }

    float smile_pupil_fade = 1.0f - app_face_smoothf((mood->smile - 0.24f) / 0.62f);
    if (smile_pupil_fade > 0.05f) {
        float pupil_ratio = 0.15f + mood->pupil_size * 0.22f;
        float pupil_w = layout->eye_w * pupil_ratio;
        float base_pupil_h = local_h * app_face_clampf(app_face_lerpf(mood->open, 1.0f, round_eye * 0.95f), 0.12f, 1.0f) *
                             (0.14f + mood->pupil_size * 0.15f);
        float round_pupil_h = local_h * pupil_ratio * 0.92f;
        float pupil_h = app_face_lerpf(base_pupil_h, round_pupil_h, round_eye);
        float blink_pupil_alpha = app_face_smooth_range(0.055f, 0.20f, open);
        pupil_h = fminf(pupil_h, local_h * open * 0.46f);
        if (pupil_h > 0.55f && blink_pupil_alpha > 0.02f) {
            float px = app_face_clampf(layout->look_x * layout->eye_w * 0.025f + mood->side_eye * layout->eye_w * 0.08f -
                                           side * layout->eye_w * 0.018f,
                                       -layout->eye_w * 0.20f,
                                       layout->eye_w * 0.20f);
            float py = app_face_clampf(layout->look_y * local_h * 0.025f - local_h * 0.016f, -local_h * 0.15f, local_h * 0.15f);
            float pupil_cx = eye_cx + surface->ux * px + surface->vx * py;
            float pupil_cy = eye_cy + surface->uy * px + surface->vy * py;
            area_include_transformed_ellipse(out, pupil_cx, pupil_cy, surface->ux, surface->uy, surface->vx, surface->vy, pupil_w, pupil_h, margin);
            if (mood->sparkle > 0.05f && mood->concave_top < 0.35f) {
                area_include_transformed_ellipse(out,
                                                 eye_cx + surface->ux * (px - side * pupil_w * 0.45f) + surface->vx * (py - pupil_h * 0.35f),
                                                 eye_cy + surface->uy * (px - side * pupil_w * 0.45f) + surface->vy * (py - pupil_h * 0.35f),
                                                 surface->ux,
                                                 surface->uy,
                                                 surface->vx,
                                                 surface->vy,
                                                 fmaxf(1.3f, layout->scale * 0.006f * mood->sparkle),
                                                 fmaxf(1.3f, layout->scale * 0.006f * mood->sparkle),
                                                 margin);
            }
        }
    }

    area_inflate_clip(out, APP_FACE_PRIMITIVE_MARGIN_PX, canvas->width, canvas->height);
    return !area_is_empty(out);
}

static void add_eye_primitive(const app_face_canvas_t *canvas,
                              app_face_render_plan_t *plan,
                              const lv_area_t *display_area,
                              const app_face_draw_layout_t *layout,
                              const app_face_surface_t *surface,
                              int side,
                              const app_face_mood_numbers_t *mood,
                              app_face_rgb_t iris,
                              app_face_rgb_t pupil)
{
    app_face_render_primitive_t primitive = {
        .type = APP_FACE_PRIMITIVE_EYE,
        .surface = *surface,
        .mood = *mood,
        .iris = iris,
        .pupil = pupil,
        .cx = layout->cx,
        .cy = layout->cy,
        .eye_w = layout->eye_w,
        .eye_h = layout->eye_h,
        .look_x = layout->look_x,
        .look_y = layout->look_y,
        .scale = layout->scale,
        .side = side,
    };
    if (!eye_bounds_area(canvas, layout, surface, side, mood, &primitive.bounds)) {
        return;
    }
    render_plan_add(plan, primitive, display_area, canvas->width, canvas->height);
}

static void add_ellipse_primitive(const app_face_canvas_t *canvas,
                                  app_face_render_plan_t *plan,
                                  const lv_area_t *display_area,
                                  float cx,
                                  float cy,
                                  const app_face_pair_frame_t *frame,
                                  float rx,
                                  float ry,
                                  app_face_rgb_t color,
                                  uint8_t alpha)
{
    app_face_render_primitive_t primitive = {
        .type = APP_FACE_PRIMITIVE_TRANSFORMED_ELLIPSE,
        .frame = *frame,
        .color = color,
        .cx = cx,
        .cy = cy,
        .rx = rx,
        .ry = ry,
        .alpha = alpha,
    };
    area_reset(&primitive.bounds, canvas->width, canvas->height);
    area_include_transformed_ellipse(&primitive.bounds, cx, cy, frame->ux, frame->uy, frame->vx, frame->vy, rx, ry, APP_FACE_PRIMITIVE_MARGIN_PX);
    area_inflate_clip(&primitive.bounds, APP_FACE_PRIMITIVE_MARGIN_PX, canvas->width, canvas->height);
    render_plan_add(plan, primitive, display_area, canvas->width, canvas->height);
}

static void build_render_plan(const app_face_canvas_t *canvas,
                              const app_face_mood_numbers_t *mood,
                              app_face_rgb_t iris,
                              app_face_rgb_t pupil,
                              const app_face_energy_profile_t *energy,
                              const app_face_draw_layout_t *layout,
                              app_face_render_plan_t *plan)
{
    render_plan_reset(plan);
    lv_area_t display_area = display_dirty_area(canvas, layout);
    const app_face_surface_t *left = &layout->left;
    const app_face_surface_t *right = &layout->right;
    if (left->z <= right->z) {
        add_eye_primitive(canvas, plan, &display_area, layout, left, -1, mood, iris, pupil);
        add_eye_primitive(canvas, plan, &display_area, layout, right, 1, mood, iris, pupil);
    } else {
        add_eye_primitive(canvas, plan, &display_area, layout, right, 1, mood, iris, pupil);
        add_eye_primitive(canvas, plan, &display_area, layout, left, -1, mood, iris, pupil);
    }

    if (mood->think_dots > 0.05f) {
        app_face_pair_frame_t frame = eye_pair_frame(layout);
        float alpha = (0.46f + 0.16f * energy->lively) * mood->think_dots * frame.visibility;
        float dot_y_base = -layout->local_h * 1.08f;
        float dot_step = layout->sep * 0.42f;
        for (int i = 0; i < 3; i++) {
            float phase = s_face.time_s * (2.4f + energy->raw * 2.2f) + (float)i * 1.1f;
            float local_x = ((float)i - 1.0f) * dot_step;
            float local_y = dot_y_base + sinf(phase) * layout->radius * 0.015f;
            float dot_x;
            float dot_y;
            float dot_r = layout->radius * (0.025f + (float)i * 0.004f);
            pair_frame_point(layout, &frame, local_x, local_y, &dot_x, &dot_y);
            add_ellipse_primitive(canvas, plan, &display_area, dot_x, dot_y, &frame, dot_r, dot_r, iris, alpha_from_float(alpha));
        }
    }

    if (mood->speaking_bars > 0.08f) {
        app_face_pair_frame_t frame = eye_pair_frame(layout);
        float total = layout->sep * 1.36f;
        float start = -total * 0.5f;
        float local_y = layout->local_h * 1.28f;
        float alpha = (0.26f + 0.38f * energy->lively) * mood->speaking_bars * frame.visibility;
        for (int i = 0; i < 7; i++) {
            float local_x = start + (float)i * (total / 6.0f);
            float amp = fabsf(sinf(s_face.time_s * (7.0f + energy->raw * 18.0f) + (float)i * 0.72f)) *
                            layout->radius * (0.05f + 0.13f * energy->lively) +
                        layout->radius * 0.02f;
            float bar_x;
            float bar_y;
            pair_frame_point(layout, &frame, local_x, local_y, &bar_x, &bar_y);
            add_ellipse_primitive(canvas,
                                  plan,
                                  &display_area,
                                  bar_x,
                                  bar_y,
                                  &frame,
                                  layout->radius * 0.018f,
                                  amp * 0.5f,
                                  iris,
                                  alpha_from_float(alpha));
        }
    }
}

static void estimate_dynamic_list_from_plan(const app_face_canvas_t *canvas,
                                            const app_face_render_plan_t *plan,
                                            app_face_dirty_list_t *list)
{
    dirty_list_reset(list);
    for (size_t i = 0; i < plan->count; i++) {
        dirty_list_add_area(list, plan->items[i].bounds, canvas->width, canvas->height);
    }
}

static void draw_render_plan(app_face_canvas_t *canvas,
                             const app_face_render_plan_t *plan,
                             const lv_area_t *dirty_area)
{
    for (size_t i = 0; i < plan->count; i++) {
        const app_face_render_primitive_t *primitive = &plan->items[i];
        if (dirty_area && !area_intersects(dirty_area, &primitive->bounds)) {
            continue;
        }
        if (primitive->type == APP_FACE_PRIMITIVE_EYE) {
            draw_eye(canvas,
                     primitive->cx,
                     primitive->cy,
                     &primitive->surface,
                     primitive->side,
                     &primitive->mood,
                     primitive->iris,
                     primitive->pupil,
                     primitive->eye_w,
                     primitive->eye_h,
                     primitive->look_x,
                     primitive->look_y,
                     primitive->scale);
        } else {
            app_face_canvas_fill_transformed_ellipse(canvas,
                                                     primitive->cx,
                                                     primitive->cy,
                                                     primitive->frame.ux,
                                                     primitive->frame.uy,
                                                     primitive->frame.vx,
                                                     primitive->frame.vy,
                                                     primitive->rx,
                                                     primitive->ry,
                                                     primitive->color,
                                                     primitive->alpha);
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
                                            uint32_t *pack_time_us,
                                            uint32_t *lcd_chunk_count,
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
        int64_t pack_start = esp_timer_get_time();
        for (int row = 0; row < rows; row++) {
            memcpy(&tx[row * width],
                   &s_face.pixels[(y + row) * s_face.canvas.width + area->x1],
                   row_bytes);
        }
        if (pack_time_us) {
            *pack_time_us += (uint32_t)(esp_timer_get_time() - pack_start);
        }

        if (packed_bytes) {
            *packed_bytes += (uint32_t)(row_bytes * (size_t)rows);
        }
        if (lcd_chunk_count) {
            (*lcd_chunk_count)++;
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
                                 uint32_t *pack_time_us,
                                 uint32_t *lcd_chunk_count,
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
        esp_err_t err = blit_area_from_framebuffer(&fragment, packed_bytes, pack_time_us, lcd_chunk_count, lcd_wait_us);
        if (err != ESP_OK) {
            final_err = err;
        }
    }
    return final_err;
}

static void update_stats(uint32_t now_ms,
                         uint32_t render_time_us,
                         uint32_t restore_time_us,
                         uint32_t draw_time_us,
                         uint32_t pack_time_us,
                         uint32_t dirty_pixels,
                         uint32_t dirty_rect_count,
                         uint32_t lcd_chunk_count,
                         uint32_t packed_bytes,
                         uint32_t lcd_wait_us,
                         bool full_refresh,
                         bool blitted)
{
    s_face.stats.target_fps = CONFIG_APP_FACE_FPS;
    s_face.stats.render_time_us = render_time_us;
    s_face.stats.restore_time_us = restore_time_us;
    s_face.stats.draw_time_us = draw_time_us;
    s_face.stats.pack_time_us = pack_time_us;
    s_face.stats.dirty_pixels = dirty_pixels;
    s_face.stats.dirty_rect_count = dirty_rect_count;
    s_face.stats.lcd_chunk_count = lcd_chunk_count;
    s_face.stats.packed_bytes = packed_bytes;
    s_face.stats.lcd_wait_time_us = lcd_wait_us;
    if (full_refresh) {
        s_face.stats.full_refresh_count++;
    }

    s_face.stats_rendered_frames++;
    if (blitted) {
        s_face.stats_blit_frames++;
    }
    s_face.stats_dirty_pixels += dirty_pixels;
    s_face.stats_packed_bytes += packed_bytes;
    s_face.stats_dirty_rects += dirty_rect_count;
    s_face.stats_lcd_chunks += lcd_chunk_count;
    s_face.stats_restore_time_us += restore_time_us;
    s_face.stats_draw_time_us += draw_time_us;
    s_face.stats_pack_time_us += pack_time_us;
    s_face.stats_lcd_wait_time_us += lcd_wait_us;
    if (full_refresh) {
        s_face.stats_full_refreshes++;
    }

    uint32_t elapsed = elapsed_ms(now_ms, s_face.stats_window_start_ms);
    if (elapsed >= 1000u) {
        s_face.stats.rendered_fps = (s_face.stats_rendered_frames * 1000u) / elapsed;
        s_face.stats.blit_fps = (s_face.stats_blit_frames * 1000u) / elapsed;
        s_face.stats.skipped_frames = s_face.stats_skipped_frames;
        s_face.stats.dirty_pixels = s_face.stats_dirty_pixels;
        s_face.stats.packed_bytes = s_face.stats_packed_bytes;
        s_face.stats.dirty_rect_count = s_face.stats_dirty_rects;
        s_face.stats.lcd_chunk_count = s_face.stats_lcd_chunks;
        s_face.stats.restore_time_us = s_face.stats_restore_time_us;
        s_face.stats.draw_time_us = s_face.stats_draw_time_us;
        s_face.stats.pack_time_us = s_face.stats_pack_time_us;
        s_face.stats.lcd_wait_time_us = s_face.stats_lcd_wait_time_us;
        s_face.stats.full_refresh_count = s_face.stats_full_refreshes;
#if CONFIG_APP_FACE_PROFILE
        ESP_LOGI(TAG,
                 "face target=%lu render=%lu blit=%lu render_us=%lu restore_us=%lu draw_us=%lu pack_us=%lu dirty_px=%lu dirty_rects=%lu chunks=%lu packed=%lu wait_us=%lu full=%lu skipped=%lu",
                 (unsigned long)s_face.stats.target_fps,
                 (unsigned long)s_face.stats.rendered_fps,
                 (unsigned long)s_face.stats.blit_fps,
                 (unsigned long)s_face.stats.render_time_us,
                 (unsigned long)s_face.stats.restore_time_us,
                 (unsigned long)s_face.stats.draw_time_us,
                 (unsigned long)s_face.stats.pack_time_us,
                 (unsigned long)s_face.stats.dirty_pixels,
                 (unsigned long)s_face.stats.dirty_rect_count,
                 (unsigned long)s_face.stats.lcd_chunk_count,
                 (unsigned long)s_face.stats.packed_bytes,
                 (unsigned long)s_face.stats.lcd_wait_time_us,
                 (unsigned long)s_face.stats.full_refresh_count,
                 (unsigned long)s_face.stats.skipped_frames);
#endif
        s_face.stats_window_start_ms = now_ms;
        s_face.stats_rendered_frames = 0;
        s_face.stats_blit_frames = 0;
        s_face.stats_skipped_frames = 0;
        s_face.stats_dirty_pixels = 0;
        s_face.stats_packed_bytes = 0;
        s_face.stats_dirty_rects = 0;
        s_face.stats_lcd_chunks = 0;
        s_face.stats_restore_time_us = 0;
        s_face.stats_draw_time_us = 0;
        s_face.stats_pack_time_us = 0;
        s_face.stats_lcd_wait_time_us = 0;
        s_face.stats_full_refreshes = 0;
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

    app_face_render_plan_t render_plan;
    build_render_plan(&s_face.canvas, &mood, iris, pupil, &energy, &layout, &render_plan);

    app_face_dirty_list_t current_list;
    estimate_dynamic_list_from_plan(&s_face.canvas, &render_plan, &current_list);

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
    uint32_t pack_time_us = 0;
    uint32_t lcd_chunk_count = 0;
    uint32_t lcd_wait_us = 0;
    uint32_t restore_time_us = 0;
    uint32_t draw_time_us = 0;
    bool blitted = false;

    if (s_face.base_pixels) {
        for (size_t i = 0; i < dirty_list.count; i++) {
            lv_area_t area = dirty_list.rects[i];
            if (!area_clip_to_bounds(&area, s_face.canvas.width, s_face.canvas.height)) {
                continue;
            }
            int64_t restore_start = esp_timer_get_time();
            app_face_canvas_restore_region(&s_face.canvas,
                                           &s_face.base_canvas,
                                           area.x1,
                                           area.y1,
                                           area.x2,
                                           area.y2);
            restore_time_us += (uint32_t)(esp_timer_get_time() - restore_start);
            int64_t draw_start = esp_timer_get_time();
            app_face_canvas_set_clip_rect(&s_face.canvas, area.x1, area.y1, area.x2, area.y2);
            app_face_canvas_set_clip_circle(&s_face.canvas, layout.display_cx, layout.display_cy, layout.radius - 2.0f);
            draw_render_plan(&s_face.canvas, &render_plan, &area);
            app_face_canvas_clear_clip(&s_face.canvas);
            draw_time_us += (uint32_t)(esp_timer_get_time() - draw_start);
        }
    } else {
        app_face_canvas_clear(&s_face.canvas, (app_face_rgb_t){.r = 1, .g = 2, .b = 5});
        int64_t draw_start = esp_timer_get_time();
        app_face_canvas_set_clip_circle(&s_face.canvas, layout.display_cx, layout.display_cy, layout.radius - 2.0f);
        draw_render_plan(&s_face.canvas, &render_plan, NULL);
        app_face_canvas_clear_clip(&s_face.canvas);
        draw_time_us += (uint32_t)(esp_timer_get_time() - draw_start);
    }

    for (size_t i = 0; i < dirty_list.count; i++) {
        esp_err_t err = blit_dirty_area(&dirty_list.rects[i],
                                        &dirty_pixels,
                                        &packed_bytes,
                                        &pack_time_us,
                                        &lcd_chunk_count,
                                        &lcd_wait_us);
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
    update_stats(now_ms,
                 render_time_us,
                 restore_time_us,
                 draw_time_us,
                 pack_time_us,
                 dirty_pixels,
                 (uint32_t)dirty_list.count,
                 lcd_chunk_count,
                 packed_bytes,
                 lcd_wait_us,
                 full_refresh,
                 blitted);
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
    s_face.energy = 0.21f;
    s_face.target_energy = 0.21f;
    s_face.blink.value = 1.0f;
    s_face.last_frame_us = (uint64_t)esp_timer_get_time();
    s_face.last_frame_ms = (uint32_t)(s_face.last_frame_us / 1000ULL);
    s_face.rng = 0xA511E9E5u ^ s_face.last_frame_ms;
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
    apply_speaking_energy_cap(mode);
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
    float input = app_face_clampf(energy_0_to_1, 0.0f, 1.0f);
    if (s_face.target_mode == APP_FACE_SPEAKING) {
        input = fminf(input, APP_FACE_SPEAKING_INPUT_ENERGY_CAP);
    }
    s_face.target_energy = app_face_clampf(input * APP_FACE_INPUT_ENERGY_SCALE, 0.0f, 1.0f);
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
    (void)now_ms;
    if (!s_face.initialized) {
        return;
    }
    uint32_t fps = CONFIG_APP_FACE_FPS > 0 ? CONFIG_APP_FACE_FPS : 30u;
    uint64_t frame_period_us = 1000000ULL / (uint64_t)fps;
    uint64_t frame_now_us = (uint64_t)esp_timer_get_time();
    uint64_t frame_elapsed_us = frame_now_us - s_face.last_frame_us;
    if (frame_elapsed_us < frame_period_us) {
        return;
    }
    if (frame_elapsed_us >= frame_period_us * 2ULL) {
        s_face.stats_skipped_frames += (uint32_t)(frame_elapsed_us / frame_period_us) - 1u;
    }
    s_face.last_frame_us = frame_now_us;
    s_face.last_frame_ms = (uint32_t)(frame_now_us / 1000ULL);
    update_runtime(s_face.last_frame_ms);
    render(s_face.last_frame_ms);
}

#endif
