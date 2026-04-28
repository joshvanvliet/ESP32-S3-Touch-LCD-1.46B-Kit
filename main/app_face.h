#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_face_config.h"
#include "esp_err.h"
#include "lvgl.h"

typedef struct {
    uint32_t target_fps;
    uint32_t rendered_fps;
    uint32_t blit_fps;
    uint32_t render_time_us;
    uint32_t restore_time_us;
    uint32_t draw_time_us;
    uint32_t pack_time_us;
    uint32_t dirty_pixels;
    uint32_t dirty_rect_count;
    uint32_t lcd_chunk_count;
    uint32_t packed_bytes;
    uint32_t lcd_wait_time_us;
    uint32_t full_refresh_count;
    uint32_t skipped_frames;
} app_face_stats_t;

esp_err_t app_face_init(lv_obj_t *parent);
lv_obj_t *app_face_get_obj(void);
void app_face_set_mode(app_face_mode_t mode);
void app_face_pulse_mode(app_face_mode_t mode, uint32_t duration_ms);
void app_face_set_energy(float energy_0_to_1);
void app_face_set_axes(float axis_x, float axis_y, float axis_z);
void app_face_force_blink(void);
void app_face_tap(void);
void app_face_tick(uint32_t now_ms);
void app_face_set_protected_areas(const lv_area_t *areas, size_t count);
void app_face_request_full_refresh(void);
bool app_face_get_stats(app_face_stats_t *out_stats);
