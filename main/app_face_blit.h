#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "lvgl.h"

typedef struct {
    uint32_t dirty_pixels;
    uint32_t packed_bytes;
    uint32_t pack_time_us;
    uint32_t lcd_chunk_count;
    uint32_t lcd_wait_time_us;
} app_face_blit_stats_t;

typedef struct {
    const lv_color_t *framebuffer;
    int framebuffer_width;
    int framebuffer_height;
    lv_color_t **tx_buffers;
    uint8_t *tx_index;
    size_t tx_buffer_bytes;
    size_t tx_buffer_count;
    const lv_area_t *protected_areas;
    size_t protected_area_count;
} app_face_blit_context_t;

esp_err_t app_face_blit_dirty_area(const app_face_blit_context_t *ctx,
                                   const lv_area_t *area,
                                   app_face_blit_stats_t *stats);
