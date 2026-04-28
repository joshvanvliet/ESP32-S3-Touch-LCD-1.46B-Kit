#include "app_face_blit.h"

#include <string.h>

#include "app_face_dirty.h"
#include "app_lcd.h"
#include "esp_timer.h"

static esp_err_t blit_area_from_framebuffer(const app_face_blit_context_t *ctx,
                                            const lv_area_t *area,
                                            app_face_blit_stats_t *stats)
{
    if (!ctx || area_is_empty(area) || !ctx->framebuffer || !ctx->tx_buffers || !ctx->tx_buffers[0]) {
        return ESP_OK;
    }

    const int width = area_width(area);
    if (width <= 0 || area_height(area) <= 0) {
        return ESP_OK;
    }

    const int pixels_per_buffer = (int)(ctx->tx_buffer_bytes / sizeof(lv_color_t));
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
        if (stats) {
            stats->lcd_wait_time_us += (uint32_t)(esp_timer_get_time() - wait_start);
        }
        if (err != ESP_OK) {
            return err;
        }

        uint8_t tx_index = ctx->tx_index ? *ctx->tx_index : 0;
        lv_color_t *tx = ctx->tx_buffers[tx_index % ctx->tx_buffer_count];
        if (ctx->tx_index) {
            (*ctx->tx_index)++;
        }

        size_t row_bytes = (size_t)width * sizeof(lv_color_t);
        int64_t pack_start = esp_timer_get_time();
        for (int row = 0; row < rows; row++) {
            memcpy(&tx[row * width],
                   &ctx->framebuffer[(y + row) * ctx->framebuffer_width + area->x1],
                   row_bytes);
        }
        if (stats) {
            stats->pack_time_us += (uint32_t)(esp_timer_get_time() - pack_start);
            stats->packed_bytes += (uint32_t)(row_bytes * (size_t)rows);
            stats->lcd_chunk_count++;
        }

        err = app_lcd_blit_rect_async(area->x1, y, width, rows, tx, NULL, NULL);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

esp_err_t app_face_blit_dirty_area(const app_face_blit_context_t *ctx,
                                   const lv_area_t *area,
                                   app_face_blit_stats_t *stats)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    lv_area_t fragments[APP_FACE_MAX_BLIT_FRAGMENTS];
    size_t count = app_face_dirty_subtract_protected_areas(area,
                                                           ctx->protected_areas,
                                                           ctx->protected_area_count,
                                                           2,
                                                           ctx->framebuffer_width,
                                                           ctx->framebuffer_height,
                                                           fragments,
                                                           APP_FACE_MAX_BLIT_FRAGMENTS);
    esp_err_t final_err = ESP_OK;
    for (size_t i = 0; i < count; i++) {
        lv_area_t fragment = fragments[i];
        if (!area_clip_to_bounds(&fragment, ctx->framebuffer_width, ctx->framebuffer_height)) {
            continue;
        }
        if (stats) {
            stats->dirty_pixels += area_pixel_count(&fragment);
        }
        esp_err_t err = blit_area_from_framebuffer(ctx, &fragment, stats);
        if (err != ESP_OK) {
            final_err = err;
        }
    }
    return final_err;
}
