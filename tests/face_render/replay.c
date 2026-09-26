/* Compile against the real renderer. Private access fixes animation inputs for
 * repeatable framebuffer comparisons without adding a production test API. */
#include <assert.h>
#include <stdio.h>
#include "app_face.c"
#include "app_lcd.h"

int64_t test_time_us = 1000000;
static lv_color_t panel[412 * 412];
static lv_color_t pending_copy[412 * 412];
static const lv_color_t *pending;
static int pending_x, pending_y, pending_w, pending_h;
static uint64_t chunks, bytes;

esp_err_t app_lcd_wait_idle(uint32_t timeout_ms)
{
    (void)timeout_ms;
    if (pending) {
        /* DMA still owns this memory until completion. Catch early reuse. */
        assert(memcmp(pending, pending_copy, pending_w * pending_h * sizeof(*pending)) == 0);
        for (int y = 0; y < pending_h; ++y) {
            memcpy(panel + (pending_y + y) * 412 + pending_x,
                   pending + y * pending_w, pending_w * sizeof(*pending));
        }
        pending = NULL;
    }
    return ESP_OK;
}

esp_err_t app_lcd_wait_frame_start(uint32_t timeout_ms)
{
    return app_lcd_wait_idle(timeout_ms);
}
esp_err_t app_lcd_finish_frame(uint32_t timeout_ms)
{
    return app_lcd_wait_idle(timeout_ms);
}

esp_err_t app_lcd_blit_rect_async(int x, int y, int w, int h,
                                const void *data, app_lcd_blit_done_cb_t cb, void *user)
{
    app_lcd_wait_idle(UINT32_MAX);
    assert(x >= 0 && y >= 0 && x + w <= 412 && y + h <= 412);
    assert(w > 0 && h > 0);
    pending = data;
    pending_x = x; pending_y = y; pending_w = w; pending_h = h;
    memcpy(pending_copy, data, w * h * sizeof(*pending));
    ++chunks;
    bytes += w * h * sizeof(*pending);
    (void)cb; (void)user;
    return ESP_OK;
}

static uint64_t hash_pixels(const lv_color_t *pixels)
{
    uint64_t h = UINT64_C(14695981039346656037);
    for (size_t i = 0; i < 412 * 412; ++i) {
        h = (h ^ (pixels[i].full & 255)) * UINT64_C(1099511628211);
        h = (h ^ (pixels[i].full >> 8)) * UINT64_C(1099511628211);
    }
    return h;
}

int main(void)
{
    assert(app_face_init(NULL) == ESP_OK);
    app_lcd_wait_idle(UINT32_MAX);
    s_face.rng = 12345;
    /* Include status/passkey protection, full refreshes, previous dirty regions,
     * transitions, blinking, partial alpha, extreme tilt, and buffer wraparound. */
    lv_area_t protection[] = {{128, 20, 283, 39}, {152, 370, 259, 387}};
    app_face_set_protected_areas(protection, 2);
    for (int mode = 0; mode < APP_FACE_MODE_COUNT; ++mode) {
        app_face_set_mode((app_face_mode_t)mode);
        for (int frame = 0; frame < 100; ++frame) {
            test_time_us += 17000;
            app_face_set_axes(sinf(frame * 0.13f) * 0.95f,
                              cosf(frame * 0.11f) * 0.95f,
                              sinf(frame * 0.07f));
            app_face_set_energy((frame % 31) / 30.0f);
            if (frame == 40) app_face_force_blink();
            if (frame == 70) app_face_request_full_refresh();
            update_runtime((uint32_t)(test_time_us / 1000));
            render((uint32_t)(test_time_us / 1000));
            app_lcd_wait_idle(UINT32_MAX);
            printf("%d %d %016llx %016llx\n", mode, frame,
                   (unsigned long long)hash_pixels(s_face.pixels),
                   (unsigned long long)hash_pixels(panel));
        }
    }
    fprintf(stderr, "1100 frames, %llu transfers, %llu bytes; DMA ownership OK\n",
            (unsigned long long)chunks, (unsigned long long)bytes);
    return 0;
}
