#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "app_face_blit.h"
#include "app_lcd.h"
#include "app_face_dirty.h"

#define WIDTH 412
#define HEIGHT 412
int64_t test_time_us;
static lv_color_t framebuffer[WIDTH * HEIGHT], panel[WIDTH * HEIGHT];
static lv_color_t buffers[2][32768 / sizeof(lv_color_t)];
static lv_color_t snapshot[32768 / sizeof(lv_color_t)];
static const lv_color_t *pending;
static int px, py, pw, ph;
static bool fail_submit;

esp_err_t app_lcd_wait_idle(uint32_t timeout)
{
    (void)timeout;
    if (pending) {
        assert(memcmp(pending, snapshot, pw * ph * sizeof(*pending)) == 0);
        for (int y = 0; y < ph; y++) {
            memcpy(panel + (py + y) * WIDTH + px, pending + y * pw, pw * sizeof(*pending));
        }
        pending = NULL;
    }
    return ESP_OK;
}

esp_err_t app_lcd_blit_rect_async(int x, int y, int w, int h, const void *data,
                                app_lcd_blit_done_cb_t cb, void *user)
{
    app_lcd_wait_idle(UINT32_MAX);
    if (fail_submit) {
        fail_submit = false;
        return ESP_ERR_TIMEOUT;
    }
    assert(x >= 0 && y >= 0 && x + w <= WIDTH && y + h <= HEIGHT);
    assert(w > 0 && h > 0 && w * h * sizeof(lv_color_t) <= sizeof(snapshot));
    assert((x % 4) == 0 && (w % 4) == 0);
    pending = data; px = x; py = y; pw = w; ph = h;
    memcpy(snapshot, data, w * h * sizeof(*pending));
    (void)cb; (void)user;
    return ESP_OK;
}

int main(void)
{
    app_face_dirty_list_t dirty = {0};
    const lv_area_t horizontal = {0, 0, 199, 39};
    const lv_area_t vertical = {180, 32, 219, 231};
    app_face_dirty_list_add_area(&dirty, horizontal, WIDTH, HEIGHT);
    app_face_dirty_list_add_area(&dirty, vertical, WIDTH, HEIGHT);
    assert(dirty.count == 2); /* Avoid repainting the empty corner of an L. */
    app_face_dirty_list_add_area(&dirty, horizontal, WIDTH, HEIGHT);
    assert(dirty.count == 2); /* Repeated old/new bounds still merge. */
    app_face_dirty_list_reset(&dirty);
    app_face_dirty_list_add_area(&dirty, (lv_area_t){0, 0, 99, 99}, WIDTH, HEIGHT);
    app_face_dirty_list_add_area(&dirty, (lv_area_t){100, 0, 199, 99}, WIDTH, HEIGHT);
    assert(dirty.count == 1 && app_face_area_pixel_count(&dirty.rects[0]) == 20000);
    lv_color_t *tx[] = {buffers[0], buffers[1]};
    /* Includes protected status text and fragments narrower than a row. */
    const lv_area_t protected = {152, 30, 259, 49};
    const lv_area_t full = {0, 0, WIDTH - 1, HEIGHT - 1};
    const lv_area_t small = {48, 19, 303, 155};
    const int sizes[] = {4096, 4120, 5768, 8192, 8240, 16384, 32768};
    for (int count = 1; count <= 2; count++) {
        for (size_t variant = 0; variant < sizeof(sizes) / sizeof(sizes[0]); variant++) {
            int size = sizes[variant];
            uint8_t index = 0;
            app_face_blit_context_t ctx = {
                .framebuffer = framebuffer, .framebuffer_width = WIDTH,
                .framebuffer_height = HEIGHT, .tx_buffers = tx, .tx_index = &index,
                .tx_buffer_bytes = size, .tx_buffer_count = count,
                .protected_areas = &protected, .protected_area_count = 1,
            };
            app_face_blit_stats_t stats = {0};
            for (int frame = 0; frame < 300; frame++) {
                app_lcd_wait_idle(UINT32_MAX);
                memset(panel, 0, sizeof(panel));
                for (size_t i = 0; i < WIDTH * HEIGHT; i++) {
                    framebuffer[i].full = (uint16_t)(i * 131u + frame * 17u);
                }
                const lv_area_t *area = frame % 7 ? &small : &full;
                assert(app_face_blit_dirty_area(&ctx, area, &stats) == ESP_OK);
                /* Complete the previous DMA only when the next submission
                 * waits, including across separate dirty-area calls. */
                assert(app_face_blit_dirty_area(&ctx, area, &stats) == ESP_OK);
                app_lcd_wait_idle(UINT32_MAX);
                for (int y = 0; y < HEIGHT; y++) {
                    for (int x = 0; x < WIDTH; x++) {
                        bool inside = x >= area->x1 && x <= area->x2 && y >= area->y1 && y <= area->y2;
                        /* Margin=2, then outward alignment to 4 pixels. */
                        bool excluded = x >= 148 && x <= 263 && y >= 28 && y <= 51;
                        uint16_t expected = inside && !excluded ? framebuffer[y * WIDTH + x].full : 0;
                        assert(panel[y * WIDTH + x].full == expected);
                    }
                }
            }
            uint8_t before = index;
            fail_submit = true;
            assert(app_face_blit_dirty_area(&ctx, &small, NULL) == ESP_ERR_TIMEOUT);
            app_lcd_wait_idle(UINT32_MAX);
            /* A failed first fragment must not advance to a live buffer. */
            assert(index == before);
            assert(app_face_blit_dirty_area(&ctx, &full, NULL) == ESP_OK);
            app_lcd_wait_idle(UINT32_MAX);
            ctx.tx_buffer_bytes = 2;
            assert(app_face_blit_dirty_area(&ctx, &full, NULL) == ESP_ERR_INVALID_ARG);
            printf("%d buffer(s), %d bytes: pixel coverage and DMA ownership passed\n", count, size);
        }
    }
    assert(app_face_blit_dirty_area(NULL, &full, NULL) == ESP_ERR_INVALID_ARG);
    return 0;
}
