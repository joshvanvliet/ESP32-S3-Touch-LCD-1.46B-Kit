/* Opt-in board benchmark: include in main.c and call lcd_transport_benchmark()
 * after LCD/LVGL initialization, before app_state_init(). Remove the call after
 * measuring. Draws a color gradient; does not modify flash or persisted state. */
#include "app_face_blit.h"
#include "app_lcd.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#if CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

static void lcd_transport_benchmark(void)
{
    const int width = EXAMPLE_LCD_WIDTH, height = EXAMPLE_LCD_HEIGHT;
    lv_color_t *frame = heap_caps_malloc(width * height * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    lv_color_t *storage = heap_caps_malloc(11536, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!frame || !storage) {
        ESP_LOGE("LCD_BENCH", "Insufficient memory for benchmark");
        heap_caps_free(frame);
        heap_caps_free(storage);
        return;
    }
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            frame[y * width + x] = lv_color_make(x * 255 / width, y * 255 / height, (x ^ y) & 255);
        }
    }
    const lv_area_t areas[] = {{0, 0, width - 1, height - 1}, {80, 100, 331, 235}};
#if CONFIG_PM_ENABLE
    esp_pm_lock_handle_t cpu_lock;
    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "lcd_bench", &cpu_lock));
    ESP_ERROR_CHECK(esp_pm_lock_acquire(cpu_lock));
#endif
    const int counts[] = {1, 2, 1, 2, 2};
    const int sizes[] = {8192, 4096, 8240, 4120, 5768};
    for (int variant = 0; variant < 5; variant++) {
        int count = counts[variant], size = sizes[variant];
        lv_color_t *buffers[] = {storage, storage + size / sizeof(lv_color_t)};
        uint8_t index = 0;
        app_face_blit_context_t ctx = {
            .framebuffer = frame, .framebuffer_width = width, .framebuffer_height = height,
            .tx_buffers = buffers, .tx_index = &index,
            .tx_buffer_bytes = size, .tx_buffer_count = count,
        };
        for (int area = 0; area < 2; area++) {
            app_face_blit_stats_t stats = {0};
            ESP_ERROR_CHECK(app_lcd_wait_idle(UINT32_MAX));
            int64_t start = esp_timer_get_time();
            for (int frame_index = 0; frame_index < 30; frame_index++) {
                ESP_ERROR_CHECK(app_face_blit_dirty_area(&ctx, &areas[area], &stats));
                ESP_ERROR_CHECK(app_lcd_wait_idle(UINT32_MAX));
                /* Yield to other ready tasks between frames. */
                taskYIELD();
            }
            int64_t elapsed = esp_timer_get_time() - start;
            ESP_LOGI("LCD_BENCH", "buffers=%d bytes=%d area=%d frames=30 total_us=%lld pack_us=%lu wait_us=%lu chunks=%lu",
                     count, size, area, (long long)elapsed,
                     (unsigned long)stats.pack_time_us, (unsigned long)stats.lcd_wait_time_us,
                     (unsigned long)stats.lcd_chunk_count);
        }
    }
#if CONFIG_PM_ENABLE
    ESP_ERROR_CHECK(esp_pm_lock_release(cpu_lock));
    ESP_ERROR_CHECK(esp_pm_lock_delete(cpu_lock));
#endif
    heap_caps_free(storage);
    heap_caps_free(frame);
}
