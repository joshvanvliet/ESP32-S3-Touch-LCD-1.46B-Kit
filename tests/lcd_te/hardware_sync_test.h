/* Optional board check. Call after LCD_Init(), before LVGL/audio startup. */
#pragma once
#include <assert.h>
#include "app_lcd.h"
#include "Display_SPD2010.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "app_face_blit.h"
#include "esp_heap_caps.h"
#include "esp_pm.h"

static void lcd_te_full_frame_test(void)
{
    const int width = EXAMPLE_LCD_WIDTH, height = EXAMPLE_LCD_HEIGHT;
    lv_color_t *frame = heap_caps_malloc(width * height * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    lv_color_t *buffers[2];
    for (int i = 0; i < 2; ++i) buffers[i] = heap_caps_malloc(CONFIG_APP_FACE_TX_BUFFER_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    assert(frame && buffers[0] && buffers[1]);
    uint8_t index = 0;
    app_face_blit_context_t ctx = {
        .framebuffer = frame, .framebuffer_width = width, .framebuffer_height = height,
        .tx_buffers = buffers, .tx_index = &index,
        .tx_buffer_bytes = CONFIG_APP_FACE_TX_BUFFER_BYTES, .tx_buffer_count = 2,
    };
    const lv_area_t area = {0, 0, width - 1, height - 1};
    esp_pm_lock_handle_t cpu;
    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "te_test", &cpu));
    ESP_ERROR_CHECK(esp_pm_lock_acquire(cpu));
    UBaseType_t priority = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, 6);
    int64_t total = 0, maximum = 0;
    for (int n = 0; n < 60; ++n) {
        /* Prepared frames move a bright stripe across the entire display. */
        for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x)
                frame[y * width + x] = lv_color_make((x + n * 13) % width < 60 ? 255 : 0, y * 255 / height, 64);
        ESP_ERROR_CHECK(app_lcd_wait_frame_start(34));
        int64_t start = esp_timer_get_time();
        ESP_ERROR_CHECK(app_face_blit_dirty_area(&ctx, &area, NULL));
        ESP_ERROR_CHECK(app_lcd_finish_frame(34));
        int64_t elapsed = esp_timer_get_time() - start;
        total += elapsed;
        if (elapsed > maximum) maximum = elapsed;
    }
    vTaskPrioritySet(NULL, priority);
    ESP_ERROR_CHECK(esp_pm_lock_release(cpu));
    ESP_ERROR_CHECK(esp_pm_lock_delete(cpu));
    heap_caps_free(frame);
    for (int i = 0; i < 2; ++i) heap_caps_free(buffers[i]);
    ESP_LOGI("TE_TEST", "Full screen: frames=60 transfer_avg_us=%lld transfer_max_us=%lld", total / 60, maximum);
}

static void lcd_te_hardware_test(void)
{
    ESP_ERROR_CHECK(app_lcd_wait_frame_start(34));
    int64_t first = esp_timer_get_time();
    ESP_ERROR_CHECK(app_lcd_wait_frame_start(34));
    int64_t period = esp_timer_get_time() - first;
    assert(period > 14000 && period < 20000);
    ESP_ERROR_CHECK(gpio_intr_disable(ESP_PANEL_LCD_SPI_IO_TE));
    first = esp_timer_get_time();
    esp_err_t missing = app_lcd_wait_frame_start(34);
    int64_t timeout = esp_timer_get_time() - first;
    ESP_ERROR_CHECK(gpio_intr_enable(ESP_PANEL_LCD_SPI_IO_TE));
    assert(missing == ESP_ERR_TIMEOUT && timeout < 50000);
    ESP_ERROR_CHECK(app_lcd_wait_frame_start(34));
    ESP_LOGI("TE_TEST", "Fresh edges, missing-signal timeout and recovery passed: period=%lld timeout=%lld",
             period, timeout);
    lcd_te_full_frame_test();
}
