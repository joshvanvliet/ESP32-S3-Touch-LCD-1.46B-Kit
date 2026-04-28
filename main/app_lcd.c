#include "app_lcd.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "portmacro.h"

static esp_lcd_panel_handle_t s_panel;
static SemaphoreHandle_t s_idle_sem;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static app_lcd_blit_done_cb_t s_done_cb;
static void *s_done_user;
static bool s_transfer_active;

esp_err_t app_lcd_init(esp_lcd_panel_handle_t panel)
{
    s_panel = panel;
    if (!s_idle_sem) {
        s_idle_sem = xSemaphoreCreateBinary();
        if (!s_idle_sem) {
            return ESP_ERR_NO_MEM;
        }
        xSemaphoreGive(s_idle_sem);
    }
    return ESP_OK;
}

bool app_lcd_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                              esp_lcd_panel_io_event_data_t *edata,
                              void *user_ctx)
{
    (void)panel_io;
    (void)edata;
    (void)user_ctx;

    app_lcd_blit_done_cb_t done_cb = NULL;
    void *done_user = NULL;
    portENTER_CRITICAL_ISR(&s_lock);
    if (s_transfer_active) {
        done_cb = s_done_cb;
        done_user = s_done_user;
        s_done_cb = NULL;
        s_done_user = NULL;
        s_transfer_active = false;
    }
    portEXIT_CRITICAL_ISR(&s_lock);

    BaseType_t high_task_woken = pdFALSE;
    if (s_idle_sem) {
        xSemaphoreGiveFromISR(s_idle_sem, &high_task_woken);
    }
    if (done_cb && done_cb(done_user)) {
        high_task_woken = pdTRUE;
    }
    return high_task_woken == pdTRUE;
}

esp_err_t app_lcd_wait_idle(uint32_t timeout_ms)
{
    if (!s_idle_sem) {
        return ESP_ERR_INVALID_STATE;
    }
    TickType_t ticks = timeout_ms == UINT32_MAX ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    if (xSemaphoreTake(s_idle_sem, ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    xSemaphoreGive(s_idle_sem);
    return ESP_OK;
}

esp_err_t app_lcd_blit_rect_async(int x,
                                  int y,
                                  int width,
                                  int height,
                                  const void *packed_rgb565,
                                  app_lcd_blit_done_cb_t done_cb,
                                  void *done_user)
{
    if (!s_panel || !s_idle_sem || !packed_rgb565 || width <= 0 || height <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_idle_sem, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    portENTER_CRITICAL(&s_lock);
    s_done_cb = done_cb;
    s_done_user = done_user;
    s_transfer_active = true;
    portEXIT_CRITICAL(&s_lock);

    esp_err_t err = esp_lcd_panel_draw_bitmap(s_panel, x, y, x + width, y + height, packed_rgb565);
    if (err != ESP_OK) {
        portENTER_CRITICAL(&s_lock);
        s_done_cb = NULL;
        s_done_user = NULL;
        s_transfer_active = false;
        portEXIT_CRITICAL(&s_lock);
        xSemaphoreGive(s_idle_sem);
    }
    return err;
}
