#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

typedef bool (*app_lcd_blit_done_cb_t)(void *user);

esp_err_t app_lcd_init(esp_lcd_panel_handle_t panel);
bool app_lcd_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                              esp_lcd_panel_io_event_data_t *edata,
                              void *user_ctx);
esp_err_t app_lcd_wait_idle(uint32_t timeout_ms);
esp_err_t app_lcd_blit_rect_async(int x,
                                  int y,
                                  int width,
                                  int height,
                                  const void *packed_rgb565,
                                  app_lcd_blit_done_cb_t done_cb,
                                  void *done_user);
