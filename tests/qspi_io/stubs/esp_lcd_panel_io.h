#pragma once
#include "driver/spi_master.h"
typedef struct esp_lcd_panel_io_t esp_lcd_panel_io_t;
typedef esp_lcd_panel_io_t *esp_lcd_panel_io_handle_t;
typedef void esp_lcd_panel_io_event_data_t;
typedef bool (*esp_lcd_panel_io_color_trans_done_cb_t)(esp_lcd_panel_io_handle_t, esp_lcd_panel_io_event_data_t *, void *);
typedef struct { esp_lcd_panel_io_color_trans_done_cb_t on_color_trans_done; } esp_lcd_panel_io_callbacks_t;
typedef struct {
    int lcd_cmd_bits, lcd_param_bits, dc_gpio_num, spi_mode, pclk_hz, cs_gpio_num;
    struct { bool quad_mode, octal_mode, sio_mode, lsb_first, cs_high_active; } flags;
    esp_lcd_panel_io_color_trans_done_cb_t on_color_trans_done;
    void *user_ctx;
} esp_lcd_panel_io_spi_config_t;
