#pragma once

#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"

/* SPD2010 QSPI framing: one opcode byte, a 24-bit address, then parameters
 * on one data line or pixels on four. Caller serializes panel operations. */
esp_err_t app_lcd_new_qspi_io(spi_host_device_t host,
                             const esp_lcd_panel_io_spi_config_t *config,
                             esp_lcd_panel_io_handle_t *out_io);
