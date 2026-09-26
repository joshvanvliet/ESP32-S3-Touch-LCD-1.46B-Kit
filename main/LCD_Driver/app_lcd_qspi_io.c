#include "app_lcd_qspi_io.h"

#include <stdlib.h>
#include <string.h>
#include "esp_lcd_panel_io_interface.h"
#include "freertos/FreeRTOS.h"

typedef struct {
    esp_lcd_panel_io_t base;
    spi_device_handle_t device;
    spi_transaction_ext_t color;
    size_t max_transfer_bytes;
    bool pending;
    esp_lcd_panel_io_color_trans_done_cb_t done;
    void *user;
} app_qspi_io_t;

static esp_err_t wait_color(app_qspi_io_t *io)
{
    if (!io->pending) return ESP_OK;
    spi_transaction_t *completed;
    esp_err_t err = spi_device_get_trans_result(io->device, &completed, portMAX_DELAY);
    if (err == ESP_OK) io->pending = false;
    return err;
}

static void color_done(spi_transaction_t *transaction)
{
    /* Only the final color chunk has a user pointer. Parameter transactions
     * and intermediate chunks must never release the caller's color buffer. */
    app_qspi_io_t *io = transaction->user;
    if (io && io->done && io->done(&io->base, NULL, io->user)) {
        portYIELD_FROM_ISR();
    }
}

static void set_command(spi_transaction_ext_t *transaction, int command)
{
    transaction->base.flags |= SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR;
    if (command >= 0) {
        transaction->command_bits = 8;
        transaction->address_bits = 24;
        transaction->base.cmd = (uint32_t)command >> 24;
        transaction->base.addr = (uint32_t)command & 0x00ffffffu;
    }
}

static esp_err_t tx_param(esp_lcd_panel_io_t *base, int command, const void *param, size_t size)
{
    app_qspi_io_t *io = (app_qspi_io_t *)base;
    if (size && !param) return ESP_ERR_INVALID_ARG;
    esp_err_t err = wait_color(io);
    if (err != ESP_OK) return err;

    spi_transaction_ext_t transaction = {0};
    set_command(&transaction, command);
    transaction.base.length = size * 8;
    if (size && size <= sizeof(transaction.base.tx_data)) {
        transaction.base.flags |= SPI_TRANS_USE_TXDATA;
        memcpy(transaction.base.tx_data, param, size);
    } else {
        transaction.base.tx_buffer = size ? param : NULL;
    }
    /* The peripheral sends header and payload under one CS assertion. This
     * keeps every controller command, but avoids a separate polling transfer
     * and bus acquisition for each command's header. */
    return spi_device_polling_transmit(io->device, &transaction.base);
}

static esp_err_t tx_color(esp_lcd_panel_io_t *base, int command, const void *pixels, size_t size)
{
    app_qspi_io_t *io = (app_qspi_io_t *)base;
    if (!pixels || !size) return ESP_ERR_INVALID_ARG;
    esp_err_t err = wait_color(io);
    if (err != ESP_OK) return err;
    err = spi_device_acquire_bus(io->device, portMAX_DELAY);
    if (err != ESP_OK) return err;

    const uint8_t *source = pixels;
    while (size) {
        size_t chunk = size < io->max_transfer_bytes ? size : io->max_transfer_bytes;
        memset(&io->color, 0, sizeof(io->color));
        set_command(&io->color, command);
        io->color.base.flags |= SPI_TRANS_MODE_QIO;
        io->color.base.length = chunk * 8;
        io->color.base.tx_buffer = source;
        if (chunk < size) {
            io->color.base.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
        } else {
            io->color.base.user = io;
        }
        err = spi_device_queue_trans(io->device, &io->color.base, portMAX_DELAY);
        if (err != ESP_OK) break;
        io->pending = true;
        size -= chunk;
        source += chunk;
        command = -1;
        if (size) {
            err = wait_color(io);
            if (err != ESP_OK) break;
        }
    }
    spi_device_release_bus(io->device);
    return err;
}

static esp_err_t rx_param(esp_lcd_panel_io_t *base, int command, void *param, size_t size)
{
    /* This board reads touch over I2C; no display register reads are used. */
    (void)base; (void)command; (void)param; (void)size;
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t register_callbacks(esp_lcd_panel_io_t *base,
                                   const esp_lcd_panel_io_callbacks_t *callbacks,
                                   void *user)
{
    if (!callbacks) return ESP_ERR_INVALID_ARG;
    app_qspi_io_t *io = (app_qspi_io_t *)base;
    esp_err_t err = wait_color(io);
    if (err != ESP_OK) return err;
    io->done = callbacks->on_color_trans_done;
    io->user = user;
    return ESP_OK;
}

static esp_err_t delete_io(esp_lcd_panel_io_t *base)
{
    app_qspi_io_t *io = (app_qspi_io_t *)base;
    esp_err_t err = wait_color(io);
    if (err != ESP_OK) return err;
    err = spi_bus_remove_device(io->device);
    if (err == ESP_OK) free(io);
    return err;
}

esp_err_t app_lcd_new_qspi_io(spi_host_device_t host,
                             const esp_lcd_panel_io_spi_config_t *config,
                             esp_lcd_panel_io_handle_t *out_io)
{
    if (!config || !out_io) return ESP_ERR_INVALID_ARG;
    *out_io = NULL;
    if (!config->flags.quad_mode || config->lcd_cmd_bits != 32 || config->lcd_param_bits != 8 ||
        config->dc_gpio_num != -1 || config->flags.octal_mode || config->flags.sio_mode ||
        config->flags.lsb_first) return ESP_ERR_NOT_SUPPORTED;

    app_qspi_io_t *io = calloc(1, sizeof(*io));
    if (!io) return ESP_ERR_NO_MEM;
    spi_device_interface_config_t device = {
        .command_bits = 8,
        .address_bits = 24,
        .mode = config->spi_mode,
        .clock_speed_hz = config->pclk_hz,
        .spics_io_num = config->cs_gpio_num,
        .flags = SPI_DEVICE_HALFDUPLEX | (config->flags.cs_high_active ? SPI_DEVICE_POSITIVE_CS : 0),
        .queue_size = 1,
        .post_cb = color_done,
    };
    /* Adding the first device initializes the SPI master's host state. Query
     * its DMA limit afterwards, as the generic ESP-IDF panel IO does. */
    esp_err_t err = spi_bus_add_device(host, &device, &io->device);
    if (err == ESP_OK) err = spi_bus_get_max_transaction_len(host, &io->max_transfer_bytes);
    if (err != ESP_OK) {
        if (io->device) spi_bus_remove_device(io->device);
        free(io);
        return err;
    }
    io->done = config->on_color_trans_done;
    io->user = config->user_ctx;
    io->base.tx_param = tx_param;
    io->base.tx_color = tx_color;
    io->base.rx_param = rx_param;
    io->base.del = delete_io;
    io->base.register_event_callbacks = register_callbacks;
    *out_io = &io->base;
    return ESP_OK;
}
