#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "app_lcd_qspi_io.h"
#include "esp_lcd_panel_io_interface.h"

unsigned test_yields;
static spi_device_interface_config_t device;
static spi_transaction_ext_t *pending;
static spi_transaction_ext_t pending_copy;
static bool acquired, keep_cs, fail_queue;
static unsigned callbacks, queued, headers;
static size_t total_bytes;
static esp_lcd_panel_io_handle_t expected_io;
static int cookie;
static uint8_t pixels[5000];

esp_err_t spi_bus_get_max_transaction_len(spi_host_device_t host, size_t *n)
{ (void)host; assert(device.post_cb); *n = 1024; return ESP_OK; }
esp_err_t spi_bus_add_device(spi_host_device_t host, const spi_device_interface_config_t *c, spi_device_handle_t *d)
{
    (void)host;
    assert(c->command_bits == 8 && c->address_bits == 24 && c->queue_size == 1);
    device = *c; *d = &device;
    return ESP_OK;
}
esp_err_t spi_bus_remove_device(spi_device_handle_t d)
{ (void)d; assert(!pending && !acquired); return ESP_OK; }
esp_err_t spi_device_acquire_bus(spi_device_handle_t d, int timeout)
{ (void)d; (void)timeout; assert(!acquired); acquired = true; return ESP_OK; }
void spi_device_release_bus(spi_device_handle_t d)
{ (void)d; assert(acquired); acquired = false; }

esp_err_t spi_device_get_trans_result(spi_device_handle_t d, spi_transaction_t **out, int timeout)
{
    (void)d; (void)timeout;
    assert(pending);
    assert(memcmp(pending, &pending_copy, sizeof(*pending)) == 0);
    *out = &pending->base;
    device.post_cb(*out);
    pending = NULL;
    return ESP_OK;
}

esp_err_t spi_device_polling_transmit(spi_device_handle_t d, spi_transaction_t *t)
{
    (void)d;
    assert(!pending && !keep_cs);
    spi_transaction_ext_t *ext = (spi_transaction_ext_t *)t;
    assert(ext->command_bits == 8 && ext->address_bits == 24);
    assert(t->cmd == 0x02 && t->addr == 0x002a00);
    assert(!(t->flags & SPI_TRANS_MODE_QIO));
    if (t->length) {
        assert(t->length == 32 && (t->flags & SPI_TRANS_USE_TXDATA));
        const uint8_t expected[] = {0, 0, 1, 155};
        assert(memcmp(t->tx_data, expected, sizeof(expected)) == 0);
    } else {
        assert(t->tx_buffer == NULL && !(t->flags & SPI_TRANS_USE_TXDATA));
    }
    device.post_cb(t);
    return ESP_OK;
}

esp_err_t spi_device_queue_trans(spi_device_handle_t d, spi_transaction_t *t, int timeout)
{
    (void)d; (void)timeout;
    assert(acquired && !pending);
    if (fail_queue) { fail_queue = false; return ESP_FAIL; }
    spi_transaction_ext_t *ext = (spi_transaction_ext_t *)t;
    assert(t->flags & SPI_TRANS_MODE_QIO);
    if (!keep_cs) {
        assert(ext->command_bits == 8 && ext->address_bits == 24);
        assert(t->cmd == 0x32 && t->addr == 0x002c00);
        headers++;
    } else {
        assert(ext->command_bits == 0 && ext->address_bits == 0);
    }
    assert(t->length > 0 && t->length <= 1024 * 8);
    keep_cs = (t->flags & SPI_TRANS_CS_KEEP_ACTIVE) != 0;
    assert((t->user != NULL) == !keep_cs);
    total_bytes += t->length / 8;
    queued++;
    pending = ext; pending_copy = *ext;
    return ESP_OK;
}

static bool done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *event, void *user)
{
    assert(io == expected_io && event == NULL && user == &cookie);
    assert(!keep_cs);
    callbacks++;
    return true;
}

int main(void)
{
    esp_lcd_panel_io_spi_config_t config = {
        .lcd_cmd_bits = 32, .lcd_param_bits = 8, .dc_gpio_num = -1,
        .spi_mode = 0, .pclk_hz = 80000000, .cs_gpio_num = 21,
        .flags.quad_mode = true, .on_color_trans_done = done, .user_ctx = &cookie,
    };
    esp_lcd_panel_io_handle_t io;
    assert(app_lcd_new_qspi_io(0, NULL, &io) == ESP_ERR_INVALID_ARG);
    assert(app_lcd_new_qspi_io(0, &config, &io) == ESP_OK);
    expected_io = io;
    const uint8_t params[] = {0, 0, 1, 155};
    assert(io->tx_param(io, 0x02002a00, params, 4) == ESP_OK);
    assert(io->tx_param(io, 0x02002a00, params, 0) == ESP_OK);
    assert(callbacks == 0);
    assert(io->tx_color(io, 0x32002c00, pixels, sizeof(pixels)) == ESP_OK);
    assert(queued == 5 && headers == 1 && total_bytes == sizeof(pixels));
    assert(callbacks == 0 && pending);
    /* A subsequent command drains the final async DMA before reusing state. */
    assert(io->tx_param(io, 0x02002a00, params, 4) == ESP_OK);
    assert(callbacks == 1 && test_yields == 1 && !pending);
    assert(io->tx_color(io, 0x32002c00, NULL, 2) == ESP_ERR_INVALID_ARG);
    fail_queue = true;
    assert(io->tx_color(io, 0x32002c00, pixels, 10) == ESP_FAIL);
    assert(!acquired && !pending && callbacks == 1);
    assert(io->tx_color(io, 0x32002c00, pixels, 10) == ESP_OK);
    assert(io->del(io) == ESP_OK);
    assert(callbacks == 2 && test_yields == 2);
    puts("QSPI headers, parameter bytes, chunk continuation, DMA lifetime, callbacks and error recovery passed");
}
