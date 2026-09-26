#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef int esp_err_t;
typedef int spi_host_device_t;
typedef void *spi_device_handle_t;
#define ESP_OK 0
#define ESP_ERR_INVALID_ARG 1
#define ESP_ERR_NO_MEM 2
#define ESP_ERR_NOT_SUPPORTED 3
#define ESP_FAIL 4
#define SPI_TRANS_VARIABLE_CMD 1
#define SPI_TRANS_VARIABLE_ADDR 2
#define SPI_TRANS_USE_TXDATA 4
#define SPI_TRANS_MODE_QIO 8
#define SPI_TRANS_CS_KEEP_ACTIVE 16
#define SPI_DEVICE_HALFDUPLEX 32
#define SPI_DEVICE_POSITIVE_CS 64
typedef struct {
    unsigned flags;
    uint16_t cmd;
    uint64_t addr;
    size_t length;
    const void *tx_buffer;
    uint8_t tx_data[4];
    void *user;
} spi_transaction_t;
typedef struct {
    spi_transaction_t base;
    unsigned command_bits, address_bits;
} spi_transaction_ext_t;
typedef struct {
    int command_bits, address_bits, mode, clock_speed_hz, spics_io_num;
    unsigned flags;
    int queue_size;
    void (*post_cb)(spi_transaction_t *);
} spi_device_interface_config_t;
esp_err_t spi_device_get_trans_result(spi_device_handle_t, spi_transaction_t **, int);
esp_err_t spi_device_polling_transmit(spi_device_handle_t, spi_transaction_t *);
esp_err_t spi_device_queue_trans(spi_device_handle_t, spi_transaction_t *, int);
esp_err_t spi_device_acquire_bus(spi_device_handle_t, int);
void spi_device_release_bus(spi_device_handle_t);
esp_err_t spi_bus_get_max_transaction_len(spi_host_device_t, size_t *);
esp_err_t spi_bus_add_device(spi_host_device_t, const spi_device_interface_config_t *, spi_device_handle_t *);
esp_err_t spi_bus_remove_device(spi_device_handle_t);
