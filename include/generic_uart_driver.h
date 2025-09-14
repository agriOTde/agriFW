#ifndef GENERIC_UART_DRIVER_H
#define GENERIC_UART_DRIVER_H

#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART_BUF_SIZE 256

#ifdef __cplusplus
extern "C" {
#endif

// UART config structure
typedef struct {
    uart_port_t uart_port;
    int tx_pin;
    int rx_pin;
    int baud_rate;
} uart_device_config_t;

// Initialize UART with given config
esp_err_t uart_device_init(uart_device_config_t *config);

// Send data over UART
esp_err_t uart_device_send(uart_port_t uart_port, const uint8_t *data, size_t len);

// Receive data from UART
int uart_device_receive(uart_port_t uart_port, uint8_t *buf, size_t max_len, TickType_t timeout);

#ifdef __cplusplus
}
#endif

#endif // GENERIC_UART_DRIVER_H
