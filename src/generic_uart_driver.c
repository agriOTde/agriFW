#include "generic_uart_driver.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "GENERIC_UART";

esp_err_t uart_device_init(uart_device_config_t *config)
{
    uart_config_t uart_conf = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    esp_err_t ret;

    ret = uart_param_config(config->uart_port, &uart_conf);
    if (ret != ESP_OK) return ret;

    ret = uart_set_pin(config->uart_port, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) return ret;

    ret = uart_driver_install(config->uart_port, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "UART%d initialized (TX: GPIO%d, RX: GPIO%d, baud: %d)",
             config->uart_port, config->tx_pin, config->rx_pin, config->baud_rate);

    return ESP_OK;
}

esp_err_t uart_device_send(uart_port_t uart_port, const uint8_t *data, size_t len)
{
    int sent = uart_write_bytes(uart_port, (const char *)data, len);
    uart_wait_tx_done(uart_port, pdMS_TO_TICKS(100));
    return (sent == len) ? ESP_OK : ESP_FAIL;
}

int uart_device_receive(uart_port_t uart_port, uint8_t *buf, size_t max_len, TickType_t timeout)
{
    return uart_read_bytes(uart_port, buf, max_len, timeout);
}
