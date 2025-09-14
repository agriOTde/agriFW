#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Default I2C address for SHT41I-AD1x
#define SHT4X_I2C_ADDR_DEFAULT  0x44

// Timeout for I2C ops (ms)
#define SHT4X_TIMEOUT_MS        1000

// CRC-8 params used by Sensirion
#define SHT4X_CRC_POLYNOMIAL    0x31
#define SHT4X_CRC_INITIAL       0xFF

// Precision (a.k.a. repeatability) / power modes
typedef enum {
    SHT4X_PREC_LOW  = 0,   // 0xE0
    SHT4X_PREC_MED  = 1,   // 0xF6
    SHT4X_PREC_HIGH = 2,   // 0xFD
} sht4x_precision_t;

/**
 * @brief Initialize SHT4x driver context.
 *
 * @param i2c_port  ESP-IDF I2C port number (e.g., I2C_NUM_0 / I2C_NUM_1)
 * @param i2c_addr  7-bit I2C address (use SHT4X_I2C_ADDR_DEFAULT if unsure)
 * @return esp_err_t
 */
esp_err_t sht4x_init(int i2c_port, uint8_t i2c_addr);

/**
 * @brief Trigger a measurement and read temperature & humidity.
 *
 * @param temperature  out (°C)
 * @param humidity     out (%RH)
 * @param precision    repeatability / power choice
 * @return esp_err_t   ESP_OK on success, ESP_ERR_INVALID_CRC if CRC fails
 */
esp_err_t sht4x_read(float *temperature, float *humidity, sht4x_precision_t precision);

/**
 * @brief Soft reset (0x94).
 */
esp_err_t sht4x_soft_reset(void);

/**
 * @brief SHT31-compat helpers (not supported on SHT4x).
 */
esp_err_t sht4x_read_status(uint16_t *status);
esp_err_t sht4x_clear_status(void);

/**
 * @brief CRC validation for a 2-byte word with CRC-8.
 *
 * @param data pointer to two data bytes
 * @param crc  received CRC byte
 * @return true if CRC is valid
 */
bool sht4x_validate_crc(const uint8_t *data, uint8_t crc);

#ifdef __cplusplus
}
#endif
