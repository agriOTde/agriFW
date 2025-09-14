#ifndef SHT31_H
#define SHT31_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SHT31_I2C_ADDR       0x44  // Default I2C address for SHT31
#define SHT31_MEAS_HIGHREP   0x2400
#define SHT31_MEAS_MEDREP    0x240B
#define SHT31_MEAS_LOWREP    0x2416

#define SHT31_READSTATUS     0xF32D
#define SHT31_CLEARSTATUS    0x3041
#define SHT31_SOFTRESET      0x30A2

typedef enum {
    SHT31_HIGH_PRECISION = SHT31_MEAS_HIGHREP,
    SHT31_MEDIUM_PRECISION = SHT31_MEAS_MEDREP,
    SHT31_LOW_PRECISION = SHT31_MEAS_LOWREP
} sht31_precision_t;

/**
 * @brief Initialize the SHT31 sensor
 * 
 * @param i2c_port I2C port number
 * @param i2c_addr I2C address of the sensor
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t sht31_init(int i2c_port, uint8_t i2c_addr);

/**
 * @brief Read temperature and humidity
 * 
 * @param temperature Pointer to store temperature (in °C multiplied by 100)
 * @param humidity Pointer to store humidity (in %RH multiplied by 100)
 * @param precision Measurement precision
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t sht31_read(float *temperature, float *humidity, sht31_precision_t precision);

/**
 * @brief Soft reset the SHT31 sensor
 * 
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t sht31_soft_reset(void);

/**
 * @brief Read status register of SHT31
 * 
 * @param status Pointer to store the status register value
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t sht31_read_status(uint16_t *status);

/**
 * @brief Clear the status register of SHT31
 * 
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t sht31_clear_status(void);

/**
 * @brief Validate CRC of received data (Optional)
 * 
 * @param data Data bytes to validate
 * @param crc Expected CRC value
 * @return true If CRC is valid
 * @return false If CRC is invalid
 */
bool sht31_validate_crc(uint8_t *data, uint8_t crc);

#ifdef __cplusplus
}
#endif

#endif // SHT31_H
