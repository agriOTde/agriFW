#include "sht31.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define SHT31_TIMEOUT_MS 1000
#define SHT31_CRC_POLYNOMIAL 0x31
#define SHT31_CRC_INITIAL 0xFF

static const char *TAG = "SHT31";

static int sht31_i2c_port = -1;
static uint8_t sht31_i2c_addr = SHT31_I2C_ADDR;

/**
 * @brief Perform I2C write operation
 */
static esp_err_t sht31_write_cmd(uint16_t cmd) {
    uint8_t buf[2] = { (cmd >> 8) & 0xFF, cmd & 0xFF };
    return i2c_master_write_to_device(sht31_i2c_port, sht31_i2c_addr, buf, sizeof(buf), SHT31_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Perform I2C read operation
 */
static esp_err_t sht31_read_data(uint8_t *data, size_t len) {
    return i2c_master_read_from_device(sht31_i2c_port, sht31_i2c_addr, data, len, SHT31_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t sht31_init(int i2c_port, uint8_t i2c_addr) {
    sht31_i2c_port = i2c_port;
    sht31_i2c_addr = i2c_addr;
    return sht31_soft_reset();
}

esp_err_t sht31_read(float *temperature, float *humidity, sht31_precision_t precision) {
    uint8_t data[6];
    esp_err_t ret = sht31_write_cmd(precision);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send measurement command");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // Wait for measurement

    ret = sht31_read_data(data, sizeof(data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement data");
        return ret;
    }

    // Validate CRC (optional)
    if (!sht31_validate_crc(&data[0], data[2]) || !sht31_validate_crc(&data[3], data[5])) {
        ESP_LOGE(TAG, "CRC validation failed");
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t raw_temp = (data[0] << 8) | data[1];
    uint16_t raw_humidity = (data[3] << 8) | data[4];

    *temperature = -45.0 + (175.0 * raw_temp / 65535.0);
    *humidity = 100.0 * raw_humidity / 65535.0;

    return ESP_OK;
}

esp_err_t sht31_soft_reset(void) {
    return sht31_write_cmd(SHT31_SOFTRESET);
}

esp_err_t sht31_read_status(uint16_t *status) {
    uint8_t data[3];

    esp_err_t ret = sht31_write_cmd(SHT31_READSTATUS);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(10));

    ret = sht31_read_data(data, sizeof(data));
    if (ret != ESP_OK) return ret;

    if (!sht31_validate_crc(&data[0], data[2])) {
        ESP_LOGE(TAG, "Status register CRC validation failed");
        return ESP_ERR_INVALID_CRC;
    }

    *status = (data[0] << 8) | data[1];
    return ESP_OK;
}

esp_err_t sht31_clear_status(void) {
    return sht31_write_cmd(SHT31_CLEARSTATUS);
}

/**
 * @brief CRC validation for received data
 */
bool sht31_validate_crc(uint8_t *data, uint8_t crc) {
    uint8_t crc_calc = SHT31_CRC_INITIAL;

    for (uint8_t i = 0; i < 2; i++) {
        crc_calc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc_calc & 0x80) {
                crc_calc = (crc_calc << 1) ^ SHT31_CRC_POLYNOMIAL;
            } else {
                crc_calc <<= 1;
            }
        }
    }
    return crc_calc == crc;
}
