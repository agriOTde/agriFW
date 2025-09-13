#include "sht41.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "SHT4x";

// ===== Driver context =====
static int     s_i2c_port  = -1;
static uint8_t s_i2c_addr  = SHT4X_I2C_ADDR_DEFAULT;

// ===== SHT4x command set =====
#define SHT4X_CMD_MEAS_HIGH   0xFD
#define SHT4X_CMD_MEAS_MED    0xF6
#define SHT4X_CMD_MEAS_LOW    0xE0
#define SHT4X_CMD_SOFT_RESET  0x94
// (Optional extras you can add later)
// #define SHT4X_CMD_READ_SN   0x89
// #define SHT4X_CMD_HEATER_*  ...

// ----- Internal helpers -----
static inline TickType_t _to_ticks(uint32_t ms) {
    return (ms / portTICK_PERIOD_MS) ? (ms / portTICK_PERIOD_MS) : 1;
}

static esp_err_t _write_cmd(uint8_t cmd)
{
    return i2c_master_write_to_device(
        s_i2c_port, s_i2c_addr, &cmd, 1, _to_ticks(SHT4X_TIMEOUT_MS));
}

static esp_err_t _read_bytes(uint8_t *data, size_t len)
{
    return i2c_master_read_from_device(
        s_i2c_port, s_i2c_addr, data, len, _to_ticks(SHT4X_TIMEOUT_MS));
}

// ----- Public API -----
esp_err_t sht4x_init(int i2c_port, uint8_t i2c_addr)
{
    s_i2c_port = i2c_port;
    s_i2c_addr = i2c_addr;
    return sht4x_soft_reset();
}

esp_err_t sht4x_soft_reset(void)
{
    esp_err_t ret = _write_cmd(SHT4X_CMD_SOFT_RESET);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Soft reset command failed: %s", esp_err_to_name(ret));
        return ret;
    }
    // Datasheet: typical reset time small; 1–2 ms is enough. Be generous:
    vTaskDelay(pdMS_TO_TICKS(2));
    return ESP_OK;
}

bool sht4x_validate_crc(const uint8_t *data, uint8_t crc)
{
    uint8_t c = SHT4X_CRC_INITIAL;
    for (int i = 0; i < 2; i++) {
        c ^= data[i];
        for (int b = 0; b < 8; b++) {
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ SHT4X_CRC_POLYNOMIAL)
                           : (uint8_t)(c << 1);
        }
    }
    return (c == crc);
}

esp_err_t sht4x_read(float *temperature, float *humidity, sht4x_precision_t precision)
{
    if (!temperature || !humidity) return ESP_ERR_INVALID_ARG;

    uint8_t cmd =
        (precision == SHT4X_PREC_HIGH) ? SHT4X_CMD_MEAS_HIGH :
        (precision == SHT4X_PREC_MED)  ? SHT4X_CMD_MEAS_MED  :
                                         SHT4X_CMD_MEAS_LOW;

    esp_err_t ret = _write_cmd(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send measurement command: %s", esp_err_to_name(ret));
        return ret;
    }

    // Max conversion time ~8.3 ms (high). 10 ms is safe for all modes.
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t buf[6] = {0};
    ret = _read_bytes(buf, sizeof(buf));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement data: %s", esp_err_to_name(ret));
        return ret;
    }

    // Validate both CRCs (T[0..1], CRC buf[2]) and (RH[3..4], CRC buf[5])
    if (!sht4x_validate_crc(&buf[0], buf[2]) || !sht4x_validate_crc(&buf[3], buf[5])) {
        ESP_LOGE(TAG, "CRC validation failed");
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t raw_t  = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t raw_rh = ((uint16_t)buf[3] << 8) | buf[4];

    // Temperature:  T [°C] = -45 + 175 * raw/65535
    float t_c = -45.0f + (175.0f * (float)raw_t / 65535.0f);

    // Humidity: RH [%] = -6 + 125 * raw/65535, clamp 0..100
    float rh = -6.0f + (125.0f * (float)raw_rh / 65535.0f);
    if (rh < 0.0f) rh = 0.0f; else if (rh > 100.0f) rh = 100.0f;

    *temperature = t_c;
    *humidity    = rh;
    return ESP_OK;
}

// --- SHT31-compat shims (not available on SHT4x) ---
esp_err_t sht4x_read_status(uint16_t *status)
{
    (void)status;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t sht4x_clear_status(void)
{
    return ESP_ERR_NOT_SUPPORTED;
}
