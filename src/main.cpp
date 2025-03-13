#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include "freertos/event_groups.h"
#include "soilMoist.h"
#include "mqtt_manager.h"
#include "npk.h"
#include "sht31.h"
#include "driver/i2c.h"
#include <string.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "cJSON.h"
#include "lwip/netdb.h"


#define I2C_MASTER_SCL_IO    22  /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO    21  /*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM       I2C_NUM_0 /*!< I2C port number */
#define I2C_MASTER_FREQ_HZ   100000 /*!< I2C clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE  0 /*!< I2C disable TX buffer */
#define I2C_MASTER_RX_BUF_DISABLE  0 /*!< I2C disable RX buffer */

#define MQTT_TOPIC_PUB "esp32/pub"
#define MQTT_TOPIC_SUB "esp32/sub"

#ifdef __cplusplus
extern "C"
{
#endif

// /***************** C headers ******************/
#include "wifi_manager.h"
#include "httpClient.h"

// /*********************************************/

#ifdef __cplusplus
} // extern "C"
#endif


// /***************** TAGS ******************/
static const char *WIFI_TAG = "wifi station";
static const char *JSON_TAG = "JSON object";
static const char *SHT_TAG = "SHT Ping";
static const char *MAIN_TAG = "MAIN";
static const char *PUB_TAG = "Publish topic";

TaskHandle_t myTaskHandle = NULL;

extern "C"
{ 

    /**
 * @brief Class to handle SHT31 sensor communication
 */
class SHT31Sensor {
    public:
        SHT31Sensor(i2c_port_t i2c_num, uint8_t address) : i2c_num(i2c_num), address(address) {}
    
        bool init() {
            if (sht31_init(i2c_num, address) == ESP_OK) {
                ESP_LOGI(SHT_TAG, "SHT31 sensor initialized successfully.");
                return true;
            } else {
                ESP_LOGE(SHT_TAG, "Failed to initialize SHT31 sensor.");
                return false;
            }
        }
    
        bool read(float &temperature, float &humidity) {
            return sht31_read(&temperature, &humidity, SHT31_HIGH_PRECISION) == ESP_OK;
        }
    
    private:
        i2c_port_t i2c_num;
        uint8_t address;
    };

    // ################################## Funtion Declarations ####################################
    
    void ping_sht(void *arg);
    void post_data(void *arg);
    static esp_err_t i2c_master_init();

    // ############################################################################################


    void app_main() {

    printf("Main\n");

    // Initialize NVS (Non-volatile storage for credentials)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
    // Initialize Wi-Fi
    wifi_init_sta();

    set_dns();
    // test DNS
    struct addrinfo *res;
    int err = getaddrinfo("aeti3itrm7uta-ats.iot.eu-north-1.amazonaws.com", NULL, NULL, &res);
    
    if (err == 0) {
        ESP_LOGI("DNS", "Hostname resolved successfully!");
        freeaddrinfo(res);
    } else {
        ESP_LOGE("DNS", "Failed to resolve hostname. Error: %d", err);
    }

    // Init MQTT Client
    mqtt_init();
    

    ESP_LOGI(MAIN_TAG, "Initializing I2C...");
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to initialize I2C");
        return;
    }

    // Task SHT
    xTaskCreatePinnedToCore
    (post_data, "posting SHT31", 9000, NULL,5, &myTaskHandle, 1);

   }

void ping_sht(void *pvParameter) {
    SHT31Sensor sensor(I2C_MASTER_NUM, SHT31_I2C_ADDR);

    if (!sensor.init()) {
        vTaskDelete(NULL);
        return;
    }

    float temperature, humidity;
    while (1) {
        if (sensor.read(temperature, humidity)) {
            ESP_LOGI(SHT_TAG, "Temperature: %.2f °C, Humidity: %.2f %%", temperature, humidity);
        } else {
            ESP_LOGE(SHT_TAG, "Failed to read temperature and humidity");
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds before next read
    }
}
/**
 * @brief Initialize the I2C master interface
 */
static esp_err_t i2c_master_init() {

    i2c_config_t conf = {};  // Zero-initialize struct
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "I2C config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER,
                             I2C_MASTER_TX_BUF_DISABLE, I2C_MASTER_RX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "I2C driver install failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

void post_data(void *arg){
    SHT31Sensor sensor(I2C_MASTER_NUM, SHT31_I2C_ADDR);

    if (!sensor.init()) {
        vTaskDelete(NULL);
        return;
    }

    float temperature, humidity;
    while (1) {
        if (sensor.read(temperature, humidity)) {
            ESP_LOGI(SHT_TAG, "Temperature: %.2f °C, Humidity: %.2f %%", temperature, humidity);

            cJSON *mqtt_data;
            mqtt_data = cJSON_CreateObject();
            cJSON_AddNumberToObject(mqtt_data, "Temp", temperature);
            cJSON_AddNumberToObject(mqtt_data, "moistureVal", humidity);
            const char *my_json_string = cJSON_Print(mqtt_data);

            char *post_data = cJSON_Print(mqtt_data);
            mqtt_publish(MQTT_TOPIC_PUB,post_data);
        } else {
            ESP_LOGE(SHT_TAG, "Failed to read temperature and humidity");
        }

        vTaskDelay(pdMS_TO_TICKS(30000)); // Wait 2 seconds before next read
    }
}


};