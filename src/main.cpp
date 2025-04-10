// #include "esp_log.h"
// #include "freertos/FreeRTOS.h"
// #include <freertos/task.h>
// #include "freertos/event_groups.h"
// #include "esp_task_wdt.h"
// #include "soilMoist.h"
// #include "mqtt_manager.h"
// #include "npk.h"
// #include "sht31.h"
// #include "driver/i2c.h"
// #include <string.h>
// #include "esp_system.h"
// #include "esp_event.h"
// #include "esp_log.h"
// #include "lwip/err.h"
// #include "lwip/sys.h"
// #include "cJSON.h"
// #include "lwip/netdb.h"


// #define I2C_MASTER_SCL_IO    22  /*!< GPIO number for I2C master clock */
// #define I2C_MASTER_SDA_IO    21  /*!< GPIO number for I2C master data  */
// #define I2C_MASTER_NUM       I2C_NUM_0 /*!< I2C port number */
// #define I2C_MASTER_FREQ_HZ   100000 /*!< I2C clock frequency */
// #define I2C_MASTER_TX_BUF_DISABLE  0 /*!< I2C disable TX buffer */
// #define I2C_MASTER_RX_BUF_DISABLE  0 /*!< I2C disable RX buffer */

// #define MQTT_TOPIC_PUB "esp32/pub"
// #define MQTT_TOPIC_SUB "esp32/sub"

// #define LOG_LOCAL_LEVEL ESP_LOG_ERROR
// #define MAX_LOGS_NVS 5

// #ifdef __cplusplus
// extern "C"
// {
// #endif

// // /***************** C headers ******************/
// #include "wifi_manager.h"
// #include "httpClient.h"

// // /*********************************************/

// #ifdef __cplusplus
// } // extern "C"
// #endif


// // /***************** TAGS ******************/
// static const char *WIFI_TAG = "wifi station";
// static const char *JSON_TAG = "JSON object";
// static const char *SHT_TAG = "SHT Ping";
// static const char *MAIN_TAG = "MAIN";
// static const char *PUB_TAG = "Publish topic";
// static const char *MQTT_TAG = "MQTT";

// TaskHandle_t myTaskHandle = NULL;


// extern "C"
// { 


// // ################################## Funtion Declarations ####################################

// void ping_sht(void *arg);
// void post_data(void *arg);
// static esp_err_t i2c_master_init();
// void read_logs_from_nvs();
// void log_error_to_nvs(const char *error_message);
// int custom_log_handler(const char *format, va_list args);

// // ############################################################################################

//     /**
//  * @brief Class to handle SHT31 sensor communication
//  */
// class SHT31Sensor {
//     public:
//         SHT31Sensor(i2c_port_t i2c_num, uint8_t address) : i2c_num(i2c_num), address(address) {}
    
//         bool init() {
//             if (sht31_init(i2c_num, address) == ESP_OK) {
//                 ESP_LOGI(SHT_TAG, "SHT31 sensor initialized successfully.");
//                 return true;
//             } else {
//                 ESP_LOGE(SHT_TAG, "Failed to initialize SHT31 sensor.");                
//                 return false;
//             }
//         }
    
//         bool read(float &temperature, float &humidity) {
//             return sht31_read(&temperature, &humidity, SHT31_HIGH_PRECISION) == ESP_OK;
//         }
    
//     private:
//         i2c_port_t i2c_num;
//         uint8_t address;
//     };



//     void app_main() {

//     printf("Main\n");
//     esp_log_set_vprintf(custom_log_handler);
    

//     // Initialize NVS (Non-volatile storage for credentials)
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ESP_ERROR_CHECK(nvs_flash_init());
//     }
//     read_logs_from_nvs(); //Read Stored Error Logs

//     // // Create a watchdog timer config
//     // esp_task_wdt_config_t config = {
//     //     .timeout_ms = 60000,   // Timeout duration in milliseconds (e.g., 5 seconds)
//     //     .idle_core_mask = (1 << 0) | (1 << 1),  // Subscribe both CPU cores
//     //     .trigger_panic = true   // Trigger a panic on timeout
//     // };

//     // // Initialize the watchdog timer with the configuration
//     // esp_err_t error_ret = esp_task_wdt_init(&config);
//     // if (ret != ESP_OK) {
//     //     ESP_LOGE(MAIN_TAG, "Failed to initialize Task Watchdog Timer: %s", esp_err_to_name(ret));
//     // }
//     // Initialize Wi-Fi
//     wifi_init_sta();

//     // Init MQTT Client
//     mqtt_init();

//     set_dns();
//     // test DNS
//     struct addrinfo *res;
//     int err = getaddrinfo("aeti3itrm7uta-ats.iot.eu-north-1.amazonaws.com", NULL, NULL, &res);
    
//     if (err == 0) {
//         ESP_LOGI("DNS", "Hostname resolved successfully!");
//         freeaddrinfo(res);
//     } else {
//         ESP_LOGE("DNS", "Failed to resolve hostname. Error: %d", err);
//     }

 
    

//     ESP_LOGI(MAIN_TAG, "Initializing I2C...");
//     if (i2c_master_init() != ESP_OK) {
//         ESP_LOGE(MAIN_TAG, "Failed to initialize I2C");
//         return;
//     }

//     // Task SHT
//     xTaskCreatePinnedToCore
//     (post_data, "posting SHT31", 4096, NULL,5, &myTaskHandle, 1);

//    }

// void ping_sht(void *pvParameter) {
//     SHT31Sensor sensor(I2C_MASTER_NUM, SHT31_I2C_ADDR);

//     if (!sensor.init()) {
//         vTaskDelete(NULL);
//         return;
//     }

//     float temperature, humidity;
//     while (1) {
//         if (sensor.read(temperature, humidity)) {
//             ESP_LOGI(SHT_TAG, "Temperature: %.2f °C, Humidity: %.2f %%", temperature, humidity);
//         } else {
//             ESP_LOGE(SHT_TAG, "Failed to read temperature and humidity");

//         }

//         vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds before next read
//     }
// }
// /**
//  * @brief Initialize the I2C master interface
//  */
// static esp_err_t i2c_master_init() {

//     i2c_config_t conf = {};  // Zero-initialize struct
//     conf.mode = I2C_MODE_MASTER;
//     conf.sda_io_num = I2C_MASTER_SDA_IO;
//     conf.scl_io_num = I2C_MASTER_SCL_IO;
//     conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
//     conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
//     conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

//     esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
//     if (ret != ESP_OK) {
//         ESP_LOGE("I2C", "I2C config failed: %s", esp_err_to_name(ret));
//         return ret;
//     }

//     ret = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER,
//                              I2C_MASTER_TX_BUF_DISABLE, I2C_MASTER_RX_BUF_DISABLE, 0);
//     if (ret != ESP_OK) {
//         ESP_LOGE("I2C", "I2C driver install failed: %s", esp_err_to_name(ret));
//     }

//     return ret;
// }

// void post_data(void *arg){
//     SHT31Sensor sensor(I2C_MASTER_NUM, SHT31_I2C_ADDR);

//     if (!sensor.init()) {
//         vTaskDelete(NULL);
//         return;
//     }

//     float temperature, humidity;

//     // esp_task_wdt_add(NULL);  // NULL will register the current task to the TWDT

//     while (1) {
//         if (sensor.read(temperature, humidity)) {
//             ESP_LOGI(SHT_TAG, "Temperature: %.2f °C, Humidity: %.2f %%", temperature, humidity);

//             cJSON *mqtt_data;
//             mqtt_data = cJSON_CreateObject();
//             cJSON_AddNumberToObject(mqtt_data, "Temp", temperature);
//             cJSON_AddNumberToObject(mqtt_data, "moistureVal", humidity);
//             const char *my_json_string = cJSON_Print(mqtt_data);
//             char *post_data = cJSON_Print(mqtt_data);
//             if(mqtt_publish(MQTT_TOPIC_PUB,post_data) != ESP_OK){
//                 ESP_LOGE(MQTT_TAG, "MQTT client not Publishing");
//             }
//             free(post_data);
//         } else {
//             ESP_LOGE(SHT_TAG, "Failed to read temperature and humidity");
//         }

//         // esp_task_wdt_reset();
//         vTaskDelay(pdMS_TO_TICKS(30000)); // Wait 2 seconds before next read
//     }
//     // esp_task_wdt_delete(NULL);
//     vTaskDelete(NULL);
// }


// void log_error_to_nvs(const char *error_message) {
//     nvs_handle_t my_handle;
//     esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
//     if (err == ESP_OK) {
//         int32_t log_index;
//         size_t size = sizeof(log_index);
//         if (nvs_get_i32(my_handle, "log_index", &log_index) != ESP_OK) {
//             log_index = 0;  // If no index exists, start at 0
//         }
//         char key[10];
//         sprintf(key, "log%ld", log_index % MAX_LOGS_NVS);  // Circular storage
//         nvs_set_str(my_handle, key, error_message);
//         log_index++;
//         nvs_set_i32(my_handle, "log_index", log_index);  // Save index
//         nvs_commit(my_handle);
//         nvs_close(my_handle);
//     }
// }

// void read_logs_from_nvs() {
//     nvs_handle_t my_handle;
//     esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
//     if (err == ESP_OK) {
//         int32_t log_index;
//         size_t size = sizeof(log_index);
//         if (nvs_get_i32(my_handle, "log_index", &log_index) != ESP_OK) {
//             log_index = 0;
//         }
//         for (int i = 0; i < MAX_LOGS_NVS; i++) {
//             char key[10], log_message[100];
//             size_t len = sizeof(log_message);
//             sprintf(key, "log%d", i);
//             if (nvs_get_str(my_handle, key, log_message, &len) == ESP_OK) {
//                 ESP_LOGE("LOG_HISTORY", "Log %d: %s", i, log_message);
//             }
//         }
//         nvs_close(my_handle);
//     }
// }
// // Custom log handler
// int custom_log_handler(const char *format, va_list args) {
//     char log_message[256];  // Buffer to hold the formatted log message
//     int ret = vsnprintf(log_message, sizeof(log_message), format, args);

//     // Check log level to save only errors
//     if (ret >= 0 && (esp_log_level_get("MAIN") <= ESP_LOG_ERROR)) {
//         log_error_to_nvs(log_message);  // Save the error message to NVS
//     }

//     // Optionally, print to the console as well
//     vprintf(format, args);

//     return ret;
// }

// };

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include "freertos/event_groups.h"
#include "esp_task_wdt.h"
#include "soilMoist.h"
#include "mqtt_manager.h"
#include "npk.h"
#include "sht31.h"
#include "driver/i2c.h"
#include <string.h>
#include "esp_system.h"
#include "esp_event.h"
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

#define LOG_LOCAL_LEVEL ESP_LOG_ERROR
#define MAX_LOGS_NVS 5

#ifdef __cplusplus
extern "C" {
#endif

#include "wifi_manager.h"
#include "httpClient.h"

#ifdef __cplusplus
} // extern "C"
#endif

// TAGS for logging
static const char *WIFI_TAG = "wifi station";
static const char *JSON_TAG = "JSON object";
static const char *SHT_TAG = "SHT Ping";
static const char *MAIN_TAG = "MAIN";
static const char *PUB_TAG = "Publish topic";
static const char *MQTT_TAG = "MQTT";

TaskHandle_t myTaskHandle = NULL;

extern "C"
{ 

// Function Declarations
void ping_sht(void *arg);
void post_data(void *arg);
static esp_err_t i2c_master_init();
void read_logs_from_nvs();
void log_error_to_nvs(const char *error_message);
int custom_log_handler(const char *format, va_list args);

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

void app_main() {
    printf("Main\n");
    esp_log_set_vprintf(custom_log_handler);

    // Initialize NVS (Non-volatile storage for credentials)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    read_logs_from_nvs(); // Read Stored Error Logs

    // Initialize Wi-Fi
    wifi_init_sta();

    // Init MQTT Client
    mqtt_init();

    // Set DNS and test DNS resolution
    set_dns();
    struct addrinfo *res;
    int err = getaddrinfo("aeti3itrm7uta-ats.iot.eu-north-1.amazonaws.com", NULL, NULL, &res);
    if (err == 0) {
        ESP_LOGI("DNS", "Hostname resolved successfully!");
        freeaddrinfo(res);
    } else {
        ESP_LOGE("DNS", "Failed to resolve hostname. Error: %d", err);
    }

    // Initialize I2C
    ESP_LOGI(MAIN_TAG, "Initializing I2C...");
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to initialize I2C");
        return;
    }

    // Task to handle SHT sensor data
    xTaskCreatePinnedToCore(post_data, "posting SHT31", 4096, NULL, 5, &myTaskHandle, 1);
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

void post_data(void *arg) {
    SHT31Sensor sensor(I2C_MASTER_NUM, SHT31_I2C_ADDR);
    if (!sensor.init()) {
        vTaskDelete(NULL);
        return;
    }

    float temperature, humidity;

    while (1) {
        if (sensor.read(temperature, humidity)) {
            ESP_LOGE("MEM", "1st Free heap: %lu", esp_get_free_heap_size());
            ESP_LOGE("MEM", "1st Minimum Heap Size: %lu", esp_get_minimum_free_heap_size());

            ESP_LOGI(SHT_TAG, "Temperature: %.2f °C, Humidity: %.2f %%", temperature, humidity);

            cJSON *mqtt_data = cJSON_CreateObject();
            cJSON_AddNumberToObject(mqtt_data, "Temp", temperature);
            cJSON_AddNumberToObject(mqtt_data, "moistureVal", humidity);
            const char *my_json_string = cJSON_Print(mqtt_data);
            if (mqtt_publish(MQTT_TOPIC_PUB, my_json_string) != ESP_OK) {
                ESP_LOGE(MQTT_TAG, "MQTT client not Publishing");
            }
            free((void *)my_json_string);  // Free the memory used by JSON string
            cJSON_Delete(mqtt_data);

        } else {
            ESP_LOGE(SHT_TAG, "Failed to read temperature and humidity");
        }
        ESP_LOGE("MEM", "2nd Free heap: %lu", esp_get_free_heap_size());
        ESP_LOGE("MEM", "2nd Minimum Heap Size: %lu", esp_get_minimum_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(30000)); // Wait 30 seconds before next read
        
    }

    vTaskDelete(NULL);
}

void log_error_to_nvs(const char *error_message) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err == ESP_OK) {
        int32_t log_index;
        if (nvs_get_i32(my_handle, "log_index", &log_index) != ESP_OK) {
            log_index = 0;  // If no index exists, start at 0
        }
        char key[10];
        sprintf(key, "log%ld", log_index % MAX_LOGS_NVS);  // Circular storage
        nvs_set_str(my_handle, key, error_message);
        log_index++;
        nvs_set_i32(my_handle, "log_index", log_index);  // Save index
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}

void read_logs_from_nvs() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        int32_t log_index;
        if (nvs_get_i32(my_handle, "log_index", &log_index) != ESP_OK) {
            log_index = 0;
        }
        for (int i = 0; i < MAX_LOGS_NVS; i++) {
            char key[10], log_message[100];
            size_t len = sizeof(log_message);
            sprintf(key, "log%d", i);
            if (nvs_get_str(my_handle, key, log_message, &len) == ESP_OK) {
                ESP_LOGE("LOG_HISTORY", "Log %d: %s", i, log_message);
            }
        }
        nvs_close(my_handle);
    }
}

int custom_log_handler(const char *format, va_list args) {
    char log_message[256];
    int ret = vsnprintf(log_message, sizeof(log_message), format, args);

    // Check if it's an error or warning to log it into NVS
    if (ret >= 0 && (strstr(log_message, "ERROR") != nullptr || strstr(log_message, "WARN") != nullptr)) {
        log_error_to_nvs(log_message);
    }

    // Print to console as well
    vprintf(format, args);
    return ret;
}
};