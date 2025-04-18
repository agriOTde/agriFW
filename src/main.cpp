#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"
#include "mqtt_manager.h"
#include "sharedData.h"
#include "generic_uart_driver.h"
#include "sht31.h"
#include "driver/i2c.h"
#include <string.h>
#include <math.h>
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
#define MOTOR_PIN GPIO_NUM_12 /*!< GPIO number for Motor */
#define MQTT_PUBLISH_DELAY 2000 /*!< GPIO number for Motor */


const char *subscribe_topics[] = {
    MQTT_TOPIC_SUB
};

// #define LOG_LOCAL_LEVEL ESP_LOG_ERROR
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
static const char *PH_TAG = "PH sensor";
static const char *MAIN_TAG = "MAIN";
static const char *PUB_TAG = "Publish topic";
static const char *MQTT_TAG = "MQTT";
static const char *MOTOR_TAG = "Motor";

TaskHandle_t myTaskHandle = NULL;
sensor_data_t shared_data;

extern "C"
{ 

// ################# Function Declarations ####################
// Tasks
void ping_sht_task(void *arg);
void ping_ph_task(void *arg);
void post_mqtt_task(void *arg);
void drive_motor_task(void *arg);

//other
static esp_err_t i2c_master_init();
void read_logs_from_nvs();
void log_error_to_nvs(const char *error_message);
int custom_log_handler(const char *format, va_list args);

// ############################################################

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
    mqtt_init(subscribe_topics, sizeof(subscribe_topics)/sizeof(subscribe_topics[0]));


    // Set DNS and test DNS resolution
    set_dns();

    // Create shared semaphore
    data_publish_mutex = xSemaphoreCreateMutex();
    shared_sub_data_mutex = xSemaphoreCreateMutex();

    // Initialize I2C
    ESP_LOGI(MAIN_TAG, "Initializing I2C...");
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to initialize I2C");
        return;
    }

    // Tasks
    xTaskCreatePinnedToCore(drive_motor_task, "Pinging PH sensor", 4096, NULL, 3, &myTaskHandle, 1);
    xTaskCreatePinnedToCore(ping_ph_task, "Pinging PH sensor", 4096, NULL, 4, &myTaskHandle, 1);
    xTaskCreatePinnedToCore(ping_sht_task, "posting SHT31", 4096, NULL, 5, &myTaskHandle, 1);
    xTaskCreatePinnedToCore(post_mqtt_task, "posting MQTT Data", 4096, NULL, 6, &myTaskHandle, 1);
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

void ping_ph_task(void *arg){

    uint8_t REQ_BYTE[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x09};
    uint8_t rx_buf[256];
    uint16_t humidity, temperature, pH;
    // UART configuration
    uart_device_config_t uart_config = {
        .uart_port = UART_NUM_2,
        .tx_pin = GPIO_NUM_17,
        .rx_pin = GPIO_NUM_16,
        .baud_rate = 4800
    };

    // Initialize UART
    if (uart_device_init(&uart_config) != ESP_OK) {
        ESP_LOGE(PH_TAG,"Failed to initialize UART\n");
        return;
    }
    else{
        ESP_LOGI(PH_TAG,"Initialized UART!\n");
    }
    while(1){

        if (uart_device_send(UART_NUM_2, REQ_BYTE, sizeof(REQ_BYTE)) == ESP_OK) {
            
            ESP_LOGI(PH_TAG,"Pinged Sensor!\n");
        } else {
            ESP_LOGE(PH_TAG,"Sensor Ping Failed!\n");
        }
        // vTaskDelay(pdMS_TO_TICKS(1000));
        uint8_t bytes_received = uart_device_receive(UART_NUM_2, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(5000));

        if (bytes_received > 0) {
            ESP_LOGI(PH_TAG, "Bytes = ");
            for(uint8_t i=0; i<12; i++){
                ESP_LOGI(PH_TAG, "0x%02X", rx_buf[i]);
            }
            // Example of reading data from the response frame
            humidity = (rx_buf[3] << 8) | rx_buf[4];  // Combine low and high bytes for humidity
            temperature = (rx_buf[5] << 8) | rx_buf[6];  // Combine low and high bytes for temperature
            pH = (rx_buf[9] << 8) | rx_buf[10];  // Combine low and high bytes for pH value

            // Convert pH to a human-readable value (0.1 increment)
            float pH_value = pH / 10.0;
            float humidity_value = humidity / 10.0;
            float temperature_value = temperature / 10.0;

            ESP_LOGI(PH_TAG, "Humidity: %.1f%%, Temperature: %.2f deg_C, pH: %.1f", humidity_value, temperature_value, pH_value);

            // Lock shared data for writing
            xSemaphoreTake(data_publish_mutex, portMAX_DELAY);
            shared_data.s_ph_value = pH_value;
            shared_data.s_temp_value = temperature_value;
            shared_data.s_hum_value = humidity_value;
            shared_data.new_data_ready = true; // Mark new data as ready
            xSemaphoreGive(data_publish_mutex); // Release lock

            // Store or process the values (e.g., send to MQTT, store in NVS, etc.)
            // cJSON *mqtt_data = cJSON_CreateObject();
            // cJSON_AddNumberToObject(mqtt_data, "Humidity", humidity);
            // cJSON_AddNumberToObject(mqtt_data, "Temperature", temperature);
            // cJSON_AddNumberToObject(mqtt_data, "Conductivity", conductivity);
            // cJSON_AddNumberToObject(mqtt_data, "pH", pH_value);

            // const char *json_string = cJSON_Print(mqtt_data);
            // if (mqtt_publish(MQTT_TOPIC_PUB, json_string) != ESP_OK) {
            //     ESP_LOGE(MQTT_TAG, "Failed to publish data.");
            // }
            // free((void *)json_string);  // Free the JSON string memory
            // cJSON_Delete(mqtt_data);
        } else {
            ESP_LOGE(PH_TAG, "Sensor Receive Failed!\n");
        }
            
    }
    vTaskDelete(NULL);
}


void ping_sht_task(void *arg) {
    SHT31Sensor sensor(I2C_MASTER_NUM, SHT31_I2C_ADDR);
    if (!sensor.init()) {
        ESP_LOGE(SHT_TAG, "SHT Failed to Initialize!");
        vTaskDelete(NULL);
        return;
    }

    float temperature, humidity;

    while (1) {
        if (sensor.read(temperature, humidity)) {
            ESP_LOGE("MEM", "1st Free heap: %lu", esp_get_free_heap_size());
            ESP_LOGE("MEM", "1st Minimum Heap Size: %lu", esp_get_minimum_free_heap_size());

            ESP_LOGI(SHT_TAG, "Temperature: %.2f °C, Humidity: %.2f %%", temperature, humidity);

            xSemaphoreTake(data_publish_mutex, portMAX_DELAY);
            shared_data.temperature = temperature;
            shared_data.humidity = humidity;
            shared_data.new_data_ready = true; // Mark new data as ready
            xSemaphoreGive(data_publish_mutex); // Release lock

            // cJSON *mqtt_data = cJSON_CreateObject();
            // cJSON_AddNumberToObject(mqtt_data, "Temp", temperature);
            // cJSON_AddNumberToObject(mqtt_data, "moistureVal", humidity);
            // const char *my_json_string = cJSON_Print(mqtt_data);
            // if (mqtt_publish(MQTT_TOPIC_PUB, my_json_string) != ESP_OK) {
            //     ESP_LOGE(MQTT_TAG, "MQTT client not Publishing");
            // }
            // free((void *)my_json_string);  // Free the memory used by JSON string
            // cJSON_Delete(mqtt_data);

        } else {
            ESP_LOGE(SHT_TAG, "Failed to read temperature and humidity");
        }
        ESP_LOGE("MEM", "2nd Free heap: %lu", esp_get_free_heap_size());
        ESP_LOGE("MEM", "2nd Minimum Heap Size: %lu", esp_get_minimum_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(30000)); // Wait 30 seconds before next read
        
    }

    vTaskDelete(NULL);
}

void post_mqtt_task(void *arg){

    while (1) {
        //Time period
        vTaskDelay(pdMS_TO_TICKS(MQTT_PUBLISH_DELAY)); // Wait 30 seconds before next read

        // Wait for new data from either task
        if (shared_data.new_data_ready) {
            // Lock shared data for reading
            xSemaphoreTake(data_publish_mutex, portMAX_DELAY);

            // Add debugging log to verify values
            ESP_LOGI(MQTT_TAG, "Preparing to send MQTT data: Temp = %.2f, Hum = %.2f, pH = %.2f",
                shared_data.s_temp_value, shared_data.s_hum_value, shared_data.s_ph_value);

                        // Round values to two decimal places before adding them to JSON
            shared_data.temperature = round(shared_data.temperature * 100.0) / 100.0;
            shared_data.humidity = round(shared_data.humidity * 100.0) / 100.0;
            shared_data.s_hum_value = round(shared_data.s_hum_value * 100.0) / 100.0;
            shared_data.s_temp_value = round(shared_data.s_temp_value * 100.0) / 100.0;
            shared_data.s_ph_value = round(shared_data.s_ph_value * 100.0) / 100.0;


            // Prepare the MQTT message
            cJSON *mqtt_data = cJSON_CreateObject();
            cJSON_AddNumberToObject(mqtt_data, "tempVal", shared_data.temperature);
            cJSON_AddNumberToObject(mqtt_data, "humVal", shared_data.humidity);
            cJSON_AddNumberToObject(mqtt_data, "sHumVal", shared_data.s_hum_value);
            cJSON_AddNumberToObject(mqtt_data, "sTempVal", shared_data.s_temp_value);
            cJSON_AddNumberToObject(mqtt_data, "sPhVal", shared_data.s_ph_value);

            // Publish the message
            const char *mqtt_json_string = cJSON_Print(mqtt_data);
            if (mqtt_publish(MQTT_TOPIC_PUB, mqtt_json_string) != ESP_OK) {
                ESP_LOGE(MQTT_TAG, "Failed to publish data");
            }
            ESP_LOGI(MQTT_TAG, "Published data: %s", mqtt_json_string);

            // Free the JSON string and delete object
            free((void *)mqtt_json_string);
            cJSON_Delete(mqtt_data);

            // Clear the new data flag
            shared_data.new_data_ready = false;

            // Release lock
            xSemaphoreGive(data_publish_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay before checking again
    }

    vTaskDelete(NULL);

}

void drive_motor_task(void *arg) {

       // 1. Configure the GPIO pin
       gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_PIN),  // Bitmask for the pin
        .mode = GPIO_MODE_OUTPUT,           // Set as output mode
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE      // No interrupts
    };
    gpio_config(&io_conf);

    bool previous_motor_state = false;

    while (1) {
        // Lock shared data to read motor state
        xSemaphoreTake(shared_sub_data_mutex, portMAX_DELAY);

        bool current_motor_state = shared_sub_data.motor_command;

        // If the motor state has changed
        if (current_motor_state != previous_motor_state) {
            if (current_motor_state == true) {
                // Start motor if not already started
                ESP_LOGI(MOTOR_TAG, "Motor Start Command received");
                gpio_set_level(MOTOR_PIN, 1);  // Set pin high (motor on)

                // Send acknowledgment to broker
                cJSON *ack_data = cJSON_CreateObject();
                cJSON_AddStringToObject(ack_data, "status", "Motor Started");

                const char *ack_json_string = cJSON_Print(ack_data);
                if (mqtt_publish(MQTT_TOPIC_PUB_ACK, ack_json_string) != ESP_OK) {
                    ESP_LOGE(MOTOR_TAG, "Failed to publish motor start acknowledgment");
                } else {
                    ESP_LOGI(MOTOR_TAG, "Published motor start acknowledgment");
                }

                free((void *)ack_json_string);
                cJSON_Delete(ack_data);

            } else {

                // Stop motor if not already stopped
                ESP_LOGI(MOTOR_TAG, "Motor Stop Command received");
                gpio_set_level(MOTOR_PIN, 0);  // Set pin low (motor off)

                // Send acknowledgment to broker
                cJSON *ack_data = cJSON_CreateObject();
                cJSON_AddStringToObject(ack_data, "status", "Motor Stopped");
                const char *ack_json_string = cJSON_Print(ack_data);

                if (mqtt_publish(MQTT_TOPIC_PUB_ACK, ack_json_string) != ESP_OK) {
                    ESP_LOGE(MOTOR_TAG, "Failed to publish motor stop acknowledgment");
                } else {
                    ESP_LOGI(MOTOR_TAG, "Published motor stop acknowledgment");
                }

                free((void *)ack_json_string);
                cJSON_Delete(ack_data);
            }

            // Update previous state
            previous_motor_state = current_motor_state;
        }

        // Release the lock after checking the state
        xSemaphoreGive(shared_sub_data_mutex);

        // Check the motor state every 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
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