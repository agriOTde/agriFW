#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_task_wdt.h"
#include "mqtt_manager.h"
#include "OTA_manager.h"
#include "generic_uart_driver.h"
#include "sht31.h"
#include "sht41.h"
#include "driver/i2c.h"
#include <string.h>
#include <vector>
#include <string>
#include <math.h>
#include "esp_system.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "cJSON.h"
#include "lwip/netdb.h"
#include "nvs_manager.h"

#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000
#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0


// Motor Related
#define MOTOR_PIN GPIO_NUM_27 
typedef struct {
    float humidity_snapshot;
    uint16_t duration;
} MotorTimerContext;


// Delay Presets
#define MQTT_PUBLISH_DELAY 180000
#define OTA_CHECK_DELAY 60000
#define PH_PING_DELAY 19000
#define PH_RECEIVE_DELAY 200 
#define SHT_READ_DELAY 15000 


//Constraint Presets
#define HUMID_CONSTRAINT 25.0


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
static const char *SHT_TAG = "SHT Ping";
static const char *PH_TAG = "PH sensor";
static const char *MAIN_TAG = "MAIN";
static const char *OTA_TAG = "OTA Update";
static const char *MQTT_TAG = "MQTT";
static const char *MOTOR_TAG = "Motor";


//Handle for all tasks
TaskHandle_t myTaskHandle = NULL;
//Handle for Motor Worker Task 
TaskHandle_t motor_worker_task_handle = NULL;

//Predefined Values for Data
sensor_data_t shared_data{
    0,       // temperature;
    0,       // humidity;  
    50,      // s_ph_value;  
    0,       // s_temp_value;  
    0,       // s_hum_value;  
    false    // new_data_ready;     
};


const std::vector<const char*> subscribe_topics = {
    MQTT_TOPIC_MOTOR_CMD,
    MQTT_TOPIC_MOTOR_SCHEDULE,
    MQTT_TOPIC_MOTOR_HUMIDITY,
    MQTT_TOPIC_OTA_CMD
};
std::vector<const char*> sub_topic_ptrs;
std::vector<std::string> sub_topics_str;


extern "C"
{ 

// ################# Function Declarations ####################
// Tasks
void ota_update_task(void *arg);
void ping_sht_task(void *arg);
void ping_ph_task(void *arg);
void post_mqtt_task(void *arg);
void drive_motor_task(void *arg);
void motor_worker_task(void *arg);

//other
static esp_err_t i2c_master_init();
void log_error_to_nvs(const char *error_message);
int custom_log_handler(const char *format, va_list args);
void motor_timer_callback(TimerHandle_t xTimer);
std::vector<std::string> id_appended_topics(const std::vector<const char*>& s_topics, const char * esp_id);
static TimerHandle_t motor_timer = NULL;
bool ota_flag = true;

// ############################################################

class SHTSensor {
    public:
        SHTSensor(i2c_port_t i2c_num, uint8_t address) : i2c_num(i2c_num), address(address) {}
    
        bool init_sht4x() {
            if (sht4x_init(i2c_num, address) == ESP_OK) {
                ESP_LOGI(SHT_TAG, "SHT31 sensor initialized successfully.");
                return true;
            } else {
                ESP_LOGE(SHT_TAG, "Failed to initialize SHT31 sensor.");
                return false;
            }
        }
        bool init_sht3x() {
            if (sht31_init(i2c_num, address) == ESP_OK) {
                ESP_LOGI(SHT_TAG, "SHT31 sensor initialized successfully.");
                return true;
            } else {
                ESP_LOGE(SHT_TAG, "Failed to initialize SHT31 sensor.");
                return false;
            }
        }

        bool read_sht4x(float &temperature, float &humidity) {
            return sht4x_read(&temperature, &humidity, SHT4X_PREC_HIGH) == ESP_OK;
        }
        bool read_sht3x(float &temperature, float &humidity) {
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

    
    // Create shared semaphore
    data_publish_mutex = xSemaphoreCreateMutex();
    shared_sub_data_mutex = xSemaphoreCreateMutex();
    ota_shared_mutex = xSemaphoreCreateMutex();

    if (xSemaphoreTake(shared_sub_data_mutex, pdMS_TO_TICKS(SHARED_SUB_DATA_MUTEX_DELAY) == pdTRUE)) {

        if(read_nvs_value(namespaces, all_keys, key_counts, NS_COUNT,"schedule","duration", TYPE_U32, &shared_sub_data.motor_duration) != ESP_OK)
            ESP_LOGE(MAIN_TAG, "Failed to read duration from NVS!");
        else{
            ESP_LOGI(MAIN_TAG, "duration = %lu",shared_sub_data.motor_duration);
        }
        if(read_nvs_value(namespaces, all_keys, key_counts, NS_COUNT,"schedule","timeperiod", TYPE_U32, &shared_sub_data.motor_timePeriod) != ESP_OK)
            ESP_LOGE(MAIN_TAG, "Failed to read timeperiod from NVS!");
        else
            ESP_LOGI(MAIN_TAG, "Timeperiod = %lu",shared_sub_data.motor_timePeriod);
        if(read_nvs_value(namespaces, all_keys, key_counts, NS_COUNT,"schedule","command", TYPE_I8, &shared_sub_data.motor_command) != ESP_OK)
            ESP_LOGE(MAIN_TAG, "Failed to read command from NVS!");
        else
            ESP_LOGI(MAIN_TAG, "Motor Command = %d",shared_sub_data.motor_command);
        esp_err_t temp_err = read_nvs_value(namespaces, all_keys, key_counts, NS_COUNT,"constraints","humidity", TYPE_I8, &shared_sub_data.humidity_constraint);
        if( temp_err != ESP_OK)
            ESP_LOGE(MAIN_TAG, "Failed to read Humidity from NVS!, Err = %X", temp_err);
        else
            ESP_LOGI(MAIN_TAG, "Motor Command = %d",shared_sub_data.humidity_constraint);
        xSemaphoreGive(shared_sub_data_mutex); 
    } else {
        ESP_LOGE(MAIN_TAG, "Failed to read values from NVS!");
    }

    // append espIDs to the topics
    const char * ESP_ID_CLIENT = get_esp_client_id();
    ESP_LOGI(MAIN_TAG, "My Chip ID: %s\n", ESP_ID_CLIENT);
    sub_topics_str = id_appended_topics(subscribe_topics, ESP_ID_CLIENT);


    for (auto& t : sub_topics_str){
        sub_topic_ptrs.push_back(t.c_str());  // safe, because sub_topics_str stays alive
    }

    for(auto t: sub_topic_ptrs){
        ESP_LOGI(MAIN_TAG, "Sub Topic: %s\n",t);
    }


    // Initialize Wi-Fi
    wifi_init_sta();

    // Init MQTT Client
    mqtt_init(sub_topic_ptrs.data(), sub_topic_ptrs.size());


    // Set DNS and test DNS resolution
    set_dns();

    // Initialize I2C
    ESP_LOGI(MAIN_TAG, "Initializing I2C...");
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "Failed to initialize I2C");
        return;
    }

    // Tasks
    xTaskCreatePinnedToCore(drive_motor_task, "Driving Motor", 4096, NULL, 5, &myTaskHandle, 0);
    xTaskCreatePinnedToCore(ping_ph_task, "Pinging PH sensor", 4096, NULL, 8, &myTaskHandle, 1);
    xTaskCreatePinnedToCore(ping_sht_task, "Pinging SHT", 4096, NULL, 7, &myTaskHandle, 1);
    xTaskCreatePinnedToCore(post_mqtt_task, "posting MQTT Data", 4096, NULL, 6, &myTaskHandle, 1);
    xTaskCreatePinnedToCore(ota_update_task, "OTA Update", 8192, NULL, 4, &myTaskHandle, 0);

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

    // #define FRAME_READ_HT_EC_PH   {0x01, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x09}
    // #define FRAME_READ_NPK_2  {0x02, 0x03, 0x00, 0x1E, 0x00, 0x03, 0x65, 0xFE}
    // #define FRAME_READ_first  {0x02, 0x03, 0x00, 0x00, 0x00, 0x01, 0xF9, 0x85}
    // #define FRAME_ENQ_SLAVE_ID    {0xFF, 0x03, 0x07, 0xD0, 0x00, 0x01, 0x91, 0x59}
    // #define FRAME_SET_SLAVE_ID_2  {0x02, 0x06, 0x07, 0xD0, 0x00, 0x01, 0xB8, 0xB8}
    // #define FRAME_SET_BAUD_9600   {0x01, 0x06, 0x07, 0xD1, 0x00, 0x02, 0x59, 0x46}

    uint8_t REQ_BYTE[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x09};
    uint8_t REQ_BYTE_2[] = {0x02, 0x03, 0x00, 0x1E, 0x00, 0x03, 0x65, 0xFE};
    uint8_t rx_buf[256];
    uint16_t humidity, temperature, pH, slaveID, N, P, K;
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
        uint8_t bytes_received = uart_device_receive(UART_NUM_2, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(PH_RECEIVE_DELAY));

        if (bytes_received > 0) {
            // ESP_LOGI(PH_TAG, "Bytes = ");
            // for(uint8_t i=0; i<12; i++){
            //     ESP_LOGI(PH_TAG, "0x%02X", rx_buf[i]);
            // }
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

        } else {
            ESP_LOGE(PH_TAG, "Sensor Receive Failed!\n");
        }

        vTaskDelay(pdMS_TO_TICKS(500));


        if (uart_device_send(UART_NUM_2, REQ_BYTE_2, sizeof(REQ_BYTE_2)) == ESP_OK) {
            
            ESP_LOGI(PH_TAG,"Pinged NPK Sensor!\n");
        } else {
            ESP_LOGE(PH_TAG,"Sensor Ping Failed!\n");
        }
        // vTaskDelay(pdMS_TO_TICKS(1000));
        bytes_received = uart_device_receive(UART_NUM_2, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(PH_RECEIVE_DELAY));

        if (bytes_received > 0) {
  
            N = (rx_buf[3] << 8) | rx_buf[4];  
            P = (rx_buf[5] << 8) | rx_buf[6];  
            K = (rx_buf[7] << 8) | rx_buf[8]; 

            // // Convert pH to a human-readable value (0.1 increment)
            // float N_value = pH / 10.0;
            // float P_value = humidity / 10.0;
            // float K_value = temperature / 10.0;

            ESP_LOGI(PH_TAG, "N: %d, P: %d, K: %d", N,P,K);

            // // Lock shared data for writing
            // xSemaphoreTake(data_publish_mutex, portMAX_DELAY);
            // shared_data.s_ph_value = pH_value;
            // shared_data.s_temp_value = temperature_value;
            // shared_data.s_hum_value = humidity_value;
            // shared_data.new_data_ready = true; // Mark new data as ready
            // xSemaphoreGive(data_publish_mutex); // Release lock
            
        } else {
            ESP_LOGE(PH_TAG, "NPK Sensor Receive Failed!\n");
        }
        vTaskDelay(pdMS_TO_TICKS(PH_PING_DELAY));
    }
    vTaskDelete(NULL);
}


void ping_sht_task(void *arg) {
    uint8_t sht_init_flag = 0;
    uint8_t sht_retries = 0;
    SHTSensor sensor(I2C_MASTER_NUM, SHT4X_I2C_ADDR_DEFAULT);
    if (!sensor.init_sht4x()) {
        ESP_LOGE(SHT_TAG, "SHT Failed to Initialize!");
        uint8_t sht_init_flag = 0;
        // vTaskDelete(NULL);
        // return;
    }
    else{
        sht_init_flag = 1;
    }

    float temperature, humidity;

    while (1) {

        if(!sht_init_flag && sht_retries < 3){
            if (!sensor.init_sht4x()) {
                ESP_LOGE(SHT_TAG, "Retry: SHT Failed to Initialize!");
                uint8_t sht_init_flag = 0;
                sht_retries += 1;
            }
            else{
                sht_init_flag = 1;
            }
        } else if(!sht_init_flag && sht_retries >= 3){
            ESP_LOGE(SHT_TAG, "SHT Failed to Initialize!, deleting task!");
            vTaskDelete(NULL);

        }
        

        if (sht_init_flag && sensor.read_sht4x(temperature, humidity)) {
            ESP_LOGE("MEM", "1st Free heap: %lu", esp_get_free_heap_size());
            ESP_LOGE("MEM", "1st Minimum Heap Size: %lu", esp_get_minimum_free_heap_size());

            ESP_LOGI(SHT_TAG, "Temperature: %.2f °C, Humidity: %.2f %%", temperature, humidity);

            xSemaphoreTake(data_publish_mutex, portMAX_DELAY);
            shared_data.temperature = temperature;
            shared_data.humidity = humidity;
            shared_data.new_data_ready = true; // Mark new data as ready
            xSemaphoreGive(data_publish_mutex); // Release lock

        } else {
            ESP_LOGE(SHT_TAG, "Failed to read temperature and humidity");
        }
        ESP_LOGE("MEM", "2nd Free heap: %lu", esp_get_free_heap_size());
        ESP_LOGE("MEM", "2nd Minimum Heap Size: %lu", esp_get_minimum_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(SHT_READ_DELAY)); // Wait 30 seconds before next read
        
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
            cJSON_AddItemToObject(mqtt_data, "espClientID", cJSON_CreateString(get_esp_client_id()));
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
    
    uint32_t duration = DEFAULT_MOTOR_DURATION;
    uint32_t last_duration = 0;
    uint32_t time_period = 0;
    uint32_t last_timer_period_ms = 0;
    int8_t current_motor_command = 0;
    int8_t previous_motor_state = 0;

    // GPIO Config
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Create worker task
    xTaskCreate(motor_worker_task, "motor_worker", 4096, NULL, 8, &motor_worker_task_handle);

    while (1) {
        vTaskDelay(1); //DEBUG DELAY
        // Access shared_sub_data safely
        if (xSemaphoreTake(shared_sub_data_mutex, pdMS_TO_TICKS(SHARED_SUB_DATA_MUTEX_DELAY)) == pdTRUE) {
            current_motor_command = shared_sub_data.motor_command;
            duration = shared_sub_data.motor_duration;
            time_period = shared_sub_data.motor_timePeriod;
            xSemaphoreGive(shared_sub_data_mutex);
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
    
        if (current_motor_command == 1 || current_motor_command == 0) {
            // Manual mode — clean up scheduled timer
            if (motor_timer != NULL) {
                xTimerStop(motor_timer, 0);
                xTimerDelete(motor_timer, 0);
                motor_timer = NULL;
                last_timer_period_ms = 0;
            }

            if (current_motor_command != previous_motor_state && current_motor_command == 1) {

                vTaskDelay(pdMS_TO_TICKS(100));

                gpio_set_level(MOTOR_PIN, 1);

                cJSON *ack = cJSON_CreateObject();
                cJSON_AddStringToObject(ack, "status", "True");
                const char *ack_str = cJSON_Print(ack);
                mqtt_publish(MQTT_TOPIC_PUB_ACK, ack_str);
                free((void *)ack_str);
                cJSON_Delete(ack);

                vTaskDelay(pdMS_TO_TICKS(duration));

                gpio_set_level(MOTOR_PIN, 0);
                ESP_LOGI(MOTOR_TAG, "Motor stopped after %lu ms (Manual)", duration);

                if (xSemaphoreTake(shared_sub_data_mutex, pdMS_TO_TICKS(SHARED_SUB_DATA_MUTEX_DELAY)) == pdTRUE) {
                    shared_sub_data.motor_command = 0;
                    xSemaphoreGive(shared_sub_data_mutex);
                }

                ack = cJSON_CreateObject();
                cJSON_AddStringToObject(ack, "status", "False");
                ack_str = cJSON_Print(ack);
                mqtt_publish(MQTT_TOPIC_PUB_ACK, ack_str);
                free((void *)ack_str);
                cJSON_Delete(ack);
            }

            previous_motor_state = current_motor_command;
        } else if (current_motor_command == -1){

            vTaskDelay(pdMS_TO_TICKS(100));

            if (motor_timer != NULL && (time_period != last_timer_period_ms || duration != last_duration)) {
                xTimerStop(motor_timer, 0);
                xTimerDelete(motor_timer, 0);
                motor_timer = NULL;
                ESP_LOGE(MOTOR_TAG, "Timer Deleted!");  

            }

            if (motor_timer == NULL) {
                MotorTimerContext *ctx = (MotorTimerContext *) malloc(sizeof(MotorTimerContext));
                ctx->duration = duration;
                motor_timer = xTimerCreate("MotorTimer", pdMS_TO_TICKS(time_period), pdTRUE, (void *)ctx, motor_timer_callback);
                ESP_LOGI(MOTOR_TAG, "New Timer Created! Timeperiod = %lu, duration = %lu ", time_period, duration);  
                xTimerStart(motor_timer, 0);
                last_timer_period_ms = time_period;
                last_duration = duration;
            }
        } else{
                continue;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void motor_timer_callback(TimerHandle_t xTimer) {
    MotorTimerContext *ctx = (MotorTimerContext *) pvTimerGetTimerID(xTimer);
    if (ctx) {
    xTaskNotifyGive(motor_worker_task_handle);  // Notify the worker task
    }
}

void motor_worker_task(void *arg) {
    float humidity = 100;
    float hum_const = HUMID_CONSTRAINT;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for timer notification

        MotorTimerContext *ctx = (MotorTimerContext *) pvTimerGetTimerID(motor_timer);

        if (!ctx) continue;
        if (xSemaphoreTake(data_publish_mutex, pdMS_TO_TICKS(DATA_PUBLISH_MUTEX_DELAY)) == pdTRUE) {
            if(shared_data.new_data_ready){
                humidity = shared_data.s_hum_value;
                xSemaphoreGive(data_publish_mutex);
                }
            }
        ESP_LOGI(MOTOR_TAG, "In Motor Worker Task, Humidity %.2f", humidity);
        
        if (xSemaphoreTake(shared_sub_data_mutex, pdMS_TO_TICKS(SHARED_SUB_DATA_MUTEX_DELAY)) == pdTRUE) {
            if(shared_sub_data.humidity_constraint){
                hum_const = shared_sub_data.humidity_constraint;
                ESP_LOGI(MOTOR_TAG, "Humidity Constraint %.2f", hum_const);
                }
            else {
                ESP_LOGE(MOTOR_TAG, "Invalid Humidity Constraint Value!");
            }
            xSemaphoreGive(shared_sub_data_mutex);

            }
        
        if (humidity < hum_const) {
            ESP_LOGI(MOTOR_TAG, "Humidity %.2f below threshold. Activating motor.", humidity);

            vTaskDelay(pdMS_TO_TICKS(10));

            ESP_LOGI(MOTOR_TAG, "Scheduled Motor start!");
            cJSON *ack = cJSON_CreateObject();
            cJSON_AddStringToObject(ack, "status", "True");
            const char *ack_str = cJSON_Print(ack);
            mqtt_publish(MQTT_TOPIC_PUB_ACK, ack_str);

            gpio_set_level(MOTOR_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(ctx->duration));
            gpio_set_level(MOTOR_PIN, 0);

            ESP_LOGI(MOTOR_TAG, "Motor stopped after scheduled run.");

            // Send acknowledgment and optionally store state in NVS
            ack = cJSON_CreateObject();
            cJSON_AddStringToObject(ack, "status", "False");
            ack_str = cJSON_Print(ack);
            mqtt_publish(MQTT_TOPIC_PUB_ACK, ack_str);

            free((void *)ack_str);
            cJSON_Delete(ack);
        }
    }
}



void ota_update_task(void *arg) {
    ESP_LOGI(OTA_TAG, "OTA Update Task Started");

    wifi_ap_record_t ap_info;
    bool ota_flag = false;
    const int max_retries = 30;
    int retry_count = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(OTA_CHECK_DELAY));
    
        ESP_LOGI(OTA_TAG, "OTA TASK BREAKPOINT");
    
        if (get_ota_flag(&ota_flag) != ESP_OK) {
            ESP_LOGE(OTA_TAG, "Failed to read OTA flag from NVS. Skipping this cycle.");
            // continue; // Allow task to retry later
        }
    
        if (xSemaphoreTake(ota_shared_mutex, portMAX_DELAY) == pdTRUE) {
            ota_flag = shared_ota_data.update_cmd;
            xSemaphoreGive(ota_shared_mutex);

            ESP_LOGI(OTA_TAG, "ota_flag = %d",ota_flag);

    
            if (!ota_flag) {
                ESP_LOGI(OTA_TAG, "OTA flag is false. Skipping OTA update.");
                continue;
            }
    
            retry_count = 0;  // Reset retry counter
            while (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK) {
                ESP_LOGW(OTA_TAG, "Wi-Fi not connected, retrying... (%d/%d)", retry_count + 1, max_retries);
                retry_count++;
                if (retry_count >= max_retries) {
                    ESP_LOGE(OTA_TAG, "Wi-Fi connection failed after max retries. Skipping OTA.");
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
    
            if (retry_count < max_retries) {
                if (ota_update()) {

                    ESP_LOGI(OTA_TAG, "OTA update successful!");
                    set_ota_flag(false); //Update NVS Memory

                    xSemaphoreTake(ota_shared_mutex, portMAX_DELAY);
                    shared_ota_data.update_cmd = false;
                    xSemaphoreGive(ota_shared_mutex);

                    // Send acknowledgment to broker
                    cJSON *ota_ack_data = cJSON_CreateObject();
                    cJSON_AddStringToObject(ota_ack_data, "OTA_Status", "OTA Updated");
                    const char *ota_ack_json_string = cJSON_Print(ota_ack_data);

                    if (mqtt_publish(MQTT_TOPIC_OTA_ACK, ota_ack_json_string) != ESP_OK) {
                        ESP_LOGE(MOTOR_TAG, "Failed to publish OTA Update acknowledgment");
                    } else {
                        ESP_LOGI(MOTOR_TAG, "Published OTA Update acknowledgment");
                    }

                    // Free memory used by JSON string and object
                    free((void *)ota_ack_json_string);
                    cJSON_Delete(ota_ack_data);

                    // Restart ESP
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    esp_restart();

                } else {
                    ESP_LOGE(OTA_TAG, "OTA update failed.");

                    // Send acknowledgment to broker
                    cJSON *ota_ack_data = cJSON_CreateObject();
                    cJSON_AddStringToObject(ota_ack_data, "OTA_Status", "OTA Failed");
                    const char *ota_ack_json_string = cJSON_Print(ota_ack_data);

                    if (mqtt_publish(MQTT_TOPIC_OTA_ACK, ota_ack_json_string) != ESP_OK) {
                        ESP_LOGE(MOTOR_TAG, "Failed to publish OTA Update FAIL acknowledgment");
                    } else {
                        ESP_LOGI(MOTOR_TAG, "Published OTA Update FAIL acknowledgment");
                    }

                    // Free memory used by JSON string and object
                    free((void *)ota_ack_json_string);
                    cJSON_Delete(ota_ack_data);
                }
            }
    
            xSemaphoreGive(ota_shared_mutex);
    
        } else {
            ESP_LOGE(OTA_TAG, "Failed to take OTA semaphore.");
        }
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

std::vector<std::string> id_appended_topics(const std::vector<const char*>& s_topics, const char * esp_id){

    std::vector<std::string> temp_topics;
    temp_topics.reserve(s_topics.size());


    for (auto base : s_topics) {
        std::string topic = std::string(esp_id) + "/" + base;
        temp_topics.push_back(topic);
    }

    return temp_topics;

}

};