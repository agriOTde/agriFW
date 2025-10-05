#include "mqtt_manager.h"
#include "nvs_manager.h"
#include "esp_log.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"
#include "mqtt_client.h"
#include "sharedData.h"
#include "driver/gpio.h"

// #define LOG_LOCAL_LEVEL ESP_LOG_ERROR

#define MQTT_LED_PIN GPIO_NUM_4
#define MQTT_LED_TIMEPERIOD 100
static TaskHandle_t mqtt_blink_task_handle = NULL;

gpio_config_t mqtt_led_conf = {
    .pin_bit_mask = (1ULL << MQTT_LED_PIN),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

static const char *TAG = "MQTT";
static const char *MQTT_PARSE = "MQTT Payload";

static esp_mqtt_client_handle_t client;  // MQTT client handle
static bool mqtt_connected = false;
static const char **subscribed_topics = NULL;
static int num_subscribed_topics = 0;

shared_ota_t shared_ota_data;

// MQTT event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;

    
    
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            mqtt_connected = true;

            if (mqtt_blink_task_handle) {
                vTaskDelete(mqtt_blink_task_handle);
                mqtt_blink_task_handle = NULL;
            }
        
            // Set LED solid ON
            gpio_set_level(MQTT_LED_PIN, 1);

            for (int i = 0; i < num_subscribed_topics; ++i) {
                int msg_id = esp_mqtt_client_subscribe(client, subscribed_topics[i], 0);
                ESP_LOGI(TAG, "Subscribed to topic: %s, msg_id: %d", subscribed_topics[i], msg_id);
            }

            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            if (mqtt_blink_task_handle == NULL) {
                xTaskCreatePinnedToCore(mqtt_led_blink_task, "mqtt_led_blink_task", 2048, NULL, 1, &mqtt_blink_task_handle, 1);
                ESP_LOGI(TAG, "LED task created");

            }
            mqtt_connected = false;
            break;
        case MQTT_EVENT_DATA:{
            // ESP_LOGI(TAG, "Received message on topic %s: %d: %.*s", event->topic, event->data_len, event->data);
                // Copy topic and data into null-terminated strings
                
            char topic[event->topic_len + 1];
            char data[event->data_len + 1];
            memcpy(topic, event->topic, event->topic_len);
            topic[event->topic_len] = '\0';
            memcpy(data, event->data, event->data_len);
            data[event->data_len] = '\0';
            ESP_LOGI(TAG, "DEBUG: incoming topic='%s' expected='%s'", topic, MQTT_TOPIC_MOTOR_HUMIDITY);


            if (strcmp(topic, MQTT_TOPIC_MOTOR_SCHEDULE) == 0) {
                // Parse JSON for schedule (esp32/motor/schedule)
                parseScheduleJson(data);
            } else if (strcmp(topic, MQTT_TOPIC_MOTOR_CMD) == 0) {
                // Parse JSON for command (esp32/motor/command)
                parseCommandJson(data);
            } else if (strcmp(topic, MQTT_TOPIC_OTA_CMD) == 0) {
                // Parse JSON for command (esp32/motor/command)
                parseOTACommandJson(data);
            } else if (strcmp(topic, MQTT_TOPIC_MOTOR_HUMIDITY) == 0) {
                // Parse JSON for command (esp32/motor/command)
                ESP_LOGI(TAG, "DEBUF: PARSE Humidity");

                parseHumSetPointJson(data);
            }

            break;
            }
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "Message published successfully");
            break;
        default:
            ESP_LOGI(TAG, "MQTT Event: %d", event->event_id);
            break;
    }
}

// Initialize MQTT
void mqtt_init(const char **topics, int num_topics) {

    gpio_config(&mqtt_led_conf);
    gpio_set_level(MQTT_LED_PIN, 0);

    subscribed_topics = topics;
    num_subscribed_topics = num_topics;

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = RPI_MQTT_BROKER_URI,
        .network.timeout_ms = 5000,
        .task.stack_size = 8192,
        .task.priority = 10
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}
// Publish a message
uint16_t mqtt_publish(const char *topic, const char *message) {
    if (client && mqtt_connected) {
        return esp_mqtt_client_publish(client, topic, message, 0, 0, 0);
    } else {
        ESP_LOGW("MQTT", "Client not connected — skipping publish");
        return ESP_FAIL;
    }
}

void mqtt_reconnect(void) {
    if (client) {
        esp_mqtt_client_stop(client);
        esp_mqtt_client_start(client);
        ESP_LOGI(TAG, "MQTT client restarted after Wi-Fi reconnect");
    }
}

void parseScheduleJson(const char *jsonBuffer) {
    // Parse the JSON string into a cJSON object
    cJSON *json = cJSON_Parse(jsonBuffer);
    if (json == NULL) {
        ESP_LOGE(MQTT_PARSE, "Error parsing JSON: %s", jsonBuffer);
        return;
    }

    // Extract values from the JSON object
    cJSON *duration = cJSON_GetObjectItemCaseSensitive(json, "Duration");
    cJSON *timePeriod = cJSON_GetObjectItemCaseSensitive(json, "TimePeriod");

    if (cJSON_IsNumber(duration)) {
        ESP_LOGI(MQTT_PARSE, "Received Duration: %d", duration->valueint);
    }
    if (cJSON_IsNumber(timePeriod)) {
        ESP_LOGI(MQTT_PARSE, "Received TimePeriod: %d", timePeriod->valueint);
    }



    // Update shared data (ensure mutex is taken to safely modify shared data)
    if (cJSON_IsNumber(duration) && cJSON_IsNumber(timePeriod)) {
        if (xSemaphoreTake(shared_sub_data_mutex, pdMS_TO_TICKS(SHARED_SUB_DATA_MUTEX_DELAY))==pdTRUE) {
            shared_sub_data.motor_duration = duration->valueint;
            shared_sub_data.motor_timePeriod = timePeriod->valueint;
            xSemaphoreGive(shared_sub_data_mutex);
            uint32_t tp = (uint32_t)timePeriod->valueint;
            store_values("schedule","timeperiod", TYPE_U32, &tp);
            uint32_t dur = (uint32_t)duration->valueint;
            store_values("schedule","duration", TYPE_U32, &dur);
        } else {
            ESP_LOGE(MQTT_PARSE, "Parse Schedule SEMAPHORE DID NOT YIELD");
        }
    }
    else {
        ESP_LOGE(MQTT_PARSE, "Invalid or missing Duration/TimePeriod in JSON");
    }
    // Clean up the cJSON object
    cJSON_Delete(json);
}

void parseCommandJson(const char *jsonBuffer) {
    // Parse the JSON string into a cJSON object
    cJSON *json = cJSON_Parse(jsonBuffer);
    if (json == NULL) {
        ESP_LOGE(MQTT_PARSE, "Error parsing JSON: %s", jsonBuffer);
        return;
    }

    // Extract the "cmd" item from the JSON object
    cJSON *cmd = cJSON_GetObjectItemCaseSensitive(json, "cmd");

    // Check if the cmd is a valid string
    if (cJSON_IsNumber(cmd)) {
        ESP_LOGI(MQTT_PARSE, "Received Command: %d", cmd->valueint);
    } else {
        ESP_LOGE(MQTT_PARSE, "Invalid command in JSON, 'cmd' is not a number or is NULL.");
        cJSON_Delete(json);
        return;
    }



    // Try to update the shared data (ensure mutex is taken to safely modify shared data)
    if (cJSON_IsNumber(cmd)){
            if (xSemaphoreTake(shared_sub_data_mutex, pdMS_TO_TICKS(SHARED_SUB_DATA_MUTEX_DELAY)) == pdTRUE) {
            ESP_LOGE(MQTT_PARSE, "SEMAPHORE YIELDED");

            // Update the shared data based on the command
            if ((cmd->valueint) == 1) {
                ESP_LOGI(MQTT_PARSE, "Setting motor_command to true");
                shared_sub_data.motor_command = true;
                int m_cmd = -2;
                store_values("schedule","command", TYPE_I8,&m_cmd);
            } else if ((cmd->valueint) == 0) {
                ESP_LOGI(MQTT_PARSE, "Setting motor_command to false");
                shared_sub_data.motor_command = false;
            } else if ((cmd->valueint) == -1) {
                ESP_LOGI(MQTT_PARSE, "Setting motor_command to Scheduled");
                shared_sub_data.motor_command = -1;                
                store_values("schedule","command", TYPE_I8,&cmd->valueint);

            }else {
                ESP_LOGE(MQTT_PARSE, "Received unknown command: %d", cmd->valueint);
            }
            // Release the semaphore
            xSemaphoreGive(shared_sub_data_mutex);
        } else {
            ESP_LOGE(MQTT_PARSE, "Parse CMD SEMAPHORE DID NOT YIELD");
        }
    } else {
        ESP_LOGE(MQTT_PARSE, "Invalid or missing Command in JSON");
    }
    // Clean up the cJSON object
    cJSON_Delete(json);
}

void parseHumSetPointJson(const char *jsonBuffer) {
    // Parse the JSON string into a cJSON object
    cJSON *json = cJSON_Parse(jsonBuffer);
    if (json == NULL) {
        ESP_LOGE(MQTT_PARSE, "Error parsing JSON: %s", jsonBuffer);
        return;
    }

    // Extract the "cmd" item from the JSON object
    cJSON *cmd = cJSON_GetObjectItemCaseSensitive(json, "humidity");

    // Check if the cmd is a valid string
    if (cJSON_IsNumber(cmd)) {
        ESP_LOGI(MQTT_PARSE, "Received Humidity: %d", cmd->valueint);
    } else {
        ESP_LOGE(MQTT_PARSE, "Invalid command in JSON, 'Humidity' is not a number or is NULL.");
        cJSON_Delete(json);
        return;
    }



    // Try to update the shared data (ensure mutex is taken to safely modify shared data)
    if (cJSON_IsNumber(cmd)){
            if (xSemaphoreTake(shared_sub_data_mutex, pdMS_TO_TICKS(SHARED_SUB_DATA_MUTEX_DELAY)) == pdTRUE) {
            // ESP_LOGE(MQTT_PARSE, "SEMAPHORE YIELDED");

            // Update the shared data based on the command
            if ((cmd->valueint) > 0 && (cmd->valueint) <= 100  ) {
                ESP_LOGE(MQTT_PARSE, "Stored Humidity");
                shared_sub_data.humidity_constraint = cmd->valueint;
                store_values("constraints","humidity", TYPE_I8,&cmd->valueint);
            } else {
                ESP_LOGI(MQTT_PARSE, "Invalid Humidity Value");
            }
            // Release the semaphore
            xSemaphoreGive(shared_sub_data_mutex);
        } else {
            ESP_LOGE(MQTT_PARSE, "Parse Humidity SEMAPHORE DID NOT YIELD");
        }
    } else {
        ESP_LOGE(MQTT_PARSE, "Invalid or missing Command in JSON");
    }
    // Clean up the cJSON object
    cJSON_Delete(json);
}
void parseOTACommandJson(const char *jsonBuffer) {
    // Parse the JSON string into a cJSON object
    cJSON *json = cJSON_Parse(jsonBuffer);
    if (json == NULL) {
        ESP_LOGE(MQTT_PARSE, "Error parsing JSON: %s", jsonBuffer);
        return;
    }

    // Extract the "cmd" item from the JSON object
    cJSON *cmd = cJSON_GetObjectItemCaseSensitive(json, "cmd");

    // Check if the cmd is a valid string
    if (cJSON_IsNumber(cmd)) {
        ESP_LOGI(MQTT_PARSE, "Received Command: %d", cmd->valueint);
    } else {
        ESP_LOGE(MQTT_PARSE, "Invalid command in JSON, 'cmd' is not a number or is NULL.");
        cJSON_Delete(json);
        return;
    }



    // Try to update the shared data (ensure mutex is taken to safely modify shared data)
    if (cJSON_IsNumber(cmd)){
            if (xSemaphoreTake(ota_shared_mutex, pdMS_TO_TICKS(OTA_SHARED_MUTEX_DELAY)) == pdTRUE) {
            // ESP_LOGE(MQTT_PARSE, "SEMAPHORE YIELDED");

            // Update the shared data based on the command
            if ((cmd->valueint) == 1) {
                ESP_LOGI(MQTT_PARSE, "Setting OTA_command to true");
                shared_ota_data.update_cmd = true;
            } else if ((cmd->valueint) == 0) {
                ESP_LOGI(MQTT_PARSE, "Setting OTA_command to false");
                shared_ota_data.update_cmd = false;
            } else {
                ESP_LOGE(MQTT_PARSE, "Received unknown command: %d", cmd->valueint);
            }
            // Release the semaphore
            xSemaphoreGive(ota_shared_mutex);
        } else {
            ESP_LOGE(MQTT_PARSE, "Parse CMD SEMAPHORE DID NOT YIELD");
        }
    } else {
        ESP_LOGE(MQTT_PARSE, "Invalid or missing Command in JSON");
    }
    // Clean up the cJSON object
    cJSON_Delete(json);
}

void mqtt_led_blink_task(void *arg) {
    while (1) {
        gpio_set_level(MQTT_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(MQTT_LED_TIMEPERIOD));
        gpio_set_level(MQTT_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(MQTT_LED_TIMEPERIOD));
    }
}