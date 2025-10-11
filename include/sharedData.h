/**
 * @file shared_data.h
 * @brief A single place to keep all shared data between tasks.
 *
 * This file holds the common settings, sensor readings, and flags used across
 * the ESP32 project — like motor commands, OTA updates, and sensor data.
 * It also defines the MQTT topics we talk on, and the mutexes that keep
 * everything thread-safe. Think of it as the “shared notebook” for your tasks.
 */


#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "cJSON.h"
#include <stdbool.h>


// Data Structs
typedef struct {
    int8_t motor_command;
    uint32_t motor_timePeriod;
    uint32_t motor_duration;
    int8_t humidity_constraint;
    // Add other flags here if needed
} shared_sub_data_t;

typedef struct {
    bool update_cmd;;
    // Add other flags here if needed
} shared_ota_t;


typedef struct {
    float temperature;
    float humidity;
    float s_ph_value;
    float s_temp_value;
    float s_hum_value;
    bool new_data_ready;  // Flag to indicate new data is available
} sensor_data_t;

// Publish Topics
#define MQTT_TOPIC_PUB "esp32/pub"
#define MQTT_TOPIC_PUB_ACK "esp32/motor/ack"

// Subscription Topics
#define MQTT_TOPIC_MOTOR_CMD "esp32/motor/command"
#define MQTT_TOPIC_MOTOR_SCHEDULE "esp32/motor/schedule"
#define MQTT_TOPIC_MOTOR_HUMIDITY "esp32/motor/humidity"
#define MQTT_TOPIC_OTA_CMD "esp32/ota/command"
#define MQTT_TOPIC_OTA_ACK "esp32/ota/ack"


#define DEFAULT_MOTOR_DURATION 5000
#define DEFAULT_MOTOR_TIMEPERIOD 2*60*1000


// Mutex wait delays
#define SHARED_SUB_DATA_MUTEX_DELAY 300
#define DATA_PUBLISH_MUTEX_DELAY 250
#define OTA_SHARED_MUTEX_DELAY 250



extern sensor_data_t shared_data;
extern shared_sub_data_t shared_sub_data;
extern shared_ota_t shared_ota_data;

extern SemaphoreHandle_t data_publish_mutex;
extern SemaphoreHandle_t shared_sub_data_mutex;
extern SemaphoreHandle_t ota_shared_mutex;

// All NVS namespaces and their keys
#define NS_COUNT 2
#define MAX_KEYS_PER_NS 3   
extern const char* namespaces[NS_COUNT];
extern const char* all_keys[NS_COUNT][MAX_KEYS_PER_NS];
extern const int key_counts[NS_COUNT];



// #### METHODS
char * get_esp_client_id(void);

#ifdef __cplusplus
}
#endif

#endif // SHARED_DATA_H