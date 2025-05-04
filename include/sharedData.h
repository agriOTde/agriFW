#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "cJSON.h"
#include <stdbool.h>

#define MQTT_TOPIC_PUB "esp32/pub"
#define MQTT_TOPIC_PUB_ACK "esp32/motor/ack"
#define MQTT_TOPIC_MOTOR_CMD "esp32/motor/command"
#define MQTT_TOPIC_MOTOR_SCHEDULE "esp32/motor/schedule"
#define MQTT_TOPIC_OTA_CMD "esp32/ota/command"
#define MQTT_TOPIC_OTA_ACK "esp32/ota/ack"


typedef struct {
    bool motor_command;
    uint32_t motor_timePeriod;
    uint16_t motor_duration;
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

extern sensor_data_t shared_data;
extern shared_sub_data_t shared_sub_data;
extern shared_ota_t shared_ota_data;

extern SemaphoreHandle_t data_publish_mutex;
extern SemaphoreHandle_t shared_sub_data_mutex;
extern SemaphoreHandle_t ota_shared_mutex;



#endif // SHARED_DATA_H