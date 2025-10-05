#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_system.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_err.h"
#include "sharedData.h"

#define RPI_MQTT_BROKER_URI "mqtt://10.144.0.143:1884"


//Methods

void mqtt_init(const char **topics, int num_topics);
uint16_t mqtt_publish(const char *topic, const char *message);
void mqtt_reconnect(void);
void parseScheduleJson(const char *jsonBuffer);
void parseCommandJson(const char *jsonBuffer);
void parseOTACommandJson(const char *jsonBuffer);
void parseHumSetPointJson(const char *jsonBuffer);
void mqtt_led_blink_task(void *arg);

#ifdef __cplusplus
}
#endif

#endif // MQTT_MANAGER_H



