#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_system.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_err.h"

#define MQTT_BROKER_URI "mqtts://aeti3itrm7uta-ats.iot.eu-north-1.amazonaws.com:8883"
#define RPI_MQTT_BROKER_URI "mqtt://192.168.1.100:1883"


// Initialize MQTT
void mqtt_init(void);

// Publish a message
uint16_t mqtt_publish(const char *topic, const char *message);

// Reconnet MQTT
void mqtt_reconnect(void);

#ifdef __cplusplus
}
#endif

#endif // MQTT_MANAGER_H



