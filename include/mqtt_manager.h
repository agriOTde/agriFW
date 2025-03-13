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


// Initialize MQTT
void mqtt_init(void);

// Publish a message
void mqtt_publish(const char *topic, const char *message);

#ifdef __cplusplus
}
#endif

#endif // MQTT_MANAGER_H



