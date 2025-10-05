/**
 * @file wifi_manager.h
 * @brief Wi-Fi initialization and connection management for ESP32.
 *
 * Provides functions to initialize the ESP32 in station mode, handle
 * connection retries, configure DNS servers, and define event group bits
 * used to signal Wi-Fi connection status.
 */


#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"


#define MAX_RETRIES    5

// Event group for Wi-Fi connection
extern EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// Function declarations
void wifi_init_sta(void);

void set_dns(void);

#endif /* WIFI_MANAGER_H */
