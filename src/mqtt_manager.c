#include "mqtt_manager.h"
#include "esp_log.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"
#include "mqtt_client.h"
#include "secrets.h"  // AWS certs & keys

static const char *TAG = "MQTT";
static esp_mqtt_client_handle_t client;  // MQTT client handle

// MQTT event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "Received message on topic %s: %.*s", event->topic, event->data_len, event->data);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "Message published successfully");
            break;
        default:
            ESP_LOGI(TAG, "MQTT Event: %d", event->event_id);
            break;
    }
}

// Initialize MQTT
void mqtt_init(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .broker.verification.certificate = (const char *)aws_root_ca_pem,
        .credentials.authentication.certificate = (const char *)device_cert_pem,
        .credentials.authentication.key = (const char *)device_key_pem,
        .network.timeout_ms = 5000
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

// Publish a message
void mqtt_publish(const char *topic, const char *message) {
    if (client) {
        esp_mqtt_client_publish(client, topic, message, 0, 1, 0);
    } else {
        ESP_LOGE(TAG, "MQTT client not initialized");
    }
}
