#include "mqtt_manager.h"
#include "esp_log.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"
#include "mqtt_client.h"
#include "sharedData.h"
#include "secrets.h"  // AWS certs & keys

#define LOG_LOCAL_LEVEL ESP_LOG_ERROR

static const char *TAG = "MQTT";
static esp_mqtt_client_handle_t client;  // MQTT client handle
static bool mqtt_connected = false;
static const char **subscribed_topics = NULL;
static int num_subscribed_topics = 0;

// MQTT event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;

    
    
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            mqtt_connected = true;

            for (int i = 0; i < num_subscribed_topics; ++i) {
                int msg_id = esp_mqtt_client_subscribe(client, subscribed_topics[i], 0);
                ESP_LOGI(TAG, "Subscribed to topic: %s, msg_id: %d", subscribed_topics[i], msg_id);
            }

            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            mqtt_connected = false;
            break;
        case MQTT_EVENT_DATA:{
            ESP_LOGI(TAG, "Received message on topic %s: %.*s", event->topic, event->data_len, event->data);
                // Copy topic and data into null-terminated strings
                
            char topic[event->topic_len + 1];
            char data[event->data_len + 1];
            memcpy(topic, event->topic, event->topic_len);
            topic[event->topic_len] = '\0';
            memcpy(data, event->data, event->data_len);
            data[event->data_len] = '\0';

            // Update shared data based on topic
            if (strcmp(topic, MQTT_TOPIC_SUB) == 0) {
                bool value = (strcmp(data, "1") == 0 || strcasecmp(data, "true") == 0);

                // Block Subsrcibed data for recording
                if (xSemaphoreTake(shared_sub_data_mutex, pdMS_TO_TICKS(100))) {
                    shared_sub_data.motor_command = value;
                    xSemaphoreGive(shared_sub_data_mutex);
                }
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
    subscribed_topics = topics;
    num_subscribed_topics = num_topics;

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = RPI_MQTT_BROKER_URI,
        .network.timeout_ms = 5000
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