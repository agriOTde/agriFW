#include "sharedData.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "string.h"

shared_sub_data_t shared_sub_data = {
    .motor_command = 0,
    .motor_duration = DEFAULT_MOTOR_DURATION,
    .motor_timePeriod = DEFAULT_MOTOR_TIMEPERIOD

};

SemaphoreHandle_t shared_sub_data_mutex = NULL;
SemaphoreHandle_t data_publish_mutex = NULL;
SemaphoreHandle_t ota_shared_mutex = NULL;

static const char* SHARED_DATA_TAG = "SHARED_DATA";

const char* namespaces[NS_COUNT] = {
    "schedule",
    "constraints"
    };
    
const char* all_keys[NS_COUNT][MAX_KEYS_PER_NS] = {{"timeperiod", "duration", "command"},{"humidity",NULL,NULL}};

const int key_counts[NS_COUNT] = {3};

char * get_esp_client_id(void){
    ESP_LOGI(SHARED_DATA_TAG, "MQTT Client ID");
    uint8_t mac[6];
    static char client_id[32];
    esp_err_t ret = esp_efuse_mac_get_default(mac);
    if (ret != ESP_OK) {
        ESP_LOGE(SHARED_DATA_TAG, "Failed to read MAC address from eFuse!");
    }

    snprintf(client_id, sizeof(client_id),
             "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);


    return client_id;
}


