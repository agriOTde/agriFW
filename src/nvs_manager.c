#include "nvs_manager.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "nvs.h"
#include <string.h>



static const char *NVS_WRITER = "NVS_WRITER";
static const char* NVS_READER_TAG = "NVS_READER";


void store_values(char *nvs_namespace, char *handle, ValueType _type, const void* val_ptr){

    nvs_handle_t nvs_handle;
    esp_err_t nvs_err;
    nvs_err = nvs_open(nvs_namespace, NVS_READWRITE, &nvs_handle);
    if (nvs_err != ESP_OK) {
        printf("Error opening NVS!\n");
        return;
    }
    switch(_type){
        case TYPE_U16: {
            uint16_t val = *(uint16_t *)val_ptr;
            nvs_err = nvs_set_u16(nvs_handle, handle, val); 
            if (nvs_err == ESP_OK) {
                nvs_commit(nvs_handle);  // save to flash
                ESP_LOGI(NVS_WRITER, "Stored value of %s = %u in namespace = %s ", handle, val, nvs_namespace);
            } else {
                ESP_LOGE(NVS_WRITER, "Failed to store value");
            }
            break;
        }
        case TYPE_U32: {
            uint32_t val = *(uint32_t *)val_ptr;
            nvs_err = nvs_set_u32(nvs_handle, handle, val); 
            if (nvs_err == ESP_OK) {
                nvs_commit(nvs_handle);  // save to flash
                ESP_LOGI(NVS_WRITER, "Stored value of %s = %lu in namespace = %s ", handle, val, nvs_namespace);
            } else {
                ESP_LOGE(NVS_WRITER, "Failed to store value");
            }
            break;
        }
        case TYPE_I8: {
            int8_t val = *(int8_t *)val_ptr;
            nvs_err = nvs_set_i8(nvs_handle, handle, val); 
            if (nvs_err == ESP_OK) {
                nvs_commit(nvs_handle);  // save to flash
                ESP_LOGI(NVS_WRITER, "Stored value of %s = %d in namespace = %s ", handle, val, nvs_namespace);
            } else {
                ESP_LOGE(NVS_WRITER, "Failed to store value");
            }
            break;
        }
        default:
            ESP_LOGW(NVS_WRITER, "Unhandled type");
        break;
    }
}

esp_err_t read_nvs_value(const char* namespaces[NS_COUNT], 
    const char* keys[NS_COUNT][MAX_KEYS_PER_NS], 
    const int key_counts[], 
    int ns_count,
    const char *ns_to_read,
    const char *key_to_read,
    ValueType _type,
    void *out_val) {

    for (int i = 0; i < ns_count; i++) {
        const char* ns = namespaces[i];
        int num_keys = key_counts[i];

        if (num_keys > MAX_KEYS_PER_NS) {
            ESP_LOGW(NVS_READER_TAG, "Skipping namespace '%s' due to too many keys", ns);
            continue;
        }

        if (strcmp(ns_to_read, ns) != 0) {
            continue;
        }

        ESP_LOGI(NVS_READER_TAG, "Namespace matched: %s", ns);
        nvs_handle_t handle;
        esp_err_t err = nvs_open(ns, NVS_READONLY, &handle);
        if (err != ESP_OK) {
            ESP_LOGE(NVS_READER_TAG, "Failed to open namespace '%s'", ns);
            return err;
        }

        for (int j = 0; j < num_keys; j++) {
            const char* key = keys[i][j];
            if (key == NULL || strlen(key) == 0) {
                ESP_LOGE(NVS_READER_TAG, "Key is NULL!");
                continue;
            }

            if (strcmp(key_to_read, key) != 0) {
                ESP_LOGE(NVS_READER_TAG, "Key Unmatched!");
                continue;
            }

            switch (_type) {
                case TYPE_U8:
                    err = nvs_get_u8(handle, key, (uint8_t *)out_val);
                    break;
                case TYPE_I8:
                    err = nvs_get_i8(handle, key, (int8_t *)out_val);
                    break;
                case TYPE_U16:
                    err = nvs_get_u16(handle, key, (uint16_t *)out_val);
                    break;
                case TYPE_I16:
                    err = nvs_get_i16(handle, key, (int16_t *)out_val);
                    break;
                case TYPE_U32:
                    err = nvs_get_u32(handle, key, (uint32_t *)out_val);
                    break;
                case TYPE_I32:
                    err = nvs_get_i32(handle, key, (int32_t *)out_val);
                    break;
                default:
                    ESP_LOGW(NVS_READER_TAG, "Unhandled value type");
                    err = ESP_ERR_INVALID_ARG;
                    break;
            }
            ESP_LOGE(NVS_READER_TAG, "DEBUG , err = %X", err);
            
            if (err == ESP_OK) {
                ESP_LOGI(NVS_READER_TAG, " Key %s :  ", key);
            } else if (err == ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGW(NVS_READER_TAG, "Key '%s' not set", key);
            } else {
                ESP_LOGE(NVS_READER_TAG, "Error reading key '%s': %s", key, esp_err_to_name(err));
            }

            nvs_close(handle);
            return err;
        }

        nvs_close(handle);
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_ERR_NOT_FOUND;
}

void read_all_nvs_values(const char* namespaces[NS_COUNT], 
    const char* keys[NS_COUNT][MAX_KEYS_PER_NS], 
    const int key_counts[], 
    int ns_count) {

    for (int i = 0; i < ns_count; i++) {
        const char* ns = namespaces[i];
        int num_keys = key_counts[i];

        if (num_keys > MAX_KEYS_PER_NS) {
            ESP_LOGW(NVS_READER_TAG, "Skipping namespace '%s' due to too many keys", ns);
            continue;
        }

        ESP_LOGI(NVS_READER_TAG, "Namespace: %s", ns);
        nvs_handle_t handle;
        esp_err_t err = nvs_open(ns, NVS_READONLY, &handle);

        if (err != ESP_OK) {
            ESP_LOGI(NVS_READER_TAG, "  Failed to open namespace '%s'", ns);
            continue;
        }

        for (int j = 0; j < num_keys; j++) {
            const char* key = keys[i][j];
            uint32_t value = 0;
            
            if (key == NULL || strlen(key) == 0) {
                ESP_LOGW(NVS_READER_TAG, "  Skipping null or empty key [%d][%d]", i, j);
                continue;
            }

            err = nvs_get_u32(handle, key, &value);
            if (err == ESP_OK) {
                ESP_LOGI(NVS_READER_TAG, "  %s: %lu", key, value);
            } else if (err == ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGI(NVS_READER_TAG, "  %s: [not set]", key);
            } else {
                ESP_LOGI(NVS_READER_TAG, "  %s: [error %d]", key, err);
            }
        
        }

        nvs_close(handle);
    }
}


