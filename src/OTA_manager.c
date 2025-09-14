#include <stdio.h>
#include <stdlib.h>
#include "OTA_manager.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "string.h"
#ifdef __cplusplus
extern "C" {
#endif

#define OTA_BUF_SIZE 1024

static const char *OTA_TAG = "OTA Update";
char ota_write_data[OTA_BUF_SIZE + 1] = { 0 };  // Buffer to store OTA data

uint8_t ota_update(void) {
    esp_err_t err;

    ESP_LOGI(OTA_TAG, "Starting OTA update...");

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(OTA_TAG, "Failed to find update partition");
        return false;
    }

    ESP_LOGI(OTA_TAG, "Writing to partition subtype %d at offset 0x%lx",
             update_partition->subtype, update_partition->address);

    esp_http_client_config_t config = {
        .url = OTA_URL,
        .timeout_ms = 10000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(OTA_TAG, "Failed to initialize HTTP client");
        return false;
    }

    if ((err = esp_http_client_open(client, 0)) != ESP_OK) {
        ESP_LOGE(OTA_TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return false;
    }
    int content_length = esp_http_client_fetch_headers(client);
    if (content_length < 0) {
        ESP_LOGE(OTA_TAG, "Failed to fetch headers");
        esp_http_client_close(client);
        return false;
    }
    ESP_LOGI(OTA_TAG, "Fetched headers, Content-Length = %d", content_length);

    esp_ota_handle_t update_handle = 0;
    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(OTA_TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }

    int binary_file_len = 0;
    int data_read;



    while ((data_read = esp_http_client_read(client, ota_write_data, OTA_BUF_SIZE)) > 0) {

        err = esp_ota_write(update_handle, (const void *)ota_write_data, data_read);
        if (err != ESP_OK) {
            ESP_LOGE(OTA_TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            esp_ota_end(update_handle);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return false;
        }
        binary_file_len += data_read;
        ESP_LOGI(OTA_TAG, "Written %d bytes...", binary_file_len);
    }

    // test
    // if((data_read = esp_http_client_read(client, ota_write_data, OTA_BUF_SIZE)) > 0){

    //     for (int i = 0; i < data_read && i < 16; ++i) {
    //         printf("%02X ", (unsigned char)ota_write_data[i]);
    //     }
    //     printf("\n");
    // } else {
    //     ESP_LOGI(OTA_TAG, "Empty Buffer!");
    // }

    // test

    if (data_read < 0) {
        ESP_LOGE(OTA_TAG, "Error reading HTTP stream");
        esp_ota_end(update_handle);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }
    if ((err = esp_ota_end(update_handle)) != ESP_OK) {
        ESP_LOGE(OTA_TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }

    if ((err = esp_ota_set_boot_partition(update_partition)) != ESP_OK) {
        ESP_LOGE(OTA_TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    ESP_LOGI(OTA_TAG, "Download complete, total bytes written: %d", binary_file_len);
    return true;  // Unreachable, but for completeness
}

esp_err_t get_ota_flag(bool *ota_status) {
    // Open NVS handle
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(OTA_TAG, "Failed to open NVS handle!");
        return err;
    }

    // Read the OTA flag from NVS
    uint8_t ota_flag = 0;  // Default to 0 (failure)
    err = nvs_get_u8(my_handle, "ota_flag", &ota_flag);
    if (err != ESP_OK) {
        ESP_LOGE(OTA_TAG, "Failed to read OTA flag from NVS!");
    }

    // Set the output variable to the OTA flag value
    *ota_status = (ota_flag == 1);  // 1 is success, 0 is failure

    // Close the NVS handle
    nvs_close(my_handle);

    return err;
}

esp_err_t set_ota_flag(bool ota_status) {
    // Open NVS handle
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(OTA_TAG, "Failed to open NVS handle!");
        return err;
    }

    // Write the OTA flag to NVS (e.g., 1 for success, 0 for failure)
    err = nvs_set_u8(my_handle, "ota_flag", ota_status ? 1 : 0);
    if (err != ESP_OK) {
        ESP_LOGE(OTA_TAG, "Failed to write OTA flag to NVS!");
    }

    // Commit changes and close the NVS handle
    err = nvs_commit(my_handle);
    nvs_close(my_handle);

    return err;
}


#ifdef __cplusplus
}
#endif