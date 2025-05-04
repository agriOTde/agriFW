#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "esp_err.h"


#ifdef __cplusplus
extern "C" {
#endif

#define OTA_URL "http://10.144.0.142:5000/firmware.bin"

uint8_t ota_update(void);
esp_err_t get_ota_flag(bool *ota_status);
esp_err_t set_ota_flag(bool ota_status);


#ifdef __cplusplus
}
#endif

#endif //OTA_MANAGER_H
