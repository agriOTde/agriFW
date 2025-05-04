#include "sharedData.h"
#include "esp_log.h"
#include "string.h"

shared_sub_data_t shared_sub_data = {
    .motor_command = false,
    .motor_duration = 5000,
    .motor_timePeriod = 0

};

SemaphoreHandle_t shared_sub_data_mutex = NULL;
SemaphoreHandle_t data_publish_mutex = NULL;
SemaphoreHandle_t ota_shared_mutex = NULL;




