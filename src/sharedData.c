#include "sharedData.h"
#include "esp_log.h"
#include "string.h"

shared_sub_data_t shared_sub_data = {
    .motor_command = 0,
    .motor_duration = DEFAULT_MOTOR_DURATION,
    .motor_timePeriod = DEFAULT_MOTOR_TIMEPERIOD

};

SemaphoreHandle_t shared_sub_data_mutex = NULL;
SemaphoreHandle_t data_publish_mutex = NULL;
SemaphoreHandle_t ota_shared_mutex = NULL;




