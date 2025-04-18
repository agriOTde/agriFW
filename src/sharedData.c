#include "sharedData.h"

shared_sub_data_t shared_sub_data = {
    .motor_command = false,
};

SemaphoreHandle_t shared_sub_data_mutex = NULL;
SemaphoreHandle_t data_publish_mutex = NULL;
