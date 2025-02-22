#include "npk.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <string.h>

npk::npk(int baudRate){
   setParams(baudRate);
}

uint8_t* npk::get_N(void)
{
    tx_data(nitrogen);
    // vTaskDelay(1000/ portTICK_PERIOD_MS);
    uint8_t *N =rx_data();
    return N;

}

uint8_t* npk::get_P(void)
{
    tx_data(phos);
    // vTaskDelay(1000/ portTICK_PERIOD_MS);
    uint8_t *P =rx_data();
    return P;

}

uint8_t* npk::get_K(void)
{
    tx_data(potassium);
    // vTaskDelay(1000/ portTICK_PERIOD_MS);
    uint8_t *K = rx_data();
    return K;

}

uint8_t **npk::getNPK(void)
{
     NPK[0] = get_N();
     NPK[1] = get_P();
     NPK[2] = get_K();

    return NPK;

}


