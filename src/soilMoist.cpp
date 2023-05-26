#include "soilMoist.h"
#include "singleReadAdc.h"
#include "esp_log.h"
static const char* TAG = "soilMoist";

#ifdef __cplusplus
extern "C"
{
#endif

// /***************** C headers ******************/
#include "httpClient.h"
#include "cJSON.h"
// /*********************************************/

#ifdef __cplusplus
} // extern "C"
#endif

// Var definitions
static const char *JSON_TAG = "wifi station";

uint32_t soilMoist::getVolt()
{
    return getVoltage(sensorChannel);
}

double soilMoist::getPerc()
{
    uint32_t voltage = getVoltage(sensorChannel);
    if(voltage < 1000 || voltage > 3500)
    ESP_LOGE(TAG,"Voltage out of Bounds error voltage= %lu", voltage );
    double MoistPerc = ((double(voltage)-MOISTMIN)/(MOISTMAX - MOISTMIN))*100.0;
    
    if(MoistPerc<0)
        MoistPerc = 0;
    else if(MoistPerc>100)
        MoistPerc=100;

    prevMoistPerc = MoistPerc;
    return MoistPerc;
}

double soilMoist::getprevMoistPerc()
{
    return prevMoistPerc;
}

   
soilMoist::soilMoist(uint8_t sensNum)
{
    sensorChannel = sensNum;
}
void soilMoist::postData()
{
        // POST DATA 
        cJSON *root;
        root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "moistureval", prevMoistPerc);

        char *post_data = cJSON_Print(root);
        ESP_LOGI(JSON_TAG, "my_json_string\n%s",post_data);
        http_rest_with_url(post_data);
}
