#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include "freertos/event_groups.h"
#include "soilMoist.h"
#include "npk.h"
#include <string.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "cJSON.h"

// #include "ESP32Ping.h"


#ifdef __cplusplus
extern "C"
{
#endif

// /***************** C headers ******************/
#include "wifi_e.h"
#include "httpClient.h"

// /*********************************************/

#ifdef __cplusplus
} // extern "C"
#endif




// /***************** TAGS ******************/
// static const char *WIFI_TAG = "wifi station";

// /***************** WIFI PARAMTERS ******************/
// #define WLAN_SSID "Mr. Wick" 
// #define WLAN_PSWD "myself.23"
// const char* server="127.0.0.1:3000/country";
    #define WIFI_CONNECTED_BIT BIT0
    #define WIFI_FAIL_BIT      BIT1
static const char *WIFI_TAG = "wifi station";
static const char *JSON_TAG = "JSON object";
static const char *NPK_TAG = "NPK Ping";



TaskHandle_t myTaskHandle = NULL;

extern "C"
{ 
    uint32_t voltage = 0;
    double moisturePerc = 0;
    void pingMoisture(void *arg);
    void pingNPK(void *arg);

// static const char *WIFI_TAG = "wifi station";

    void app_main() {

    // static const char* TAG ="MAIN";
    // ESP_LOGD(TAG,"Hello in Main");
    printf("Main\n");

    // -----------WIFI INIT-------------------
        //Initialize NVS
    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //   ESP_ERROR_CHECK(nvs_flash_erase());
    //   ret = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK(ret);

    // ESP_LOGI(WIFI_TAG, "ESP_WIFI_MODE_STA");
    // wifi_init_sta();


    // -----------WIFI INIT-------------------

    // vTaskDelay(1000/ portTICK_PERIOD_MS);

    // Tasks created
    // xTaskCreatePinnedToCore
    // (pingMoisture, "Pinging Moisture from the sensors", 9000, NULL,10, &myTaskHandle, 1);
    xTaskCreatePinnedToCore
    (pingNPK, "Pinging NPK sensor", 9000, NULL,10, &myTaskHandle, 1);

   }
   
//    void pingMoisture(void *arg)
//    {
//     uint8_t i=0;
//     while(1){


//         soilMoist mois1(MOIST_2);

//         vTaskDelay(1000/ portTICK_PERIOD_MS);
//         voltage =  mois1.getVolt();
//         moisturePerc = mois1.getPerc();

//         printf("Moisture Pinging\n");
        
//         printf("Moisture Percent = %f\nRaw Voltage = %lu",
//         moisturePerc,voltage);

//         // POST DATA 
//         cJSON *root;
//         root = cJSON_CreateObject();
//         cJSON_AddNumberToObject(root, "moistureNum", i);
//         cJSON_AddNumberToObject(root, "moistureVal", moisturePerc);

//         const char *my_json_string = cJSON_Print(root);
//         char *post_data = cJSON_Print(root);
//         ESP_LOGI(JSON_TAG, "my_json_string\n%s",post_data);
//         http_rest_with_url(post_data);

//         i++;


//         // TODO: Humidity Calibration
//         // TODO: NPK Sensor Implementation
//         // if(i>5)
//         //     break;}
//         // vTaskDelete(myTaskHandle);
//     }
//    }

      void pingNPK(void *arg)
   {
    printf("In NPK\n");
    uint8_t j=0;
    while(1){

        npk NPK1(9600);

        uint8_t **npk_val = NPK1.getNPK();
        printf("Nitrogen = %s, Phosphorus = %s, Potassium = %s\n",
        npk_val[0],npk_val[1],npk_val[2]);
        // j++;
        // if(j>5)
        //     break;

    }
   }

};
