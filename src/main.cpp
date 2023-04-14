#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "lwip/sys.h"
#include "soilMoist.h"

// /***************** TAGS ******************/
// static const char *WIFI_TAG = "wifi station";

// /***************** WIFI PARAMTERS ******************/
// #define WLAN_SSID "Mr. Wick" 
// #define WLAN_PSWD "myself.23"
// const char* server="127.0.0.1:3000/country";

TaskHandle_t myTaskHandle = NULL;
extern "C"
{ 
    uint32_t voltage = 0;
    double moisturePerc = 0;
    void pingMoisture(void *arg);

    void app_main() {

    // static const char* TAG ="MAIN";
    // ESP_LOGD(TAG,"Hello in Main");
    printf("Main\n");
    vTaskDelay(1000/ portTICK_RATE_MS);

    // Tasks created
    xTaskCreatePinnedToCore
    (pingMoisture, "Pinging Moisture from the sensors", 4096, NULL,10, &myTaskHandle, 1);

   }
   
   void pingMoisture(void *arg)
   {
    // int i=0;
    while(1){
        
        soilMoist mois1(MOIST_2);

        vTaskDelay(500/ portTICK_RATE_MS);

        voltage =  mois1.getVolt();
        moisturePerc = mois1.getPerc();

        printf("Moisture Pinging\n");
        printf("Moisture Percent = %f\nRaw Voltage = %d", moisturePerc,voltage);
        // TODO: Humidity Calibration
        // TODO: NPK Sensor Implementation
        // i++;
        // if(i>5)
        //     break;
        // vTaskDelete(myTaskHandle);
    }
   }

}
