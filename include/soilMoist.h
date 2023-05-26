#include <stdio.h>
#include <stdlib.h>

// max min voltage for moisture
//FIXME:
#define MOISTMAX 1410
#define MOISTMIN 2840

typedef enum {

    MOIST_0 = 1,    /*!< MOIST_0 is ADC1 channel 1 which is GPIO37 */
    MOIST_1,        /*!< MOIST_1 is ADC1 channel 2 which is GPIO38 */  
    MOIST_2,        /*!< MOIST_2 is ADC1 channel 3 which is GPIO39 */ 
} soilMoistNum;

class soilMoist
{
    public:
    soilMoist(uint8_t sensNum);
    uint32_t getVolt();
    double getPerc();
    double getprevMoistPerc();
    void postData();

    private:
    double prevMoistPerc = 0;
    uint8_t sensorChannel=MOIST_2; //default channel is 1
};