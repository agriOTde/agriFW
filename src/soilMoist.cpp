#include "soilMoist.h"
#include "singleReadAdc.h"


uint32_t soilMoist::getVolt()
{
    return getVoltage(sensorChannel);
}

double soilMoist::soilMoisture()
{
    uint32_t voltage = getVoltage(sensorChannel);

    double MoistPerc = (voltage-MOISTMIN)/(MOISTMAX - MOISTMIN);
    MoistPerc = MoistPerc * 100;
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
