#include "esp_system.h"
#include "esp_event.h"
#include <stdint.h>
#include "esp_log.h"
#include "serial_uart.h"

const char nitrogen[] = {0x01,0x03,0x00,0x1E,0x00,0x01,0xE4,0x0C}; 
const char phos[] = {0x01,0x03,0x00,0x1F,0x00,0x01,0xB5,0xCC};
const char potassium[] = {0x01,0x03,0x00,0x20,0x00,0x01,0x85,0xC0};


class npk{

    public:

    npk(int baudRate);

    uint8_t* get_N(void);
    uint8_t* get_P(void);
    uint8_t* get_K(void);
    uint8_t **getNPK(void);


    uint8_t *NPK[3] = {0};
};