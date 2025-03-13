#include <stdio.h>
#include <stdlib.h>
#include "esp_system.h"

#ifdef __cplusplus
extern "C"
{
#endif

// *****************************************************
//                   VAR DECLARATIONS
// *****************************************************


// *****************************************************
//                  FUNCTION DECLARATIONS
// *****************************************************

void setParams(int baud_rate);
uint8_t* rx_data(void);
void tx_data(const char *npkFrames);



#ifdef __cplusplus
} // extern "C"
#endif
