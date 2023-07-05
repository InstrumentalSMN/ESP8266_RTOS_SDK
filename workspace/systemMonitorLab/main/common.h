#ifndef _COMMON_H_
#define _COMMON_H_
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#define MAX_PRECISION   (10)
/*==================[external data declaration]==============================*/
extern char *TAG;
extern char payload2[300];

/*==================[function declaration]==============================*/
char* floatToString( float value, char* result, int32_t precision );


/*==================[end of file]============================================*/
#endif
