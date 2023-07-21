#ifndef _SENSORAHT10_H_
#define _SENSORAHT10_H_

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "common.h"


/*==================[macros]=================================================*/



/*==================[typedef]================================================*/

/*==================[external variable declaration]=========================*/
extern char temp_string_aht10[10];
extern char rh_string_aht10[10];
/*==================[external functions declaration]=========================*/
esp_err_t readTemperature(i2c_port_t i2c_num, float *temperature);
esp_err_t readHumidity(i2c_port_t i2c_num, float *rh);
void aht10_task(void *arg);




/*==================[end of file]============================================*/
#endif
