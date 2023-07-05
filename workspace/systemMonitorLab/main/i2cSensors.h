#ifndef _I2SENSORS_H_
#define _I2SENSORS_H_

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








/*==================[external functions declaration]=========================*/
esp_err_t readTemperature(i2c_port_t i2c_num, float *temperature);
esp_err_t readHumidity(i2c_port_t i2c_num, float *rh);
void i2c_task_example(void *arg);
char* floatToString( float value, char* result, int32_t precision );



/*==================[end of file]============================================*/
#endif
