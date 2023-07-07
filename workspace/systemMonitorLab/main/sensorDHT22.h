#ifndef _SENSORDHT22_H_
#define _SENSORDHT22_H_

#include <stdbool.h>
#include "common.h"
#define DHT_OK 0
#define DHT_CHECKSUM_ERROR -1
#define DHT_TIMEOUT_ERROR -2

/*==================[external variable declaration]=========================*/
extern char temp_string_dht22[10];
extern char rh_string_dht22[10];

// == function prototypes =======================================

void 	setDHTgpio(int gpio);
void 	errorHandler(int response);
int 	readDHT();
float 	getHumidity();
float 	getTemperature();
int 	getSignalLevel( int usTimeOut, bool state );
void 	DHT_task(void *pvParameter);

#endif
