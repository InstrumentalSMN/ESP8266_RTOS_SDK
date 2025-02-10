#ifndef _CONFIGRTC_H_
#define _CONFIGRTC_H_

#include <stdbool.h>
#include "common.h"


/*==================[external variable declaration]=========================*/

extern char strftime_buf[64];
// == function prototypes =======================================

void sntp_set_rtc_task(void *arg);

#endif
