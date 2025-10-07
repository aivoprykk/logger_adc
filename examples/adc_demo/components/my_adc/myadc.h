#ifndef DDFACF95_9B53_4E09_A525_8DBF0F8607D9
#define DDFACF95_9B53_4E09_A525_8DBF0F8607D9

#include "esp_sleep.h"
#include <stdio.h>
#include <string.h>

void init_ulp_program(void);

/* This function is called every time before going into deep sleep.
 * It starts the ULP program and resets measurement counter.
 */
void start_ulp_program(void);

void take_measurement(void);
    
#endif /* DDFACF95_9B53_4E09_A525_8DBF0F8607D9 */
