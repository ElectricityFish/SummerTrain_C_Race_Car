#ifndef __PROMOPT_H
#define __PROMOPT_H
#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

extern uint8_t promopt_count;

void promopt_init(void);
void promopt_tick(void);

#endif
