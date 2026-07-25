#include "Promopt.h"

uint8_t promopt_count=0;

void promopt_init(void)
{
	gpio_init(D7, GPO, GPIO_LOW, GPO_PUSH_PULL);
}


void promopt_tick(void)
{
	if(promopt_count>0)
	{
		promopt_count--;
		gpio_set_level(D7, GPIO_HIGH);
	}
	else{
		gpio_set_level(D7, GPIO_LOW);
	}
}

