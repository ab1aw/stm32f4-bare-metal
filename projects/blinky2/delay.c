#include "stm32f407xx.h"
#include "system_stm32f4xx.h"


void delay(volatile uint32_t);

// A simple and not accurate delay function
// that will change the speed based on the optimization settings
void delay(volatile uint32_t s)
{
	for(s; s>0; s--){
		// Do nothing
	}
}
