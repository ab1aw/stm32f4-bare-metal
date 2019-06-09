#ifndef __ELEVATOR_UART_H__
#define __ELEVATOR_UART_H__ 1

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f407xx.h"
#include "system_stm32f4xx.h"

// create a led delay. Just a rough estimate
// for one second delay
#define LEDDELAY1	10000000
#define LEDDELAY2	5000000


typedef struct list_entry_s {
	uint8_t *data;
	int data_len;
} list_entry_t;

extern list_entry_t brand;

extern volatile int tx_complete;
extern volatile int bufpos;

void USART2_IRQHandler(void);

void flash(volatile uint32_t d);
void delay(volatile uint32_t s);

#ifdef __cplusplus
}
#endif

#endif
