/*
 * uart.c
 *
 * author: Furkan Cayci
 * description:
 *   UART example with tx interrupt
 *   uses USART2 PA2/PA3 pins to transmit data
 *   connect a Serial to USB adapter to see the
 *   data on PC
 *
 * setup:
 *   1. enable usart clock from RCC
 *   2. enable gpioa clock
 *   3. set PA2 and PA3 as af7
 *   4. set uart word length and parity
 *   5. enable transmit and receive (TE/RE bits)
 *   6. calculate baud rate and set BRR
 *   7. enable uart
 *   8. setup uart handler to send out a given buffer
 *   9. enable tx interrupt from NVIC
 *   10.enable tx interrupt and disable when the buffer
 *      transmission is complete
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "uart.h"

/*************************************************
* function declarations
*************************************************/
void USART2_IRQHandler(void);

list_entry_t brand;

volatile int tx_complete = 0;
volatile int bufpos = 0;

void USART2_IRQHandler(void)
{
	// check if the source is transmit interrupt
	if (USART2->SR & (1 << 7)) {
		// clear interrupt
		USART2->SR &= (uint32_t)~(1 << 7);

		if (bufpos == brand.data_len) {
			// buffer is flushed out, disable tx interrupt
			tx_complete = 1;
			USART2->CR1 &= (uint32_t)~(1 << 7);
		}
		else {
			// flush ot the next char in the buffer
			tx_complete = 0;
			USART2->DR = brand.data[bufpos++];
		}
	}
}
