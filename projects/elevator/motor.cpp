#include <tinyfsm.hpp>
#include "motor.hpp"
#include "uart.h"


// ----------------------------------------------------------------------------
// Motor states
//

class Stopped
: public Motor
{
  void entry() override {
	brand.data = (uint8_t *)"Motor: stopped\n\r";
	brand.data_len = sizeof("Motor: stopped\n\r");
	tx_complete = 0;
	bufpos = 0;
	// enable usart2 tx interrupt
	USART2->CR1 |= (1 << 7);

	while(tx_complete == 0)
	{
		flash(LEDDELAY1);
	}
    direction = 0;
  };
};

class Up
: public Motor
{
  void entry() override {
	brand.data = (uint8_t *)"Motor: moving up\n\r";
	brand.data_len = sizeof("Motor: moving up\n\r");
	tx_complete = 0;
	bufpos = 0;
	// enable usart2 tx interrupt
	USART2->CR1 |= (1 << 7);

	while(tx_complete == 0)
	{
		flash(LEDDELAY1);
	}
    direction = 1;
  };
};

class Down
: public Motor
{
  void entry() override {
	brand.data = (uint8_t *)"Motor: moving down\n\r";
	brand.data_len = sizeof("Motor: moving down\n\r");
	tx_complete = 0;
	bufpos = 0;
	// enable usart2 tx interrupt
	USART2->CR1 |= (1 << 7);

	while(tx_complete == 0)
	{
		flash(LEDDELAY1);
	}
    direction = -1;
  };
};


// ----------------------------------------------------------------------------
// Base State: default implementations
//

void Motor::react(MotorStop const &) {
  transit<Stopped>();
}

void Motor::react(MotorUp const &) {
  transit<Up>();
}

void Motor::react(MotorDown const &) {
  transit<Down>();
}

int Motor::direction{0};


// ----------------------------------------------------------------------------
// Initial state definition
//
FSM_INITIAL_STATE(Motor, Stopped)
