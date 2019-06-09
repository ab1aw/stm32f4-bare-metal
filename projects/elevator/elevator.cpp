#include <tinyfsm.hpp>

#include "elevator.hpp"
#include "fsmlist.hpp"
#include "uart.h"

class Idle; // forward declaration


// ----------------------------------------------------------------------------
// Transition functions
//

static void CallMaintenance() {
	brand.data = (uint8_t *)"*** calling maintenance ***\n\r";
	brand.data_len = sizeof("*** calling maintenance ***\n\r");
	tx_complete = 0;
	bufpos = 0;
	// enable usart2 tx interrupt
	USART2->CR1 |= (1 << 7);

	while(tx_complete == 0)
	{
		flash(LEDDELAY1);
	}
}

static void CallFirefighters() {
	brand.data = (uint8_t *)"*** calling firefighters ***\n\r";
	brand.data_len = sizeof("*** calling firefighters ***\n\r");
	tx_complete = 0;
	bufpos = 0;
	// enable usart2 tx interrupt
	USART2->CR1 |= (1 << 7);

	while(tx_complete == 0)
	{
		flash(LEDDELAY1);
	}
}


// ----------------------------------------------------------------------------
// State: Panic
//

class Panic
: public Elevator
{
  void entry() override {
    send_event(MotorStop());
  }
};


// ----------------------------------------------------------------------------
// State: Moving
//

class Moving
: public Elevator
{
  void react(FloorSensor const & e) override {
    int floor_expected = current_floor + Motor::getDirection();
    if(floor_expected != e.floor)
    {
		brand.data = (uint8_t *)"Floor sensor defect\n\r";
		brand.data_len = sizeof("Floor sensor defect\n\r");
		tx_complete = 0;
		bufpos = 0;
		// enable usart2 tx interrupt
		USART2->CR1 |= (1 << 7);

		while(tx_complete == 0)
		{
			flash(LEDDELAY1);
		}
      transit<Panic>(CallMaintenance);
    }
    else
    {
		brand.data = (uint8_t *)"Reached floor\n\r";
		brand.data_len = sizeof("Reached floor\n\r");
		tx_complete = 0;
		bufpos = 0;
		// enable usart2 tx interrupt
		USART2->CR1 |= (1 << 7);

		while(tx_complete == 0)
		{
			flash(LEDDELAY1);
		}
      current_floor = e.floor;
      if(e.floor == dest_floor)
        transit<Idle>();
    }
  };
};


// ----------------------------------------------------------------------------
// State: Idle
//

class Idle
: public Elevator
{
  void entry() override {
    send_event(MotorStop());
  }

  void react(Call const & e) override {
    dest_floor = e.floor;

    if(dest_floor == current_floor)
      return;

    /* lambda function used for transition action */
    auto action = [] { 
      if(dest_floor > current_floor)
        send_event(MotorUp());
      else if(dest_floor < current_floor)
        send_event(MotorDown());
    };

    transit<Moving>(action);
  };
};


// ----------------------------------------------------------------------------
// Base state: default implementations
//

void Elevator::react(Call const &) {
	brand.data = (uint8_t *)"Call event ignored\n\r";
	brand.data_len = sizeof("Call event ignored\n\r");
	tx_complete = 0;
	bufpos = 0;
	// enable usart2 tx interrupt
	USART2->CR1 |= (1 << 7);

	while(tx_complete == 0)
	{
		flash(LEDDELAY1);
	}
}

void Elevator::react(FloorSensor const &) {
	brand.data = (uint8_t *)"FloorSensor event ignored\n\r";
	brand.data_len = sizeof("FloorSensor event ignored\n\r");
	tx_complete = 0;
	bufpos = 0;
	// enable usart2 tx interrupt
	USART2->CR1 |= (1 << 7);

	while(tx_complete == 0)
	{
		flash(LEDDELAY1);
	}
}

void Elevator::react(Alarm const &) {
  transit<Panic>(CallFirefighters);
}

int Elevator::current_floor = Elevator::initial_floor;
int Elevator::dest_floor    = Elevator::initial_floor;


// ----------------------------------------------------------------------------
// Initial state definition
//
FSM_INITIAL_STATE(Elevator, Idle)
