#ifndef __SHAFT_ENCODER_HPP_
#define __SHAFT_ENCODER_HPP_

#include "mbed.h"
#include "rtos.h"


class ShaftEncoder {

public:
	ShaftEncoder(PinName);

	double get_pps();

private:
	InterruptIn *_pin;

	volatile uint32_t last_pulse4;
	volatile uint32_t last_pulse3;
	volatile uint32_t last_pulse2;
	volatile uint32_t last_pulse1;
	volatile uint32_t last_pulse0;
	double current_pps;

	void _rise_interrupt();
	//void _fall_interrupt();
};


#endif // __SHAFT_ENCODER_HPP_
