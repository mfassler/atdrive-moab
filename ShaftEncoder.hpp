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
	Timer timer1;
	Timer timer2;

	double rise_pps;
	double fall_pps;
	double current_pps;

	void _rise_interrupt();
	void _fall_interrupt();
};


#endif // __SHAFT_ENCODER_HPP_
