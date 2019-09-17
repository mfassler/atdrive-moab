
#include "ShaftEncoder.hpp"


ShaftEncoder::ShaftEncoder(PinName pinName) {

	_pin = new InterruptIn(pinName, PullUp);

	_pin->rise(callback(this, &ShaftEncoder::_rise_interrupt));
	_pin->fall(callback(this, &ShaftEncoder::_fall_interrupt));

	// My shaft encoder seems to have an asymmetry, so we keep a
	// separate "rise" timer vs "fall" timer
	timer1.start();
	timer2.start();

	rise_pps = 0;
	fall_pps = 0;
	current_pps = 0;  // pulses per second
}


void ShaftEncoder::_rise_interrupt() {

	uint32_t ts = timer1.read_us();
	timer1.reset();

	rise_pps = 1000000.0 / ts;
}


void ShaftEncoder::_fall_interrupt() {
	uint32_t ts = timer2.read_us();
	timer2.reset();

	fall_pps = 1000000.0 / ts;
}


double ShaftEncoder::get_pps() {

	uint32_t ts1 = timer1.read_us();
	uint32_t ts2 = timer2.read_us();

	double max_possible_pps;
	if (ts1 < ts2) {
		max_possible_pps = 1000000.0 / ts1;
	} else {
		max_possible_pps = 1000000.0 / ts2;
	}

	current_pps = 0.5 * (rise_pps + fall_pps);

	if (current_pps > max_possible_pps) {
		return max_possible_pps;
	}

	return current_pps;
}


