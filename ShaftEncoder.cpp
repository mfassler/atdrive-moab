
#include "ShaftEncoder.hpp"


ShaftEncoder::ShaftEncoder(PinName pinName) {

	_pin = new InterruptIn(pinName, PullUp);

	_pin->rise(callback(this, &ShaftEncoder::_rise_interrupt));
	_pin->fall(callback(this, &ShaftEncoder::_rise_interrupt));

	last_pulse0 = 0;  // rise or fall
	last_pulse1 = 0;  // rise or fall
	last_pulse2 = 0;  // rise or fall
	last_pulse3 = 0;  // rise or fall
	last_pulse4 = 0;  // rise or fall

	current_pps = 0;  // pulses per second
}



void ShaftEncoder::_rise_interrupt() {
	last_pulse4 = last_pulse3;
	last_pulse3 = last_pulse2;
	last_pulse2 = last_pulse1;
	last_pulse1 = last_pulse0;
	last_pulse0 = us_ticker_read();

	uint64_t ts = last_pulse0;
	if (ts < last_pulse4) {
		ts |= 0x100000000;
	}

	current_pps = 4000000.0 / (ts - last_pulse4);
}

/*
void ShaftEncoder::_fall_interrupt() {
	last_last_pulse = last_pulse;
	last_pulse = us_ticker_read();

	_event_flags.set(_EVENT_FLAG_SHAFT_FALL);
}
*/

double ShaftEncoder::get_pps() {
	uint64_t ts = us_ticker_read();

	if (ts < last_pulse4) {
		ts |= 0x100000000;
	}

	double max_possible_pps = 4000000.0 / (ts - last_pulse4);

	if (current_pps > max_possible_pps) {
		return max_possible_pps;
	}
	return current_pps;
}


