#ifndef __SHAFT_ENCODER_HPP_
#define __SHAFT_ENCODER_HPP_

#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"

#include "ROBOT_CONFIG.hpp"
#include "EVENT_FLAGS.hpp"

#define SHAFT_PORT 27112

class ShaftEncoder {

public:
	ShaftEncoder(PinName, EthernetInterface *net); //, EventFlags *event_flags);

	void start();

	uint64_t last_rise;
	uint64_t last_fall;

private:
	InterruptIn *_pin;
	EventFlags _event_flags;
	UDPSocket _tx_sock;

	Thread _shaft_thread;

	void _shaft_worker();

	void _rise_interrupt();
	void _fall_interrupt();

};


#endif // __SHAFT_ENCODER_HPP_
