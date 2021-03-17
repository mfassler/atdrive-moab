#ifndef __X_WHEELS_HPP
#define __X_WHEELS_HPP

#include "mbed.h"
#include "EVENT_FLAGS.hpp"



class XWheels {

public:
	XWheels(PinName, PinName);

	void Start();
	void setRPMs(float, float);
	void set_steering_and_throttle(uint16_t, uint16_t);


private:
	PinName _tx_pin;
	PinName _rx_pin;

	BufferedSerial *_serport;

	float _rpm_motor_1;
	float _rpm_motor_2;
	int _init_count;

	EventFlags _event_flags;

	Thread main_thread;
	Thread timeout_thread;

	void main_worker();
	void timeout_worker();

	void _do_something(void);
	void _send_init();
	void _send_motor_command();

};


#endif // __X_WHEELS_HPP
