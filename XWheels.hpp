#ifndef __X_WHEELS_HPP
#define __X_WHEELS_HPP

#include "mbed.h"


class XWheels {

public:
	XWheels(PinName, PinName);

	void Start();
	void setRPMs(float, float);
	void set_steering_and_throttle(uint16_t, uint16_t);


private:
	PinName _tx_pin;
	PinName _rx_pin;

	RawSerial *_uart;

	float _rpm_motor_1;
	float _rpm_motor_2;

	Thread main_thread;

	void main_worker();
	void waitUntilFourZero();
	void ESCHandShake();
	void send_motor_command();
};


#endif // __X_WHEELS_HPP
