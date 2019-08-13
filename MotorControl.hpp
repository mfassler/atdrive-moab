#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "mbed.h"

class MotorControl {

public:
	MotorControl(PinName, PinName);
	void set_steering(uint16_t);
	void set_throttle(uint16_t);

	uint16_t get_value_a(void);
	uint16_t get_value_b(void);
	float get_pw_a(void);
	float get_pw_b(void);

private:
	PwmOut *_motor_A;
	PwmOut *_motor_B;

	uint16_t _prev_value_a;
	uint16_t _prev_value_b;

	float _pw_a;
	float _pw_b;
};


#endif // __MOTOR_CONTROL_H
