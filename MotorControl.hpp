#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "mbed.h"

class MotorControl {

public:
	MotorControl(PinName, PinName);
	void set_steering_and_throttle(uint16_t, uint16_t);

	uint16_t get_steer_value(void);
	uint16_t get_throt_value(void);
	float get_pw_a(void);
	float get_pw_b(void);

private:
	PwmOut *_motor_A;
	PwmOut *_motor_B;

	uint16_t _prev_steer_value;
	uint16_t _prev_throt_value;

	float _pw_a;
	float _pw_b;
};


#endif // __MOTOR_CONTROL_H
