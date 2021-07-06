#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "mbed.h"

class MotorControl {

public:
	MotorControl(PinName, PinName);
	void set_steering_and_throttle(uint16_t, uint16_t, bool Force=false);

	uint16_t get_steer_value(void);
	uint16_t get_throt_value(void);
	float get_pw_a(void);
	float get_pw_b(void);

	void reset_steering_trim(void);
	void adjust_steering_trim(float);

private:
	PwmOut *_motor_A;
	PwmOut *_motor_B;

	uint16_t _prev_steer_value;
	uint16_t _prev_throt_value;

	float _pw_a;
	float _pw_b;

	//float _default_steering_pw_center;
	float _current_steering_pw_center;
};


#endif // __MOTOR_CONTROL_H
