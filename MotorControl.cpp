
#include "MotorControl.hpp"
#include "ROBOT_CONFIG.hpp"


MotorControl::MotorControl(PinName a, PinName b) {

	_motor_A = new PwmOut(a);
	_motor_B = new PwmOut(b);

	_motor_A->period_ms(15);  // same period as S.Bus
	_motor_B->period_ms(15);  // same period as S.Bus

	_prev_steer_value = 0;
	_prev_throt_value = 0;

	_pw_a = 0.0;
	_pw_b = 0.0;
}



#define SBUS_CENTER 1024
// center + 672
#define SBUS_MAX 1696
// center - 672
#define SBUS_MIN 352


// These numbers are specific to this particular servo on this particular
// frame:

#ifndef _THROTTLE_PW_CENTER
#define _THROTTLE_PW_CENTER 0.001515
#endif // _THROTTLE_PW_CENTER

#ifndef _THROTTLE_PW_RANGE
#define _THROTTLE_PW_RANGE 0.000400
#endif // _THROTTLE_PW_RANGE

#ifndef _THROTTLE_PW_MAX
const float THROTTLE_PW_MAX = _THROTTLE_PW_CENTER + _THROTTLE_PW_RANGE;
#else
const float THROTTLE_PW_MAX = _THROTTLE_PW_MAX;
#endif // _THROTTLE_PW_MAX

#ifndef _THROTTLE_PW_MIN
const float THROTTLE_PW_MIN = _THROTTLE_PW_CENTER - _THROTTLE_PW_RANGE;
#else
const float THROTTLE_PW_MIN = _THROTTLE_PW_MIN;
#endif // _THROTTLE_PW_MIN



const float _STEERING_PW_MAX = _STEERING_PW_CENTER + _STEERING_PW_RANGE;
const float _STEERING_PW_MIN = _STEERING_PW_CENTER - _STEERING_PW_RANGE;

void MotorControl::set_steering_and_throttle(uint16_t sb_steering, uint16_t sb_throttle) {

	if (sb_steering > SBUS_MAX) {
		sb_steering = SBUS_MAX;
	} else if (sb_steering < SBUS_MIN) {
		sb_steering = SBUS_MIN;
	}

	if (sb_throttle > SBUS_MAX) {
		sb_throttle = SBUS_MAX;
	} else if (sb_throttle < SBUS_MIN) {
		sb_throttle = SBUS_MIN;
	}

	if ((_prev_steer_value == sb_steering) && (_prev_throt_value == sb_throttle)) {
		return;
	}

	_prev_steer_value = sb_steering;
	_prev_throt_value = sb_throttle;

	// This is a value between -1.0 and +1.0:
	float steer_percentValue = ((float) sb_steering - 1024.0) / 672.0;  // -1 left, +1 right
	float throt_percentValue = ((float) sb_throttle - 1024.0) / 672.0;  // -1 backwards, +1 forward

#ifdef USE_SKIDMODE
	float leftWheel = throt_percentValue + steer_percentValue;
	float rightWheel = throt_percentValue - steer_percentValue;

	if (leftWheel > 1.0) {
		leftWheel = 1.0;
	} else if (leftWheel < -1.0) {
		leftWheel = -1.0;
	}

	if (rightWheel > 1.0) {
		rightWheel = 1.0;
	} else if (rightWheel < -1.0) {
		rightWheel = -1.0;
	}

	_pw_a = leftWheel * _LEFT_PW_RANGE + _LEFT_PW_CENTER;
	_pw_b = rightWheel * _RIGHT_PW_RANGE + _RIGHT_PW_CENTER;

#else // USE_SKIDMODE

	_pw_a = steer_percentValue * _STEERING_PW_RANGE + _STEERING_PW_CENTER;
	_pw_b = throt_percentValue * _THROTTLE_PW_RANGE + _THROTTLE_PW_CENTER;

#endif // USE_SKIDMODE


	// The limits of this particular servo on this particular bot:
	if (_pw_a < _STEERING_PW_MIN) {
		_pw_a = _STEERING_PW_MIN;
	} else if (_pw_a > _STEERING_PW_MAX) {
		_pw_a = _STEERING_PW_MAX;
	}


	// The limits of this particular servo on this particular bot:
	if (_pw_b < THROTTLE_PW_MIN) {
		_pw_b = THROTTLE_PW_MIN;
	} else if (_pw_b > THROTTLE_PW_MAX) {
		_pw_b = THROTTLE_PW_MAX;
	}

	_motor_B->pulsewidth(_pw_b);
	_motor_A->pulsewidth(_pw_a);
}



uint16_t MotorControl::get_steer_value(void) {
	return _prev_steer_value;
}

uint16_t MotorControl::get_throt_value(void) {
	return _prev_throt_value;
}

float MotorControl::get_pw_a(void) {
	return _pw_a;
}

float MotorControl::get_pw_b(void) {
	return _pw_b;
}


