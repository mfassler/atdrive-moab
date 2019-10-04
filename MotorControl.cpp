
#include "MotorControl.hpp"
//#include "ROBOT_CONFIG.hpp"
#define _STEERING_PW_CENTER 0.001424
#define _STEERING_PW_RANGE 0.000391

MotorControl::MotorControl(PinName a, PinName b) {

	_motor_A = new PwmOut(a);
	_motor_B = new PwmOut(b);

	_motor_A->period_ms(15);  // same period as S.Bus
	_motor_B->period_ms(15);  // same period as S.Bus

	_prev_value_a = 0;
	_prev_value_b = 0;

	_pw_a = 0.0;
	_pw_b = 0.0;
}



#define SBUS_CENTER 1024
// center + 672
#define SBUS_MAX 1696
// center - 672
#define SBUS_MIN 352


const float _STEERING_PW_MAX = _STEERING_PW_CENTER + _STEERING_PW_RANGE;
const float _STEERING_PW_MIN = _STEERING_PW_CENTER - _STEERING_PW_RANGE;

void MotorControl::set_steering(uint16_t value) {

	if (value > SBUS_MAX) {
		value = SBUS_MAX;
	} else if (value < SBUS_MIN) {
		value = SBUS_MIN;
	}

	if (_prev_value_a == value) {
		return;
	}

	_prev_value_a = value;

	// This is a value between -1.0 and +1.0:
	float percentValue = ((float) value - 1024.0) / 672.0;

	_pw_a = percentValue * _STEERING_PW_RANGE + _STEERING_PW_CENTER;

	// The limits of this particular servo on this particular bot:
	if (_pw_a < _STEERING_PW_MIN) {
		_pw_a = _STEERING_PW_MIN;
	} else if (_pw_a > _STEERING_PW_MAX) {
		_pw_a = _STEERING_PW_MAX;
	}

	_motor_A->pulsewidth(_pw_a);
}



// These numbers are specific to this particular servo on this particular
// frame:
#define _THROTTLE_PW_CENTER 0.001515
#define _THROTTLE_PW_RANGE 0.000400
const float _THROTTLE_PW_MAX = _THROTTLE_PW_CENTER + _THROTTLE_PW_RANGE;
const float _THROTTLE_PW_MIN = _THROTTLE_PW_CENTER - _THROTTLE_PW_RANGE;
void MotorControl::set_throttle(uint16_t value) {

	if (value > SBUS_MAX) {
		value = SBUS_MAX;
	} else if (value < SBUS_MIN) {
		value = SBUS_MIN;
	}

	if (_prev_value_b == value) {
		return;
	}

	_prev_value_b = value;

	// This is a value between -1.0 and +1.0:
	float percentValue = ((float) value - 1024.0) / 672.0;

	_pw_b = percentValue * _THROTTLE_PW_RANGE + _THROTTLE_PW_CENTER;

	// The limits of this particular servo on this particular bot:
	if (_pw_b < _THROTTLE_PW_MIN) {
		_pw_b = _THROTTLE_PW_MIN;
	} else if (_pw_b > _THROTTLE_PW_MAX) {
		_pw_b = _THROTTLE_PW_MAX;
	}

	_motor_B->pulsewidth(_pw_b);
}


uint16_t MotorControl::get_value_a(void) {
	return _prev_value_a;
}

uint16_t MotorControl::get_value_b(void) {
	return _prev_value_b;
}

float MotorControl::get_pw_a(void) {
	return _pw_a;
}

float MotorControl::get_pw_b(void) {
	return _pw_b;
}


