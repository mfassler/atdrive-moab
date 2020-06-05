#ifndef __RX_COMMAND_PARSER_HPP
#define __RX_COMMAND_PARSER_HPP

#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"

#include "ROBOT_CONFIG.hpp"


class RxCommandParser {
public:
	RxCommandParser(EthernetInterface*);
	void Start();

	uint16_t auto_ch1 = 1024;
	uint16_t auto_ch2 = 1024;


private:
	UDPSocket *_rx_sock; // one, single thread for RX

	Thread main_thread;
	void main_worker();

// External servos/switches:

#ifdef USER_DIGITAL_OUT_0
	DigitalOut *user_dout_0;
#endif // USER_DIGITAL_OUT_0

// USER_PWM_OUT assumes a pulse-width in percentages
#ifdef USER_PWM_OUT_0
	PwmOut *user_pout_0;
#endif // USER_PWM_OUT_0

#ifdef USER_PWM_OUT_1
	PwmOut *user_pout_1;
#endif // USER_PWM_OUT_1

#ifdef USER_PWM_OUT_2
	PwmOut *user_pout_2;
#endif // USER_PWM_OUT_2

#ifdef USER_PWM_OUT_3
	PwmOut *user_pout_3;
#endif // USER_PWM_OUT_3

// USER_SERVO_OUT assumes a Futaba-style pulse-width in micro-seconds
#ifdef USER_SERVO_OUT_0
	PwmOut *user_servo_0;
#endif // USER_SERVO_OUT_0

};


#endif // __RX_COMMAND_PARSER_HPP
