#ifndef __PUSH_BUTTON_DAEMON__HPP
#define __PUSH_BUTTON_DAEMON__HPP

#include "mbed.h"

#include "ROBOT_CONFIG.hpp"
#include "EVENT_FLAGS.hpp"
#include "MOAB_DEFINITIONS.h"



class PushButton_daemon {
public:
	PushButton_daemon(PinName, UDPSocket*);

	void Start();


private:

	InterruptIn *_button;
	UDPSocket *_sock;

	SocketAddress _destSockAddr;

	volatile uint64_t _last_pgm_fall = 0;
	volatile uint64_t _last_pgm_rise = 0;
	uint64_t _last_pgm_fall_debounce = 0;
	uint64_t _last_pgm_rise_debounce = 0;
	uint8_t _pgm_value_debounce = 1;

	bool _pgm_notice_sent = false;

	void _interrupt();

	Thread main_thread;
	void main_worker();
};


#endif // __PUSH_BUTTON_DAEMON__HPP
