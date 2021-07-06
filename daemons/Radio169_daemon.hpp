#ifndef __RADIO169_MODULE_HPP
#define __RADIO169_MODULE_HPP

#include "mbed.h"
#include "EthernetInterface.h"

#include "ROBOT_CONFIG.hpp"
#include "EVENT_FLAGS.hpp"
#include "MOAB_DEFINITIONS.h"

#include "MotorControl.hpp"

struct controller_values_t {
	// Special buttons:
	bool start;
	bool back;
	bool logitech;

	// The four buttons on the front side:
	bool LB;
	bool LT;
	bool RB;
	bool RT;

	// The four individual buttons on the right:
	bool Y;
	bool A;
	bool B;
	bool X;

	// The 4-way D-pad on the left:
	bool dpad_up;
	bool dpad_down;
	bool dpad_right;
	bool dpad_left;

	// The analog joystick on the left:
	signed char leftjoy_lr;
	signed char leftjoy_ud;

	// The analog joystick on the right:
	signed char rightjoy_lr;
	signed char rightjoy_ud;
};




enum button_state_t {
	no_press,
	press
};



class Radio169_daemon {

public:
	Radio169_daemon(PinName, PinName, UDPSocket*, MotorControl*);

	void Start();

	void attachCallback(Callback<void()>);
	struct controller_values_t controller_values;
	bool timeout;

	uint16_t sb_steering;
	uint16_t sb_throttle;

	enum Moab_State_t requested_moab_state;

	void _stateful_stuff(void);

private:
	UDPSocket *_sock;
	SocketAddress _destSockAddr;
	BufferedSerial *_serport;

	EventFlags _event_flags;

	MotorControl *_motorControl;

	Callback<void()> _callback;

	Thread main_thread;
	Thread timeout_thread;

	void main_worker(void);
	void timeout_worker(void);

	char output_buffer[8];
	void _parse_vals(void);

	// Stop release must be held down for 0.1 seconds
	enum button_state_t _stop_release_state = no_press;
	Kernel::Clock::time_point _stop_release_time = Kernel::Clock::now();

	// If max throttle for more than 2 seconds, then give extra boost
	enum button_state_t _max_throttle_state = no_press;
	Kernel::Clock::time_point _max_throttle_time = Kernel::Clock::now();

};

#endif // __RADIO169_MODULE_HPP
