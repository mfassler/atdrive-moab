#ifndef __RADIO169_MODULE_HPP
#define __RADIO169_MODULE_HPP

#include "mbed.h"
#include "EthernetInterface.h"

#include "ROBOT_CONFIG.hpp"
#include "EVENT_FLAGS.hpp"
#include "MOAB_DEFINITIONS.h"



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




class Radio169_daemon {

public:
	Radio169_daemon(PinName, PinName, UDPSocket*);

	void Start();

	void attachCallback(Callback<void(bool)>);
	struct controller_values_t controller_values;


private:
	UDPSocket *_sock;
	RawSerial *_serport;

	EventFlags _event_flags;

	Thread main_thread;

	Callback<void(bool)> _callback;

	void main_worker(void);
	void _Serial_Rx_Interrupt(void);

	char output_buffer[8];
	void _parse_vals(void);

	// Ring buffer for serial-to-UDP operations:
	#define _RING_BUFFER_SIZE169 256

	char _ringBuf[_RING_BUFFER_SIZE169];
	int _inputIDX = 0;
	int _outputIDX = 0;


};

#endif // __RADIO169_MODULE_HPP