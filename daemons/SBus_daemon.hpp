#ifndef __SBUS_MODULE_HPP
#define __SBUS_MODULE_HPP

#include "mbed.h"
#include "EthernetInterface.h"

#include "ROBOT_CONFIG.hpp"
#include "EVENT_FLAGS.hpp"
#include "MOAB_DEFINITIONS.h"

#include "SbusParser.hpp"



class SBus_daemon {

public:
	SBus_daemon(PinName, UDPSocket*);

	void Start();

	void attachCallback(Callback<void(bool)>);

	struct sbus_udp_payload sbup;


private:
	UDPSocket *_sock;
	RawSerial *_serport;
	EventFlags _event_flags;
	Thread main_thread;
	SbusParser *_parser;
	Callback<void(bool)> _callback;
	void main_worker(void);
	void _serial_rx_interrupt(void);


};


#endif // __SBUS_MODULE_HPP
