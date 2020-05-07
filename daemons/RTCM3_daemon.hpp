#ifndef __RTCM3_MODULE_HPP
#define __RTCM3_MODULE_HPP

#include "mbed.h"
#include "EthernetInterface.h"

#include "ROBOT_CONFIG.hpp"
#include "EVENT_FLAGS.hpp"
#include "MOAB_DEFINITIONS.h"


class RTCM3_daemon {

public:
	RTCM3_daemon(PinName, PinName, UDPSocket*);

	void Start();


private:
	UDPSocket *_sock;
	RawSerial *_serport;

	EventFlags _event_flags;

	Thread main_thread;

	void main_worker(void);
	void _Serial_Rx_Interrupt(void);

};

#endif // __RTCM3_MODULE_HPP
