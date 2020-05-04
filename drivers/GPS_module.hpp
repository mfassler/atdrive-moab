#ifndef __GPS_MODULE_HPP
#define __GPS_MODULE_HPP

#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"

#include "ROBOT_CONFIG.hpp"
#include "EVENT_FLAGS.hpp"
#include "MOAB_DEFINITIONS.h"



class GPS_module {
public:
	GPS_module(PinName, PinName, EthernetInterface*);

	void Start();


private:
	UDPSocket *_sock;
	RawSerial *_gps_in;
	EventFlags _event_flags;  // I think these are shared globally...

	Thread main_thread;

	void main_worker();
	void _Gps_Rx_Interrupt();

};

#endif // __GPS_MODULE_HPP

