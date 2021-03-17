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
	SocketAddress _destSockAddr;
	UnbufferedSerial *_serport;

	EventFlags _event_flags;

	Thread main_thread;

	void main_worker(void);
	void _Serial_Rx_Interrupt(void);


	// Ring buffer for serial-to-UDP operations:
	#define _RING_BUFFER_SIZE 4096

	char _ringBuf[_RING_BUFFER_SIZE];
	int _inputIDX = 0;
	int _outputIDX = 0;


};

#endif // __RTCM3_MODULE_HPP
