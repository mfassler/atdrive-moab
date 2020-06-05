#ifndef __YTLIDAR_DAEMON_HPP
#define __YTLIDAR_DAEMON_HPP

#include "mbed.h"
#include "EthernetInterface.h"

#include "ROBOT_CONFIG.hpp"
#include "EVENT_FLAGS.hpp"
#include "MOAB_DEFINITIONS.h"


class YDLidar_daemon {

public:
	YDLidar_daemon(PinName, PinName, UDPSocket*);

	void Start();


private:
	UDPSocket *_sock;
	RawSerial *_serport;

	EventFlags _event_flags;

	Thread main_rx_thread;
	Thread main_tx_thread;

	void main_rx_worker(void);
	void main_tx_worker(void);

	void _Serial_Rx_Interrupt(void);


	// Ring buffer for serial-to-UDP operations:
	#define _RING_BUFFER_SIZE 512

	char _ringBuf[_RING_BUFFER_SIZE];
	int _inputIDX = 0;
	int _outputIDX = 0;


};

#endif // __YTLIDAR_DAEMON_HPP
