#ifndef __GPS_DAEMON_HPP
#define __GPS_DAEMON_HPP

#include "mbed.h"
#include "EthernetInterface.h"

#include "ROBOT_CONFIG.hpp"
#include "MOAB_DEFINITIONS.h"



class GPS_daemon {
public:
	GPS_daemon(PinName, PinName, EthernetInterface*);

	void Start();

private:
	UDPSocket *_sock;
	SocketAddress _destSockAddr;
	BufferedSerial *_serport;

	Thread serial_rx_thread;
	Thread udp_rx_thread;

	void serial_rx_worker();
	void udp_rx_worker();
};

#endif // __GPS_DAEMON_HPP
