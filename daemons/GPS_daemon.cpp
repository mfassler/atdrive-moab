
#include "GPS_daemon.hpp"

extern void u_printf(const char *fmt, ...);  // Defined in main()



GPS_daemon::GPS_daemon(PinName tx, PinName rx, EthernetInterface *net) {
	_serport = new BufferedSerial(tx, rx, 115200);  //tx, then rx

	_sock = new UDPSocket();
	_sock->open(net);
	_sock->bind(UDP_PORT_GPS_NMEA);

	_destSockAddr.set_ip_address(_BROADCAST_IP_ADDRESS);
	_destSockAddr.set_port(UDP_PORT_GPS_NMEA);

}


void GPS_daemon::Start() {
	serial_rx_thread.start(callback(this, &GPS_daemon::serial_rx_worker));
	udp_rx_thread.start(callback(this, &GPS_daemon::udp_rx_worker));
}


void GPS_daemon::serial_rx_worker() {
	#define _NMEA_MAX_LEN 128
	char buffer[_NMEA_MAX_LEN];
	int i = 0;

	while (true) {
		_serport->read(&(buffer[i]), 1);

		// All NMEA sentences start with either '$' or '!'.
		// All NMEA sentences end with a linefeed, 0x0a.
		// Max sentence length is 82 characters.  

		// But we're just going to look for 0x0a or a full buffer:
		i++;

		if ( (buffer[i-1] == 0x0a) || (i >= _NMEA_MAX_LEN)) {

			_sock->sendto(_destSockAddr, buffer, i);
			i = 0;

		}
	}
}


void GPS_daemon::udp_rx_worker() {
	/*
	 * If the auto-pilot wants to send messages to the GPS modules, we handle that
	 * here.  This is intended for RTCM3 messages.
	 */

	SocketAddress sockAddr;
	// RTCM3 max packet size is 1029
	// ethernet MTU is about 1500
	char inputBuffer[1500];

	while (true) {
		int n = _sock->recvfrom(&sockAddr, inputBuffer, 1500);
		_serport->write(inputBuffer, n);
	}
}

