
#include "GPS_daemon.hpp"

extern void u_printf(const char *fmt, ...);  // Defined in main()



// Incoming buffer, from serial port:
char _gpsRxBuf[1500];
int _gpsRxBufLen = 0;

// Outgoing buffers (circular buffer), via network UDP:
#define _GPS_RING_BUFFER_SIZE 8
char _gpsTxBuf[_GPS_RING_BUFFER_SIZE][1500];
int _gpsTxBufLen[_GPS_RING_BUFFER_SIZE] = {0, 0, 0, 0};
int _gpsTxBufIdxFI = 0;
int _gpsTxBufIdxFO = 0;






GPS_daemon::GPS_daemon(PinName tx, PinName rx, EthernetInterface *net) {
	_gps_in = new UnbufferedSerial(tx, rx, 115200);  //tx, then rx

	_sock = new UDPSocket();
	_sock->open(net);
	_sock->bind(UDP_PORT_GPS_NMEA);

	_destSockAddr.set_ip_address(_BROADCAST_IP_ADDRESS);
	_destSockAddr.set_port(UDP_PORT_GPS_NMEA);

	_gps_in->attach(callback(this, &GPS_daemon::_Gps_Rx_Interrupt));

}


void GPS_daemon::Start() {
	main_thread.start(callback(this, &GPS_daemon::main_worker));
	udp_rx_thread.start(callback(this, &GPS_daemon::udp_rx_worker));
}




void GPS_daemon::main_worker() {

	uint32_t flags_read;

	while (true) {
		flags_read = _event_flags.wait_any(_EVENT_FLAG_GPS, 1200);

		if (flags_read & osFlagsError) {

			u_printf("GPS timeout!\n");

		} else {

			while (_gpsTxBufIdxFO != _gpsTxBufIdxFI) {
				int retval = _sock->sendto(_destSockAddr, _gpsTxBuf[_gpsTxBufIdxFO], _gpsTxBufLen[_gpsTxBufIdxFO]);

				_gpsTxBufIdxFO++;
				if (_gpsTxBufIdxFO >= _GPS_RING_BUFFER_SIZE) {
					_gpsTxBufIdxFO = 0;
				}

				//if (retval < 0 && NETWORK_IS_UP) {
				//	printf("UDP socket error in gps_reTx_worker\r\n");
				//}
			}
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
		_gps_in->write(inputBuffer, n);
	}
}



void GPS_daemon::_Gps_Rx_Interrupt() {
	char c;
	while (_gps_in->readable()) {

		_gps_in->read(&c, 1);
		_gpsRxBuf[_gpsRxBufLen] = c;
		_gpsRxBufLen++;

		if ((c == 0x0a) || (_gpsRxBufLen > 1400)) {

			memcpy(_gpsTxBuf[_gpsTxBufIdxFI], _gpsRxBuf, _gpsRxBufLen);
			_gpsTxBufLen[_gpsTxBufIdxFI] = _gpsRxBufLen;

			_gpsTxBufIdxFI++;
			if (_gpsTxBufIdxFI >= _GPS_RING_BUFFER_SIZE) {
				_gpsTxBufIdxFI = 0;
			}

			_event_flags.set(_EVENT_FLAG_GPS);
			_gpsRxBufLen = 0;
		}
	}
}


