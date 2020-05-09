
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
	_gps_in = new RawSerial(tx, rx, 115200);  //tx, then rx

	_sock = new UDPSocket();
	_sock->open(net);
	_sock->bind(UDP_PORT_GPS_NMEA);

	_gps_in->attach(callback(this, &GPS_daemon::_Gps_Rx_Interrupt));

}


void GPS_daemon::Start() {
	main_thread.start(callback(this, &GPS_daemon::main_worker));
}




void GPS_daemon::main_worker() {

	uint32_t flags_read;

	while (true) {
		flags_read = _event_flags.wait_any(_EVENT_FLAG_GPS, 1200);

		if (flags_read & osFlagsError) {

			u_printf("GPS timeout!\n");

		} else {

			while (_gpsTxBufIdxFO != _gpsTxBufIdxFI) {
				int retval = _sock->sendto(_BROADCAST_IP_ADDRESS, UDP_PORT_GPS_NMEA,
						_gpsTxBuf[_gpsTxBufIdxFO], _gpsTxBufLen[_gpsTxBufIdxFO]);

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



void GPS_daemon::_Gps_Rx_Interrupt() {
	int c;
	while (_gps_in->readable()) {

		c = _gps_in->getc();
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


