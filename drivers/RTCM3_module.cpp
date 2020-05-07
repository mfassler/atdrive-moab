/*
 * This module is intended for use with the RTK base station.  
 * It is not needed for the rover.
 *
 * Intended usage:
 *  - receive RTCM3 messsages from a serial port
 *  - parse the boundaries of the RTCM3 messages and send out each message
 *    as its own UDP packet
 *
 */

#include "RTCM3_module.hpp"


extern void u_printf(const char *fmt, ...); // defined in main


// Ring buffer for serial-to-UDP operations:
#define _RING_BUFFER_SIZE 4
char _ringBuf[_RING_BUFFER_SIZE][1030];
int _bufLen[_RING_BUFFER_SIZE] = {0, 0, 0, 0};
int _inputIDX = 0;
int _outputIDX = 0;



RTCM3_module::RTCM3_module(PinName tx, PinName rx, UDPSocket *tx_sock) {
	_serport = new RawSerial(tx, rx, 115200);

	_sock = tx_sock;
	_serport->attach(callback(this, &RTCM3_module::_Serial_Rx_Interrupt));
}



void RTCM3_module::Start() {
	main_thread.start(callback(this, &RTCM3_module::main_worker));
}



void RTCM3_module::_Serial_Rx_Interrupt() {

	int c;

	int plen = 0;
	while (_serport->readable()) {
		c = _serport->getc();

		if (c == 0xd3) {  // magic start byte
			_ringBuf[_inputIDX][0] = c;
			_bufLen[_inputIDX] = 1;

			c = _serport->getc();  //packet_length, high byte
			_ringBuf[_inputIDX][1] = c;
			_bufLen[_inputIDX] = 2;
			if (c > 3) {
				u_printf("WARN: plen hi-byte: %d\n", c);
			}
			plen = (0x03 & c) << 8;

			c = _serport->getc();  //packet_length, lo byte
			_ringBuf[_inputIDX][2] = c;
			_bufLen[_inputIDX] = 3;
			plen |= c;

			plen += 3;  // also get the cksum at the end

			// Read the packet
			for (int i=0; i<plen; ++i) {
				_ringBuf[_inputIDX][_bufLen[_inputIDX]] = _serport->getc();
				_bufLen[_inputIDX]++;
			}

			// Trigger out-going send on UDP:
			_event_flags.set(_EVENT_FLAG_RTCM3);

			_inputIDX++;
			if (_inputIDX >= _RING_BUFFER_SIZE) {
				_inputIDX = 0;
			}

		} else {
			plen = 0;
		}
	}
}



void RTCM3_module::main_worker() {

	uint32_t flags_read;

	while (true) {
		flags_read = _event_flags.wait_any(_EVENT_FLAG_RTCM3, 1200);

		if (flags_read & osFlagsError) {

			u_printf("RTCM3 timeout!\n");

		} else {

			while (_outputIDX != _inputIDX) {
				int retval = _sock->sendto(_BROADCAST_IP_ADDRESS, UDP_PORT_GPS_RTCM3, 
						_ringBuf[_outputIDX], _bufLen[_outputIDX]);

				_outputIDX++;
				if (_outputIDX >= _RING_BUFFER_SIZE) {
					_outputIDX = 0;
				}
			}
		}
	}
}


