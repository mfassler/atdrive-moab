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

#include "RTCM3_daemon.hpp"


extern void u_printf(const char *fmt, ...); // defined in main


int mod(int a, int b) {
	// The C/C++ version of the % operator doesn't return the same signs as
	// the Python version, so we use this:

	int r = a % b;
	return r < 0 ? r + b : r; 
}



RTCM3_daemon::RTCM3_daemon(PinName tx, PinName rx, UDPSocket *tx_sock) {
	_serport = new RawSerial(tx, rx, 115200);

	_sock = tx_sock;
	_serport->attach(callback(this, &RTCM3_daemon::_Serial_Rx_Interrupt));
}



void RTCM3_daemon::Start() {
	main_thread.start(callback(this, &RTCM3_daemon::main_worker));
}



void RTCM3_daemon::_Serial_Rx_Interrupt() {

	while (_serport->readable()) {

		_ringBuf[_inputIDX] = _serport->getc();

		_inputIDX++;
		if (_inputIDX >= _RING_BUFFER_SIZE) {
			_inputIDX = 0;
		}

		// Trigger out-going send on UDP:
		_event_flags.set(_EVENT_FLAG_RTCM3);
	}
}



void RTCM3_daemon::main_worker() {

	uint32_t flags_read;
	char output_buffer[1029];

	while (true) {
		flags_read = _event_flags.wait_any(_EVENT_FLAG_RTCM3, 250);

		if (flags_read & osFlagsError) {

			uint8_t plen_hibyte = _ringBuf[  mod(_outputIDX+1, _RING_BUFFER_SIZE) ];
			uint8_t plen_lobyte = _ringBuf[  mod(_outputIDX+2, _RING_BUFFER_SIZE) ];
			int bufLen = mod(_inputIDX-_outputIDX, _RING_BUFFER_SIZE);
			u_printf("RTCM3 timeout!  %d %d %d %x %x %x\n",
				 _inputIDX, _outputIDX, bufLen, 

				_ringBuf[_outputIDX], plen_hibyte, plen_lobyte);

		} else {


			// Every RTCM3 packet starts with a magic byte of 0xd3, so scan for that:
			while (_ringBuf[_outputIDX] != 0xd3 && (_inputIDX != _outputIDX)) {
				u_printf(" -- RTCM3 missing magic start byte\n");
				_outputIDX++;
				if (_outputIDX >= _RING_BUFFER_SIZE) {
					_outputIDX = 0;
				}
			}

			// RTCM3 packet is:
			//   - 1 char, magic start byte
			//   - 2 chars for payload length, max 1023
			//   -   ... payload ...
			//   - 3 chars for checksum
			//  so max size is: 3+1023+3 = 1029 bytes
			//     min size is: 3+3 = 6 bytes

			int bufLen = mod(_inputIDX-_outputIDX, _RING_BUFFER_SIZE);
			if (bufLen >= 6) {
				uint8_t plen_hibyte = _ringBuf[  mod(_outputIDX+1, _RING_BUFFER_SIZE) ];
				uint8_t plen_lobyte = _ringBuf[  mod(_outputIDX+2, _RING_BUFFER_SIZE) ];

				int plen = (((0x03 & plen_hibyte) << 8) | plen_lobyte) + 6;


				if (bufLen >= plen) {  // We have a complete RTCM3 packet
					if (plen_hibyte > 3) {
						u_printf("RTCM3 WARN: plen_hibyte: %d\n", plen_hibyte);
					}

					if ((_outputIDX + plen) < _RING_BUFFER_SIZE) {
						// Contiguous data, single copy:
						memcpy(output_buffer, &(_ringBuf[_outputIDX]), plen);

					} else {
						// wrapped around the ring buffer, so 2 copies:
						int len1 = _RING_BUFFER_SIZE - _outputIDX;
						int len2 = plen - len1;
						memcpy(output_buffer, &(_ringBuf[_outputIDX]), len1);
						memcpy(&(output_buffer[len1]), _ringBuf, len2);
					}

					if (output_buffer[0] != 0xd3) {
						u_printf("RTCM3 WARN: wtf: %d %d\n", _inputIDX, _outputIDX);
					}
					_outputIDX = mod(_outputIDX+plen, _RING_BUFFER_SIZE);

					int retval = _sock->sendto(_BROADCAST_IP_ADDRESS, UDP_PORT_GPS_RTCM3, 
						output_buffer, plen);
				}
			}
		}
	}
}


