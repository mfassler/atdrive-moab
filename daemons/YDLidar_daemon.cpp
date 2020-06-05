/*
 * This module is intended for use with the YDLidar TG30, in its default
 * configuration.
 *
 */

#include "YDLidar_daemon.hpp"


extern void u_printf(const char *fmt, ...); // defined in main


// TODO?:  dupilicate in RTCM3_daemon...   maybe move to misc_math?
static int mod(int a, int b) {
	// The C/C++ version of the % operator doesn't return the same signs as
	// the Python version, so we use this:

	int r = a % b;
	return r < 0 ? r + b : r; 
}



YDLidar_daemon::YDLidar_daemon(PinName tx, PinName rx, UDPSocket *tx_sock) {
	_serport = new RawSerial(tx, rx, 500000);

	_sock = tx_sock;
	_serport->attach(callback(this, &YDLidar_daemon::_Serial_Rx_Interrupt));
}



void YDLidar_daemon::Start() {
	main_rx_thread.start(callback(this, &YDLidar_daemon::main_rx_worker));
	main_tx_thread.start(callback(this, &YDLidar_daemon::main_tx_worker));
}



void YDLidar_daemon::_Serial_Rx_Interrupt() {

	while (_serport->readable()) {

		_ringBuf[_inputIDX] = _serport->getc();

		_inputIDX++;
		if (_inputIDX >= _RING_BUFFER_SIZE) {
			_inputIDX = 0;
		}

		// Trigger out-going send on UDP:
		_event_flags.set(_EVENT_FLAG_YDLIDAR);
	}
}


void YDLidar_daemon::main_tx_worker() {
	ThisThread::sleep_for(1000);
	// force_stop:
	_serport->putc(0xa5);
	_serport->putc(0x00);

	// stop:
	_serport->putc(0xa5);
	_serport->putc(0x65);

	// get_device_health:
	_serport->putc(0xa5);
	_serport->putc(0x92);

	// get_device_info:
	_serport->putc(0xa5);
	_serport->putc(0x90);

	ThisThread::sleep_for(1000);

	// scan:
	_serport->putc(0xa5);
	_serport->putc(0x60);

	ThisThread::sleep_for(5000);
	// stop:
	_serport->putc(0xa5);
	_serport->putc(0x65);

}



void YDLidar_daemon::main_rx_worker() {

	uint32_t flags_read;
	char output_buffer[1029];

	while (true) {
		flags_read = _event_flags.wait_any(_EVENT_FLAG_YDLIDAR, 2000);

		if (flags_read & osFlagsError) {

			uint8_t plen_hibyte = _ringBuf[  mod(_outputIDX+1, _RING_BUFFER_SIZE) ];
			uint8_t plen_lobyte = _ringBuf[  mod(_outputIDX+2, _RING_BUFFER_SIZE) ];
			int bufLen = mod(_inputIDX-_outputIDX, _RING_BUFFER_SIZE);
			u_printf("YDLidar timeout!  %d %d %d %x %x %x\n",
				 _inputIDX, _outputIDX, bufLen, 

				_ringBuf[_outputIDX], plen_hibyte, plen_lobyte);

		} else {

			// Every YDLidar packet starts with either 0xa5 or 0xaa, so scan for that
			while (_inputIDX != _outputIDX) {
				char c1 = _ringBuf[ _outputIDX];
				if (c1 == 0xa5) {
					break;
				} else if (c1 == 0xaa) {
					break;
				} else {
					u_printf(" -- YDLidar missing magic start byte: %x\n", c1);
					_outputIDX++;
					if (_outputIDX >= _RING_BUFFER_SIZE) {
						_outputIDX = 0;
					}
				}
			}

			// A single-reponse packet is at least 7 bytes
			// A continuous-reponse packet is at least 10 bytes
			
			int bufLen = mod(_inputIDX-_outputIDX, _RING_BUFFER_SIZE);
			if (bufLen >= 7) {
				char c1 = _ringBuf[ _outputIDX];
				char c2 = _ringBuf[  mod(_outputIDX+1, _RING_BUFFER_SIZE) ];

				if (c1 == 0xa5 && c2 == 0x5a) {
					// This is a single-response packet

					char hbuf[5];
					hbuf[0] = _ringBuf[  mod(_outputIDX+2, _RING_BUFFER_SIZE) ];
					hbuf[1] = _ringBuf[  mod(_outputIDX+3, _RING_BUFFER_SIZE) ];
					hbuf[2] = _ringBuf[  mod(_outputIDX+4, _RING_BUFFER_SIZE) ];
					hbuf[3] = _ringBuf[  mod(_outputIDX+5, _RING_BUFFER_SIZE) ];
					hbuf[4] = _ringBuf[  mod(_outputIDX+6, _RING_BUFFER_SIZE) ];

					uint32_t *plen_p = (uint32_t*) hbuf;
					uint32_t plen = *plen_p & 0x3fffffff;

					if (plen > 50) {
						// SANITY CHECK -- I've never seen a packet this large
						u_printf("YDLidar protocol error.  huge packet: %d bytes\n", plen);
						plen = 50;
					}

					if (bufLen >= (plen + 7)) {
						uint8_t mode = (hbuf[3] & 0xc0) >> 6;
						uint8_t type = hbuf[4];

						u_printf("YDL, single: %d %d %d\n", plen, mode, type);

						// TODO:  actually parse the data here...


						_outputIDX = mod(_outputIDX + plen + 7, _RING_BUFFER_SIZE);
					}


				} else if (c1 == 0xaa && c2 == 0x55) {
					// This is a continuous-response packet
					uint8_t ct = _ringBuf[  mod(_outputIDX+2, _RING_BUFFER_SIZE) ];
					uint8_t lsn = _ringBuf[  mod(_outputIDX+3, _RING_BUFFER_SIZE) ];

					if (bufLen >= (lsn * 2 + 10)) {
						u_printf("YDL, cont: %d %d\n", ct, lsn);


						// TODO:  actually parse the data here...


						_outputIDX = mod(_outputIDX + lsn*2 + 10, _RING_BUFFER_SIZE);
					}

				} else {
					// parser error
					u_printf(" -- YDLidar missing magic 2nd byte\n");
					_outputIDX++;
					if (_outputIDX >= _RING_BUFFER_SIZE) {
						_outputIDX = 0;
					}
				}
			}





/*
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
*/

		}
	}
}


