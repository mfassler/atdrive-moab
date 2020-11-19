/*
 * This module is intended for use with a custom receiver, which
 * takes the place of the Futaba->S.Bus system.
 *
 *
 */

#include "Radio169_daemon.hpp"


extern void u_printf(const char *fmt, ...); // defined in main


static int mod(int a, int b) {
	// The C/C++ version of the % operator doesn't return the same signs as
	// the Python version, so we use this:

	int r = a % b;
	return r < 0 ? r + b : r;
}



Radio169_daemon::Radio169_daemon(PinName tx, PinName rx, UDPSocket *tx_sock) {
	// TODO?  tx is not needed...
	_serport = new RawSerial(tx, rx, 57600);

	_sock = tx_sock;
	_serport->attach(callback(this, &Radio169_daemon::_Serial_Rx_Interrupt));

	_stop_release_time = rtos::Kernel::get_ms_count();

	timeout = true;
	requested_moab_state = Stop;

}

void Radio169_daemon::attachCallback(Callback<void(bool)>fp) {
	_callback = fp;
}

void Radio169_daemon::Start() {
	main_thread.start(callback(this, &Radio169_daemon::main_worker));
}



void Radio169_daemon::_Serial_Rx_Interrupt() {

	while (_serport->readable()) {

		_ringBuf[_inputIDX] = _serport->getc();

		_inputIDX++;
		if (_inputIDX >= _RING_BUFFER_SIZE169) {
			_inputIDX = 0;
		}

		// Trigger out-going send on UDP:
		_event_flags.set(_EVENT_FLAG_RADIO169);
	}
}



void Radio169_daemon::_parse_vals() {
	if (output_buffer[0] != 0x80) {
		u_printf("r169:  BAD PACKET\n");
		return;
	}

	controller_values.start = (bool) (output_buffer[1] & 0x10);
	controller_values.back = (bool) (output_buffer[1] & 0x20);
	controller_values.logitech = (bool) (output_buffer[1] & 0x40);

	controller_values.LB = (bool) (output_buffer[1] & 0x01);
	controller_values.LT = (bool) (output_buffer[1] & 0x02);
	controller_values.RB = (bool) (output_buffer[1] & 0x04);
	controller_values.RT = (bool) (output_buffer[1] & 0x08);

	controller_values.Y = (bool) (output_buffer[2] & 0x10);  // orange/yellow
	controller_values.A = (bool) (output_buffer[2] & 0x20);  // green
	controller_values.B = (bool) (output_buffer[2] & 0x40);  // red
	controller_values.X = (bool) (output_buffer[2] & 0x80);  // blue

	controller_values.dpad_up    = (bool) (output_buffer[2] & 0x01);
	controller_values.dpad_down  = (bool) (output_buffer[2] & 0x02);
	controller_values.dpad_right = (bool) (output_buffer[2] & 0x04);
	controller_values.dpad_left  = (bool) (output_buffer[2] & 0x08);

	controller_values.leftjoy_lr = output_buffer[3] - 64;
	controller_values.leftjoy_ud = output_buffer[4] - 64;
	controller_values.rightjoy_lr = output_buffer[5] - 64;
	controller_values.rightjoy_ud = output_buffer[6] - 64;

	timeout = false;

	// Fake SBus values, since Moab was originally built around S.Bus:
	sb_steering = 1024 + (int) controller_values.rightjoy_lr * 10 * _SCALE_STEERING;

	if (controller_values.leftjoy_ud > 0) {
		sb_throttle = 1024 + (int) controller_values.leftjoy_ud * 10 * _SCALE_THROTTLE;
	} else {
		sb_throttle = 1024 + (int) controller_values.leftjoy_ud * 10 * _SCALE_BRAKE;
	}

	_stateful_stuff();
	_callback(false);
}


void Radio169_daemon::_stateful_stuff(void) {


	if (controller_values.X) {
		if (_blue_button_state == no_press) {
			_blue_button_state = press;
			if (requested_moab_state == Manual) {
				requested_moab_state = Auto;
				u_printf("User toggle:  Manual --> Auto\n");
			} else if (requested_moab_state == Auto) {
				requested_moab_state = Manual;
				u_printf("User toggle:  Auto --> Manual\n");
			}
		}
	} else {
		if (_blue_button_state == press) {
			_blue_button_state = no_press;
		}
	}


	if (controller_values.LB) {  // LB: stop with full brakes

		requested_moab_state = Stop;
		u_printf("USER EMERGENCY STOP\n");

	} else if (controller_values.RB) { // RB: stop with neutral brake/throttle

		requested_moab_state = Stop_no_brakes;
		u_printf("User stop\n");

	} else if (controller_values.LT && controller_values.RT) {
		// LT and RT must be held down for 2 seconds to release emergency stop
		if (_stop_release_state == no_press) {

			_stop_release_time = rtos::Kernel::get_ms_count();
			_stop_release_state = press;

		} else {
			if (rtos::Kernel::get_ms_count() - _stop_release_time > 2000) {
				requested_moab_state = Manual;
				u_printf("User emergency stop release\n");
			}
		}

	} else {
		_stop_release_state = no_press;
	}
}


void Radio169_daemon::main_worker() {

	uint32_t flags_read;

	while (true) {
		flags_read = _event_flags.wait_any(_EVENT_FLAG_RADIO169, 350);

		if (flags_read & osFlagsError) {

			timeout = true;
			_callback(true);
			uint8_t plen_hibyte = _ringBuf[  mod(_outputIDX+1, _RING_BUFFER_SIZE169) ];
			uint8_t plen_lobyte = _ringBuf[  mod(_outputIDX+2, _RING_BUFFER_SIZE169) ];
			int bufLen = mod(_inputIDX-_outputIDX, _RING_BUFFER_SIZE169);
			u_printf("Radio169 timeout!  %d %d %d %x %x %x\n",
				 _inputIDX, _outputIDX, bufLen,

				_ringBuf[_outputIDX], plen_hibyte, plen_lobyte);

		} else {


			// Every Radio169 packet starts with 4 magic bytes:
			// B0 A3 C5 4D, so scan for that
			while (_ringBuf[_outputIDX] != 0xb0 && (_inputIDX != _outputIDX)) {

				// TODO, FIXME:  This seems to happen alot...  alot more than
				// my Python-based parser....   something weird is happening...
				u_printf(" -- Radio169 missing magic start byte\n");
				_outputIDX++;
				if (_outputIDX >= _RING_BUFFER_SIZE169) {
					_outputIDX = 0;
				}
			}

			// Radio169 packet is:
			//   - 4 chars, magic start bytes:  B0 A3 C5 4D
			//   - 1 chars for payload type
			//   - 1 char unused
			//   - 2 chars for total message length, big-endian
			//   -   ... payload -- 8 bytes that we care about...
			//   -  sometimes there are 2 bytes at the end that we don't care about (cksum??)

			// packets from the serial port receiver will be either 16 bytes or 21 bytes
			//   (... I think on the receiver side, we will only ever see 21 bytes...)
			// we only care about the 8 bytes in the middle.
			//  (actually, there's only 6 bytes that we care about...)


			int bufLen = mod(_inputIDX-_outputIDX, _RING_BUFFER_SIZE169);
			if (bufLen >= 16) {
				uint8_t magicChar1 = _ringBuf[  mod(_outputIDX+1, _RING_BUFFER_SIZE169) ];
				uint8_t magicChar2 = _ringBuf[  mod(_outputIDX+2, _RING_BUFFER_SIZE169) ];
				uint8_t magicChar3 = _ringBuf[  mod(_outputIDX+3, _RING_BUFFER_SIZE169) ];

				if (magicChar1 != 0xa3) {
					u_printf(" -- Radio169 bad magic start byte 1\n");
					_outputIDX++;
					if (_outputIDX >= _RING_BUFFER_SIZE169) {
						_outputIDX = 0;
					}
					//return;
					goto THE_END;
				}

				if (magicChar2 != 0xc5) {
					u_printf(" -- Radio169 bad magic start byte 2\n");
					_outputIDX++;
					if (_outputIDX >= _RING_BUFFER_SIZE169) {
						_outputIDX = 0;
					}
					//return;
					goto THE_END;
				}

				if (magicChar3 != 0x4d) {
					u_printf(" -- Radio169 bad magic start byte 3\n");
					_outputIDX++;
					if (_outputIDX >= _RING_BUFFER_SIZE169) {
						_outputIDX = 0;
					}
					//return;
					goto THE_END;
				}

				// ******* We have a good header!
				// Let's see if we have the entire packet:...

				uint8_t msgType     = _ringBuf[  mod(_outputIDX+4, _RING_BUFFER_SIZE169) ];
				//uint8_t nothing   = _ringBuf[  mod(_outputIDX+5, _RING_BUFFER_SIZE169) ];
				uint8_t plen_hibyte = _ringBuf[  mod(_outputIDX+6, _RING_BUFFER_SIZE169) ];
				uint8_t plen_lobyte = _ringBuf[  mod(_outputIDX+7, _RING_BUFFER_SIZE169) ];

				int plen = (plen_hibyte << 8) | plen_lobyte;


				if (bufLen >= plen) {  // We have a complete packet
					if (msgType == 3 && plen == 16) {
						// Good packet
						u_printf(" -- Radio169 unsupported: msgType: %d, plen: %d\n",
								msgType, plen);
						_outputIDX++;
						if (_outputIDX >= _RING_BUFFER_SIZE169) {
							_outputIDX = 0;
						}
						//return;
						//goto THE_END;

					} else if (msgType == 2 && plen == 21) {
						// Good packet
						output_buffer[0] = _ringBuf[ mod(_outputIDX+11, _RING_BUFFER_SIZE169) ];
						output_buffer[1] = _ringBuf[ mod(_outputIDX+12, _RING_BUFFER_SIZE169) ];
						output_buffer[2] = _ringBuf[ mod(_outputIDX+13, _RING_BUFFER_SIZE169) ];
						output_buffer[3] = _ringBuf[ mod(_outputIDX+14, _RING_BUFFER_SIZE169) ];
						output_buffer[4] = _ringBuf[ mod(_outputIDX+15, _RING_BUFFER_SIZE169) ];
						output_buffer[5] = _ringBuf[ mod(_outputIDX+16, _RING_BUFFER_SIZE169) ];
						output_buffer[6] = _ringBuf[ mod(_outputIDX+17, _RING_BUFFER_SIZE169) ];
						output_buffer[7] = _ringBuf[ mod(_outputIDX+18, _RING_BUFFER_SIZE169) ];

						_outputIDX = mod(_outputIDX+plen, _RING_BUFFER_SIZE169);

						int retval = _sock->sendto(_BROADCAST_IP_ADDRESS, UDP_PORT_RADIO169, 
							output_buffer, 8);

						_parse_vals();

					} else {
						// WTF?
						u_printf(" -- Radio169 error: msgType: %d, plen: %d\n",
								msgType, plen);
						_outputIDX++;
						if (_outputIDX >= _RING_BUFFER_SIZE169) {
							_outputIDX = 0;
						}
						//return;
						//goto THE_END;
					}
				}
			}
		}

		THE_END:
			continue;
	}
}


