/*
 * This module is intended for use with a custom receiver, which
 * takes the place of the Futaba->S.Bus system.
 *
 *
 */

#include "Radio169_daemon.hpp"


extern void u_printf(const char *fmt, ...); // defined in main



Radio169_daemon::Radio169_daemon(PinName tx, PinName rx, UDPSocket *tx_sock) {
	// TODO?  tx is not needed...
	_serport = new BufferedSerial(tx, rx, 57600);

	_sock = tx_sock;

	_stop_release_time = Kernel::Clock::now();

	_destSockAddr.set_ip_address(_BROADCAST_IP_ADDRESS);
	_destSockAddr.set_port(UDP_PORT_RADIO169);

	timeout = true;
	requested_moab_state = Stop;

}


void Radio169_daemon::attachCallback(Callback<void()>fp) {
	_callback = fp;
}


void Radio169_daemon::Start() {
	main_thread.start(callback(this, &Radio169_daemon::main_worker));
	timeout_thread.start(callback(this, &Radio169_daemon::timeout_worker));
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

	// The r169 seems to have a deadzone from 0 to 11 (+) and 0 to -11 (-)

	// Dead-zone and steering eqs:
	//  from 0 to 10, ignore (set to 0)
	//  from 10 to 39, go from 0.0 to 0.25
	//  from 40 to 63, go from 0.25 to 1.0

	float sPercent;
/*
	float SK1 = 0.25 / 29.0;  // 29 steps to increase by 0.25
	float SK2 = 0.75 / 23.0;  // 23 steps to increase by 0.75

	if (controller_values.rightjoy_lr > 39) {
		sPercent = 0.25 + SK2 * (controller_values.rightjoy_lr - 39);
		sb_steering = 1024 + (int) (672 * sPercent) * _SCALE_STEERING;
	} else if (controller_values.rightjoy_lr > 10) {
		sPercent = SK1 * (controller_values.rightjoy_lr - 10);
		sb_steering = 1024 + (int) (672 * sPercent) * _SCALE_STEERING;
	} else if (controller_values.rightjoy_lr < -39) {
		sPercent = 0.25 + SK2 * (-controller_values.rightjoy_lr - 39);
		sb_steering = 1024 - (int) (672 * sPercent) * _SCALE_STEERING;
	} else if (controller_values.rightjoy_lr < -10) {
		sPercent = SK1 * (-controller_values.rightjoy_lr - 10);
		sb_steering = 1024 - (int) (672 * sPercent) * _SCALE_STEERING;
	} else {
		sb_steering = 1024;
	}
*/

	if (controller_values.rightjoy_lr > 10) {
		sPercent = (expf((controller_values.rightjoy_lr-10.0f) / 20.0f) - 1.0f) / (expf(53.0f/20.0f) - 1.0f);
		sb_steering = 1024 + (int) (672 * sPercent) * _SCALE_STEERING;
	} else if (controller_values.rightjoy_lr < -10) {
		sPercent = (expf((-controller_values.rightjoy_lr-10.0f) / 20.0f) - 1.0f) / (expf(53.0f/20.0f) - 1.0f);
		sb_steering = 1024 - (int) (672 * sPercent) * _SCALE_STEERING;
	} else {
		sb_steering = 1024;
	}


	// Dead-zone and throttle stuff...
	//  from 0 to 10, ignore (set to 0)
	// from 11 to 31 go from 0.0 to 0.2
	// from 32 to 63 go from 0.2 to 0.4
	//
	// if holding max throttle for 2 seconds, go to 0.5

	//float K1 = 0.2 / 29.0;  // 21 steps to increase by 0.2
	//float K2 = 0.2 / 23.0;  // 31 steps to increase by 0.2

	float tPercent;

	if (controller_values.leftjoy_ud == 63) {
		if (_max_throttle_state == no_press) {
			_max_throttle_state = press;
			_max_throttle_time = Kernel::Clock::now();
		}
		if (Kernel::Clock::now() - _max_throttle_time > 3s) {
			tPercent = 0.5;
		} else {
			tPercent = 0.4;
		}
		sb_throttle = 1024 + (int) (672 * tPercent) * _SCALE_THROTTLE;

	} else if (controller_values.leftjoy_ud > 10) {
		_max_throttle_state = no_press;
		sPercent = 0.4f * (expf((controller_values.leftjoy_ud-10.0f) / 20.0f) - 1.0f) / (expf(52.0f/20.0f) - 1.0f);
		sb_throttle = 1024 + (int) (672 * sPercent) * _SCALE_THROTTLE;
	} else if (controller_values.leftjoy_ud > 0) {
		_max_throttle_state = no_press;
		sb_throttle = 1024;
	} else {
		_max_throttle_state = no_press;
		sb_throttle = 1024 + (int) controller_values.leftjoy_ud * 10 * _SCALE_BRAKE;
	}

	_stateful_stuff();
	_callback();
}


void Radio169_daemon::_stateful_stuff(void) {

	if (controller_values.A) {  // Auto PGM 1

		requested_moab_state = Auto;
		u_printf("User auto A\n");

	} else if (controller_values.start) {  // Follow mode

		requested_moab_state = Auto;
		u_printf("User auto start\n");

	} else if (controller_values.Y) {  // Auto PGM 3

		requested_moab_state = Auto;
		u_printf("User auto Y\n");

	} else if (controller_values.LB) {  // LB: stop with full brakes

		requested_moab_state = Stop;
		u_printf("User STOP\n");

	} else if (controller_values.RB) { // RB: Manual

		requested_moab_state = Manual;
		u_printf("User Manual\n");

	} else if (controller_values.LT && controller_values.RT) {
		// LT and RT must be held down for 0.1 seconds to release stop
		if (_stop_release_state == no_press) {

			_stop_release_time = Kernel::Clock::now();
			_stop_release_state = press;

		} else {
			if (Kernel::Clock::now() - _stop_release_time > 100ms) {
				requested_moab_state = Auto;
				u_printf("Pause Cancel\n");
			}
		}

	} else {
		_stop_release_state = no_press;
	}
}


void Radio169_daemon::main_worker() {

	char c;

	uint8_t msgType;
	uint8_t nothing;
	uint8_t plen_hibyte;
	uint8_t plen_lobyte;

	while (true) {

		// Every Radio169 packet starts with 4 magic bytes:
		//   B0 A3 C5 4D
		// ...so scan for that

		_serport->read(&c, 1);
		if (c == 0xb0) {
			_serport->read(&c, 1);
			if (c == 0xa3) {
				_serport->read(&c, 1);
				if (c == 0xc5) {
					_serport->read(&c, 1);
					if (c == 0x4d) {

						_serport->read(&msgType, 1);
						_serport->read(&nothing, 1);
						_serport->read(&plen_hibyte, 1);
						_serport->read(&plen_lobyte, 1);

						int plen = (plen_hibyte << 8) | plen_lobyte;

						if (msgType == 0x03 && plen == 16) {

							u_printf("r169: msgType 0x03 not supported.\n");

						} else if (msgType == 0x02 && plen == 21) {

							_serport->read(&nothing, 1);
							_serport->read(&nothing, 1);
							_serport->read(&nothing, 1);
							_serport->read(&(output_buffer[0]), 1);
							_serport->read(&(output_buffer[1]), 1);
							_serport->read(&(output_buffer[2]), 1);
							_serport->read(&(output_buffer[3]), 1);
							_serport->read(&(output_buffer[4]), 1);
							_serport->read(&(output_buffer[5]), 1);
							_serport->read(&(output_buffer[6]), 1);
							_serport->read(&(output_buffer[7]), 1);
							_serport->read(&nothing, 1);
							_serport->read(&nothing, 1);

							_sock->sendto(_destSockAddr, output_buffer, 8);
							_parse_vals();

							// reset the timeout:
							_event_flags.set(_EVENT_FLAG_RADIO169);

						} else {
							u_printf("r169: unknown packet: %02x %02x\n", msgType, plen);
						}
					} else {
						u_printf("r169: bad magic byte #3\n");
					}
				} else {
					u_printf("r169: bad magic byte #2\n");
				}
			} else {
				u_printf("r169: bad magic byte #1\n");
			}
		} else {
			// TODO, FIXME:  this happens alot with Moab, but not with my Python
			// coding using a USB-FTDi receiver.  What's up with that...
			//   ... wondering if the Moab is a bit more sensitive to radio noise than
			//       the FTDi chip...
			//u_printf("r169: bad magic byte #0: %02x\n", c);
		}
	}
}


void Radio169_daemon::timeout_worker() {
	uint32_t flags_read;

	while (true) {
		flags_read = _event_flags.wait_any(_EVENT_FLAG_RADIO169, 500);

		if (flags_read & osFlagsError) {
			u_printf("r169: timeout!\n");

			timeout = true;
		} else {
			timeout = false;
		}
	}
}


