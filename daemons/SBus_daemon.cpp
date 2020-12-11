
#include "SBus_daemon.hpp"

extern void u_printf(const char *fmt, ...); // defined in main


SBus_daemon::SBus_daemon(PinName rx, UDPSocket *tx_sock) {

	// S.Bus is 100000Hz, 8E2, electrically inverted
	_serport = new RawSerial(NC, rx, 100000);  // tx, then rx
	_serport->format(8, SerialBase::Even, 2); // S.Bus is 8E2
	_parser = new SbusParser(&sbup);

	_sock = tx_sock;
	_serport->attach(callback(this, &SBus_daemon::_serial_rx_interrupt));

	timeout = true;
	requested_moab_state = NoSignal;
}


void SBus_daemon::attachCallback(Callback<void()>fp) {
	_callback = fp;
}


void SBus_daemon::Start() {
	main_thread.start(callback(this, &SBus_daemon::main_worker));
}


void SBus_daemon::_serial_rx_interrupt() {
	int c;

	while (_serport->readable()) {

		c = _serport->getc();
		int status = _parser->rx_char(c);

		if (status == 1) {

			if (sbup.failsafe) {
				requested_moab_state = NoSignal;
			} else if (sbup.ch5 < 688) {
				requested_moab_state = Stop;
			} else if (sbup.ch5 < 1360) {
				requested_moab_state = Manual;
			} else {
				requested_moab_state = Auto;
			}

			_event_flags.set(_EVENT_FLAG_SBUS);
		}
	}
}


void SBus_daemon::main_worker() {
	uint32_t flags_read;

	while (true) {
		flags_read = _event_flags.wait_any(_EVENT_FLAG_SBUS, 100);

		if (flags_read & osFlagsError) {
			timeout = true;
			_callback();
			//u_printf("S.Bus timeout!\n");
		} else {
			timeout = false;
			_callback();
			int retval = _sock->sendto(_AUTOPILOT_IP_ADDRESS, UDP_PORT_SBUS,
				(char *) &sbup, sizeof(struct sbus_udp_payload));

			//if (retval < 0 && NETWORK_IS_UP) {
			//	printf("UDP socket error in sbus_reTx_worker\r\n");
			//}
		}
	}
}


