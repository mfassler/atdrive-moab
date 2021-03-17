
#include "PushButton_daemon.hpp"

extern void u_printf(const char *fmt, ...);  // Defined in main()



PushButton_daemon::PushButton_daemon(PinName _pin, UDPSocket *txsock) {
	_button = new InterruptIn(_pin, PullUp);
	_sock = txsock;

	_destSockAddr.set_ip_address(_BROADCAST_IP_ADDRESS);
	_destSockAddr.set_port(UDP_PORT_PUSHBUTTON);
}


void PushButton_daemon::Start() {

	_button->rise(callback(this, &PushButton_daemon::_interrupt));
	_button->fall(callback(this, &PushButton_daemon::_interrupt));


	main_thread.start(callback(this, &PushButton_daemon::main_worker));
}




void PushButton_daemon::_interrupt() {

	if (_button->read()) {
		_last_pgm_rise = rtos::Kernel::get_ms_count();
	} else {
		_last_pgm_fall = rtos::Kernel::get_ms_count();
	}
}



void PushButton_daemon::main_worker() {

	while (true) {
		ThisThread::sleep_for(20);


		// We do all this time-stamp weirdness to de-bounce the switch
		uint64_t ts_ms = rtos::Kernel::get_ms_count();

		// Switch must be stable for at least 30ms:
		if ((ts_ms - _last_pgm_fall > 30) && (ts_ms - _last_pgm_rise > 30)) {
			// current stable value:
			uint8_t val = _button->read();
			if (_pgm_value_debounce != val) { // debounce value has changed
				_pgm_value_debounce = val;
				if (val) {
					_last_pgm_rise_debounce = ts_ms;
					_pgm_notice_sent = false;
				} else {
					_last_pgm_fall_debounce = ts_ms;
				}
			}
		}

		if (!_pgm_notice_sent) {
			if (!_pgm_value_debounce) {
				if (ts_ms - _last_pgm_rise_debounce > 300) {

					int retval = _sock->sendto(_destSockAddr, &ts_ms, sizeof(ts_ms));

					//if (retval < 0 && NETWORK_IS_UP) {
					//	printf("UDP socket error in Check_Pgm_Button\r\n");
					//}
					_pgm_notice_sent = true;
				}
			}
		}
	}
}


