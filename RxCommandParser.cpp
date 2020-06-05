
#include "mbed.h"
#include "RxCommandParser.hpp"
#include "ROBOT_CONFIG.hpp"


extern void u_printf(const char *fmt, ...);  // Defined in main()


RxCommandParser::RxCommandParser(EthernetInterface *net) {

	auto_ch1 = 1024;
	auto_ch2 = 1024;

	_rx_sock = new UDPSocket();
	_rx_sock->open(net);
	_rx_sock->bind(12346);

#ifdef USER_DIGITAL_OUT_0
	user_dout_0 = new DigitalOut(USER_DIGITAL_OUT_0);
	*user_dout_0 = 0;
#endif // USER_DIGITAL_OUT_0

#ifdef USER_PWM_OUT_0
	user_pout_0 = new PwmOut(USER_PWM_OUT_0);
	user_pout_0->period(0.02);
	user_pout_0->write(0.0);
#endif // USER_PWM_OUT_0

#ifdef USER_PWM_OUT_1
	user_pout_1 = new PwmOut(USER_PWM_OUT_1);
	user_pout_1->period(0.02);
	user_pout_1->write(0.0);
#endif // USER_PWM_OUT_1

#ifdef USER_PWM_OUT_2
	user_pout_2 = new PwmOut(USER_PWM_OUT_2);
	user_pout_2->period(0.02);
	user_pout_2->write(0.0);
#endif // USER_PWM_OUT_2

#ifdef USER_PWM_OUT_3
	user_pout_3 = new PwmOut(USER_PWM_OUT_3);
	user_pout_3->period(0.02);
	user_pout_3->write(0.0);
#endif // USER_PWM_OUT_3


#ifdef USER_SERVO_OUT_0  // assume a Futaba-style pulse-width in micro-seconds
	user_servo_0 = new PwmOut(USER_SERVO_OUT_0);
	user_servo_0->period_ms(15);  // same period as Futaba S.Bus
	user_servo_0->pulsewidth_us(0);
#endif // USER_SERVO_OUT_0

}


void RxCommandParser::Start() {
	main_thread.start(callback(this, &RxCommandParser::main_worker));
}



void RxCommandParser::main_worker() {

	/*
	 * Here we receive throttle and steering control from the auto-pilot computer
	 */
	SocketAddress sockAddr;
	char inputBuffer[65];
	memset(inputBuffer, 0, 65);

	uint64_t _last_autopilot = 0;

	uint16_t *control = (uint16_t *) &(inputBuffer[0]);

	_rx_sock->set_blocking(true);
	_rx_sock->set_timeout(500);

	while (true) {

		int n = _rx_sock->recvfrom(&sockAddr, inputBuffer, 64);
		inputBuffer[n] = 0;

		uint64_t ts = rtos::Kernel::get_ms_count();
		if (ts - _last_autopilot > 500) {
			if (auto_ch1 != 1024 || auto_ch2 != 1024) {
				u_printf("Timeout: resetting auto sbus values\n");
				auto_ch1 = 1024;
				auto_ch2 = 1024;
			}
		}

		if (n == 2*sizeof(uint16_t)) {
			_last_autopilot = ts;
			auto_ch1 = control[0];
			auto_ch2 = control[1];
		} else if (n > 4) {
			if (strncmp(inputBuffer, "set ", 4) == 0) {
				if (n > 9 && strncmp(&(inputBuffer[4]), "relay", 5) == 0) {
#ifdef USER_DIGITAL_OUT_0
					// ascii "1" is 49
					if (inputBuffer[9] == 49) {
						*user_dout_0 = 1;
					} else {
						*user_dout_0 = 0;
					}
#endif // USER_DIGITAL_OUT_0
				} else if (n > 8 && strncmp(&(inputBuffer[4]), "PWM", 3) == 0) {
					u_printf(inputBuffer);
					switch (inputBuffer[7]) {
					case '0':
#ifdef USER_PWM_OUT_0
						float val = (float) atoi(&(inputBuffer[8])) / 255.0;
						if (val < 0.0) {
							val = 0.0;
						} else if (val > 1.0) {
							val = 1.0;
						}
						user_pout_0->write(val);
						u_printf("--- set PWM0: %f\n", val);
#endif // USER_PWM_OUT_0
						break;

					}
				} else if (n >= 11 && strncmp(&(inputBuffer[4]), "servo", 3) == 0) {
					u_printf(inputBuffer);
					switch (inputBuffer[9]) {
					case '0':
#ifdef USER_SERVO_OUT_0
						int val = atoi(&(inputBuffer[10]));
						// Futaba servo PWs are usually about 300 to 1700 us, but we 
						// will allow anything that is physically possible
						if (val < 0) {
							val = 0;
						} else if (val > 15000) {
							val = 15000;
						}
						user_servo_0->pulsewidth_us(val);
						u_printf("--- set servo0: %d\n", val);
#endif // USER_SERVO_OUT_0

						break;
					}
				}
			}
		} else if (n > 0) {
			inputBuffer[n] = 0;
			printf("rx %d bytes:\r\n", n);
			printf(inputBuffer);
			printf("\r\n");
		} else {
			//printf("empty packet\r\n");
		}
	}
}


