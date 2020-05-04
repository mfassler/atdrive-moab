/*
 * atdrive-moab
 *
 * Firmware (using mbed-os) for the Motor and I/O Control Board
 *    (called  AT-Drive, Moab)
 *
 * Copyright 2019 Mark Fassler
 * Licensed under the GPLv3
 *
 */


#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"

#include "ROBOT_CONFIG.hpp"
#include "EVENT_FLAGS.hpp"
#include "MOAB_DEFINITIONS.h"

#include "drivers/IMU_module.hpp"

#include "SbusParser.hpp"
//#include "UbloxParser.hpp"
#include "MotorControl.hpp"


EventFlags event_flags;

bool NETWORK_IS_UP = false;
 
// Network interface
EthernetInterface net;
UDPSocket rx_sock; // one, single thread for RX
UDPSocket tx_sock; // tx will be completely non-blocking

Thread udp_rx_thread;
Thread sbus_reTx_thread;
Thread gps_reTx_thread;
Thread gp_interrupt_messages_thread;

// Heartbeat LED:
PwmOut hb_led(PA_6);

// Flight-mode LEDs:
DigitalOut myledR(LED3, 0);
DigitalOut myledG(LED1, 0);
DigitalOut myledB(LED2, 0);


// IMU background process:
IMU_module imu_module(&net);

// Motors:
MotorControl motorControl(PD_14, PD_15);




// S.Bus is 100000Hz, 8E2, electrically inverted
RawSerial sbus_in(NC, PD_2, 100000);  // tx, then rx
RawSerial gps_in(PE_8, PE_7, 115200);  //tx, then rx

InterruptIn pgm_switch(PE_9, PullUp);

void u_printf(const char *fmt, ...) {
	va_list args;
	char buffer[1500];
	int bufLen;

	va_start(args, fmt);
	bufLen = vsnprintf(buffer, 1499, fmt, args);
	va_end(args);

	int retval = tx_sock.sendto(_BROADCAST_IP_ADDRESS, UDP_PORT_DEBUG, buffer, bufLen);

	if (retval < 0 && NETWORK_IS_UP) {
		printf("socket error in u_printf() function\n");
		return;
	}
}


uint16_t auto_ch1 = 1024;
uint16_t auto_ch2 = 1024;
void udp_rx_worker() {
	/*
	 * Here we receive throttle and steering control from the auto-pilot computer
	 */
	SocketAddress sockAddr;
	char inputBuffer[33];
	inputBuffer[32] = 0;
	uint64_t _last_autopilot = 0;

	uint16_t *control = (uint16_t *) &(inputBuffer[0]);

	rx_sock.set_blocking(true);
	rx_sock.set_timeout(500);

	while (true) {

		int n = rx_sock.recvfrom(&sockAddr, inputBuffer, 64);
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
		} else if (n > 0) {
			inputBuffer[n] = 0;
			printf("rx %d bytes\n", n);
			printf(inputBuffer);
		} else {
			//printf("empty packet\n");
		}
	}
}


struct sbus_udp_payload sbup;
SbusParser sbusParser(&sbup);
//UbloxParser ubloxParser;



void set_mode_sbus_failsafe() {
	myledR = 0;
	myledG = 0;
	myledB = 0;

	motorControl.set_steering(1024);
	motorControl.set_throttle(352);
}

void set_mode_stop() {
	myledR = 1;
	myledG = 0;
	myledB = 0;

	motorControl.set_steering(sbup.ch1);
	motorControl.set_throttle(352);
}

void set_mode_manual() {
	myledR = 0;
	myledG = 1;
	myledB = 0;

	motorControl.set_steering(sbup.ch1);
	motorControl.set_throttle(sbup.ch3);
}

void set_mode_auto() {
	myledR = 0;
	myledG = 0;
	myledB = 1;

	motorControl.set_steering(auto_ch1);
	motorControl.set_throttle(auto_ch2);
}





volatile uint64_t _last_pgm_fall = 0;
volatile uint64_t _last_pgm_rise = 0;
uint64_t _last_pgm_fall_debounce = 0;
uint64_t _last_pgm_rise_debounce = 0;
uint8_t _pgm_value_debounce = 1;

void Gpin_Interrupt_Pgm() {

	if (pgm_switch.read()) {
		_last_pgm_rise = rtos::Kernel::get_ms_count();
	} else {
		_last_pgm_fall = rtos::Kernel::get_ms_count();
	}
}


bool _pgm_notice_sent = false;
void Check_Pgm_Button() {
	// We do all this time-stamp weirdness to de-bounce the switch
	uint64_t ts_ms = rtos::Kernel::get_ms_count();

	// Switch must be stable for at least 30ms:
	if ((ts_ms - _last_pgm_fall > 30) && (ts_ms - _last_pgm_rise > 30)) {
		// current stable value:
		uint8_t val = pgm_switch.read();
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

				int retval = tx_sock.sendto(_BROADCAST_IP_ADDRESS, UDP_PORT_PUSHBUTTON,
					&ts_ms, sizeof(ts_ms));

				if (retval < 0 && NETWORK_IS_UP) {
					printf("UDP socket error in Check_Pgm_Button\n");
				}
				_pgm_notice_sent = true;
			}
		}
	}
}


void Sbus_Rx_Interrupt() {

	int c;

	while (sbus_in.readable()) {

		c = sbus_in.getc();
		int status = sbusParser.rx_char(c);

		if (status == 1) {
			event_flags.set(_EVENT_FLAG_SBUS);
		}
	}
}


// Incoming buffer, from serial port:
char _gpsRxBuf[1500];
int _gpsRxBufLen = 0;

// Outgoing buffers (circular buffer), via network UDP:
#define _GPS_RING_BUFFER_SIZE 8
char _gpsTxBuf[_GPS_RING_BUFFER_SIZE][1500];
int _gpsTxBufLen[_GPS_RING_BUFFER_SIZE] = {0, 0, 0, 0};
int _gpsTxBufIdxFI = 0;
int _gpsTxBufIdxFO = 0;

void Gps_Rx_Interrupt() {
	int c;
	while (gps_in.readable()) {

		c = gps_in.getc();
		_gpsRxBuf[_gpsRxBufLen] = c;
		_gpsRxBufLen++;

		if ((c == 0x0a) || (_gpsRxBufLen > 1400)) {

			memcpy(_gpsTxBuf[_gpsTxBufIdxFI], _gpsRxBuf, _gpsRxBufLen);
			_gpsTxBufLen[_gpsTxBufIdxFI] = _gpsRxBufLen;

			_gpsTxBufIdxFI++;
			if (_gpsTxBufIdxFI >= _GPS_RING_BUFFER_SIZE) {
				_gpsTxBufIdxFI = 0;
			}

			event_flags.set(_EVENT_FLAG_GPS);
			_gpsRxBufLen = 0;
		}
	}
}


void gps_reTx_worker() {

	uint32_t flags_read;

	while (true) {
		flags_read = event_flags.wait_any(_EVENT_FLAG_GPS, 1200);

		if (flags_read & osFlagsError) {

			u_printf("GPS timeout!\n");

		} else {

			while (_gpsTxBufIdxFO != _gpsTxBufIdxFI) {
				int retval = tx_sock.sendto(_BROADCAST_IP_ADDRESS, UDP_PORT_GPS_NMEA,
						_gpsTxBuf[_gpsTxBufIdxFO], _gpsTxBufLen[_gpsTxBufIdxFO]);

				_gpsTxBufIdxFO++;
				if (_gpsTxBufIdxFO >= _GPS_RING_BUFFER_SIZE) {
					_gpsTxBufIdxFO = 0;
				}

				if (retval < 0 && NETWORK_IS_UP) {
					printf("UDP socket error in gps_reTx_worker\n");
				}
			}
		}
	}
}


void sbus_reTx_worker() {

	uint32_t flags_read;

	while (true) {
		flags_read = event_flags.wait_any(_EVENT_FLAG_SBUS, 100);

		if (flags_read & osFlagsError) {
			u_printf("S.Bus timeout!\n");
			sbup.failsafe = true;
			set_mode_sbus_failsafe();
		} else if (sbup.failsafe) {
			u_printf("S.Bus failsafe!\n");
			set_mode_sbus_failsafe();
		} else {
			if (sbup.ch5 < 688) {
				set_mode_stop();
			} else if (sbup.ch5 < 1360) {
				set_mode_manual();
			} else {
				set_mode_auto();
			}

		}
		int retval = tx_sock.sendto(_AUTOPILOT_IP_ADDRESS, UDP_PORT_SBUS,
				(char *) &sbup, sizeof(struct sbus_udp_payload));

		if (retval < 0 && NETWORK_IS_UP) {
			printf("UDP socket error in sbus_reTx_worker\n");
		}
	}
}




void eth_callback(nsapi_event_t status, intptr_t param) {
	const char *ip;

	printf("Connection status changed!\r\n");
	switch(param) {
		case NSAPI_STATUS_LOCAL_UP:
			NETWORK_IS_UP = false;
			printf("Local IP address set.\r\n");
			break;
		case NSAPI_STATUS_GLOBAL_UP:
			printf("Global IP address set.\r\n");
			NETWORK_IS_UP = true;
			ip = net.get_ip_address();  // <--dhcp
			if (ip) {
				printf("IP address is: %s\n", ip);
			} else {
				printf("no IP address... we're screwed\n");
				//return -1;
			}
			break;
		case NSAPI_STATUS_DISCONNECTED:
			NETWORK_IS_UP = false;
			printf("No connection to network.\r\n");
			break;
		case NSAPI_STATUS_CONNECTING:
			NETWORK_IS_UP = false;
			printf("Connecting to network...\r\n");
			break;
		default:
			NETWORK_IS_UP = false;
			printf("Not supported");
			break;
	}
}

 
int main() {

	//  ######################################
	//  #########################################
	//  ###########################################
	//   BEGIN:  setup network and udp socket
	//  ############################################

	printf("\n\nStarting the network...\n");

	net.attach(&eth_callback);
	net.set_dhcp(false);
	net.set_network(_MOAB_IP_ADDRESS, _NETMASK, _DEFUALT_GATEWAY);
	net.set_blocking(false);

	net.connect();

	//  ############################################
	//   END:  setup network and udp socket
	//  ###########################################
	//  #########################################
	//  ######################################


	// UDP Sockets
	rx_sock.open(&net);
	rx_sock.bind(12346);

	tx_sock.open(&net);
	tx_sock.bind(12347);
	tx_sock.set_blocking(false);


	// Serial ports
	sbus_in.format(8, SerialBase::Even, 2);  // S.Bus is 8E2
	sbus_in.attach(&Sbus_Rx_Interrupt);

	gps_in.attach(&Gps_Rx_Interrupt);


	// Background threads
	udp_rx_thread.start(udp_rx_worker);
	sbus_reTx_thread.start(sbus_reTx_worker);
	gps_reTx_thread.start(gps_reTx_worker);

	imu_module.Start();  // will start a separate thread

	pgm_switch.rise(&Gpin_Interrupt_Pgm);
	pgm_switch.fall(&Gpin_Interrupt_Pgm);

	hb_led.period(0.02);
	hb_led.write(0.0);


	for (int ct=0; true; ++ct){

		for (int i=0; i < 11; ++i) {
				float brightness = i/10.0;
				hb_led.write(brightness);
				ThisThread::sleep_for(20);
				Check_Pgm_Button();
		}
		for (int i=0; i < 11; ++i) {
				float brightness = 1.0 - i/10.0;
				hb_led.write(brightness);
				ThisThread::sleep_for(20);
				Check_Pgm_Button();
		}

		u_printf("heeartbeatZ: %d\n", ct);

		// Report motor values (for convience when setting trim)
		uint16_t sbus_a = motorControl.get_value_a();
		float pw_a = motorControl.get_pw_a();
		u_printf("steering: %d %f\n", sbus_a, pw_a);

		uint16_t sbus_b = motorControl.get_value_b();
		float pw_b = motorControl.get_pw_b();
		u_printf("throttle: %d %f\n", sbus_b, pw_b);


	}

   
	// Close the socket and bring down the network interface
	rx_sock.close();
	tx_sock.close();
	net.disconnect();
	return 0;
}

