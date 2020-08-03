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

//#include "daemons/Radio169_daemon.hpp"
#include "daemons/SBus_daemon.hpp"
#include "daemons/IMU_daemon.hpp"
#include "daemons/GPS_daemon.hpp"
//#include "daemons/RTCM3_daemon.hpp"
#include "daemons/PushButton_daemon.hpp"

#include "MotorControl.hpp"
#include "RxCommandParser.hpp"

EventFlags event_flags;

bool NETWORK_IS_UP = false;
 
// Network interface
EthernetInterface net;
UDPSocket tx_sock; // tx will be completely non-blocking

// Heartbeat LED:
PwmOut hb_led(PA_6);

// Flight-mode LEDs:
DigitalOut myledR(LED3, 0);
DigitalOut myledG(LED1, 0);
DigitalOut myledB(LED2, 0);



// *** NOTE: By default, mbed-os only allows for 4 sockets, so
// *** we will re-use the tx_sock (non-blocking) wherever possible.


// Background I/O processes:
//  (minimal inter-dependence; mostly independent of anything else)
SBus_daemon sbus_daemon(PD_2, &tx_sock);
//Radio169_daemon r169_daemon(NC, PD_6, &tx_sock);
IMU_daemon imu_daemon(&tx_sock);
GPS_daemon gps_daemon(PE_8, PE_7, &net);
//RTCM3_daemon rtcm3_daemon(PD_5, PD_6, &tx_sock);
PushButton_daemon pushButton_daemon(PE_9, &tx_sock);

RxCommandParser rxParser(&net);

// Motors:
MotorControl motorControl(PD_14, PD_15);


void u_printf(const char *fmt, ...) {
	va_list args;
	char buffer[1500];
	int bufLen;

	va_start(args, fmt);
	bufLen = vsnprintf(buffer, 1499, fmt, args);
	va_end(args);

	int retval = tx_sock.sendto(_BROADCAST_IP_ADDRESS, UDP_PORT_DEBUG, buffer, bufLen);

	if (retval < 0 && NETWORK_IS_UP) {
		printf("sock.err in u_printf(): %d\r\n", retval);
		return;
	}
}



enum Moab_State_t {
	NoSignal = 0,
	Stop = 1,
	Manual = 2,
	Auto = 3
} moab_state = Stop;


void set_mode_sbus_failsafe() {
	// Radio contact with the transmitter has timed-out, so stop the motors
	moab_state = NoSignal;

	myledR = 0;
	myledG = 0;
	myledB = 0;

	motorControl.set_steering(1024);
	motorControl.set_throttle(352);

	// Convient info for LocationServices, on the host PC:
	imu_daemon.set_extra_info(1024, 352, moab_state);
}

void set_mode_stop() {
	moab_state = Stop;

	myledR = 1;
	myledG = 0;
	myledB = 0;

	motorControl.set_steering(sbus_daemon.sbup.ch1);
	motorControl.set_throttle(352);

	// Convient info for LocationServices, on the host PC:
	imu_daemon.set_extra_info(1024, 352, moab_state);
}

void set_mode_manual() {
	moab_state = Manual;

	myledR = 0;
	myledG = 1;
	myledB = 0;

	motorControl.set_steering(sbus_daemon.sbup.ch1);
	motorControl.set_throttle(sbus_daemon.sbup.ch3);

	// Convient info for LocationServices, on the host PC:
	imu_daemon.set_extra_info(sbus_daemon.sbup.ch1, sbus_daemon.sbup.ch3, moab_state);
}

void set_mode_auto() {
	moab_state = Auto;

	myledR = 0;
	myledG = 0;
	myledB = 1;

	motorControl.set_steering(rxParser.auto_ch1);
	motorControl.set_throttle(rxParser.auto_ch2);

	// Convient info for LocationServices, on the host PC:
	imu_daemon.set_extra_info(rxParser.auto_ch1, rxParser.auto_ch2, moab_state);
}



void radio_callback(bool force_stop) {
	if (force_stop) {
		// radio disconnected or something...
		u_printf("S.Bus timeout!\n");
		set_mode_sbus_failsafe();
	} else if (sbus_daemon.sbup.failsafe) {
		u_printf("S.Bus failsafe!\n");
		set_mode_sbus_failsafe();
	} else if (sbus_daemon.sbup.ch5 < 688) {
		set_mode_stop();
	} else if (sbus_daemon.sbup.ch5 < 1360) {
		set_mode_manual();
	} else {
		set_mode_auto();
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
			printf("Global IP address set.  ");
			NETWORK_IS_UP = true;
			ip = net.get_ip_address();  // <--dhcp
			if (ip) {
				printf("IP address is: %s\r\n", ip);
			} else {
				printf("no IP address... we're screwed\r\n");
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
			printf("Not supported\r\n");
			break;
	}
}

 
int main() {
	printf("\r\n");
	printf("   ##### This is AT-Drive Moab #####\r\n");

	//  ######################################
	//  #########################################
	//  ###########################################
	//   BEGIN:  setup network and udp socket
	//  ############################################

	printf("Starting the network...\r\n");

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
	tx_sock.open(&net);
	tx_sock.bind(12347);
	tx_sock.set_blocking(false);

	rxParser.Start();  // will start a separate thread

	// Background threads
	sbus_daemon.attachCallback(&radio_callback);
	sbus_daemon.Start();  // will start a separate thread
	//r169_daemon.attachCallback(&radio_callback);
	//r169_daemon.Start();  // will start a separate thread


	imu_daemon.Start();  // will start a separate thread
	gps_daemon.Start();  // will start a separate thread
	//rtcm3_daemon.Start();  // will start a separate thread
	pushButton_daemon.Start();  // will start a separate thread


	hb_led.period(0.02);
	hb_led.write(0.0);

	for (int ct=0; true; ++ct){

		// Heartbeat LED glows brighter:
		for (int i=0; i < 11; ++i) {
				float brightness = i/10.0;
				hb_led.write(brightness);
				ThisThread::sleep_for(20);
		}
		// Heartbeat LED dims darker:
		for (int i=0; i < 11; ++i) {
				float brightness = 1.0 - i/10.0;
				hb_led.write(brightness);
				ThisThread::sleep_for(20);
		}

		u_printf("heartbeat: %d\n", ct);

		// Report motor values (for convience when setting trim)
		uint16_t sbus_a = motorControl.get_value_a();
		float pw_a = motorControl.get_pw_a();
		u_printf("steering: %d %f\n", sbus_a, pw_a);

		uint16_t sbus_b = motorControl.get_value_b();
		float pw_b = motorControl.get_pw_b();
		u_printf("throttle: %d %f\n", sbus_b, pw_b);


	}

	// Close the socket and bring down the network interface
	tx_sock.close();
	net.disconnect();
	return 0;
}

