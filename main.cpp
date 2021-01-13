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

#include "daemons/Radio169_daemon.hpp"
#include "daemons/SBus_daemon.hpp"
#include "daemons/IMU_daemon.hpp"
#include "daemons/GPS_daemon.hpp"
//#include "daemons/RTCM3_daemon.hpp"
#include "daemons/PushButton_daemon.hpp"

#include "SbusParser.hpp"
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

#ifdef _USE_RADIO169
Radio169_daemon r169_daemon(NC, PD_6, &tx_sock);
#endif // _USE_RADIO169

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



enum Moab_State_t moab_state = Stop;


void radio_callback() {

	uint16_t sb_steering = 1024;
	uint16_t sb_throttle = 1024;

	enum Rc_controller_source_t {
		RC_NONE,
		RC_SBUS,
		RC_R169
	} rc_radio_source = RC_NONE;

	// If both sbus and r169 have timeout, then set state to no_signal
	// if sbus has timeout, but r169 has signal, then use r169
	// if sbus has signal, then use sbus

	// From S.Bus, there are 3 modes:
	//   Stop
	//   Manual
	//   Auto

	// From r169, there are 4 modes:
	//   Stop
	//   Manual
	//   Auto
	//   Stop_no_brakes

	// If mode is "Auto", but there's a timeout from the autopilot,
	//  then the mode is Auto_no_autopilot (same as "Stop")

	if (sbus_daemon.timeout == false) {  // Trust the S.Bus

		rc_radio_source = RC_SBUS;
		moab_state = sbus_daemon.requested_moab_state;
		sb_steering = sbus_daemon.sbup.ch1;
		sb_throttle = sbus_daemon.sbup.ch3;


#ifdef _USE_RADIO169
	} else if (r169_daemon.timeout == false) {  // Trust the r169

		rc_radio_source = RC_R169;
		moab_state = r169_daemon.requested_moab_state;
		sb_steering = r169_daemon.sb_steering;
		sb_throttle = r169_daemon.sb_throttle;
#endif // _USE_RADIO169

	} else {

		rc_radio_source = RC_NONE;
		moab_state = NoSignal;
	}

	if (moab_state == Auto && rxParser.timeout) {
		moab_state = Auto_no_autopilot;
	}


	switch (moab_state) {
	case NoSignal:
		myledR = 0;
		myledG = 0;
		myledB = 0;

		sb_steering = 1024;
		sb_throttle = 352;
		break;

	case Stop: // with brakes
		myledR = 1;
		myledG = 0;
		myledB = 0;

		sb_throttle = 352;
		break;

	case Manual:
		myledR = 0;
		myledG = 1;
		myledB = 0;

		break;

	case Auto:
		myledR = 0;
		myledG = 0;
		myledB = 1;

		sb_steering = rxParser.auto_ch1;
		sb_throttle = rxParser.auto_ch2;
		break;

	case Stop_no_brakes:
		myledR = 1;
		myledG = 0;
		myledB = 0;

		sb_throttle = 1024;
		break;

	case Auto_no_autopilot:
		myledR = 0;
		myledG = 0;
		myledB = 1;

		sb_steering = 1024;
		sb_throttle = 1024;
		break;
	}

	motorControl.set_steering_and_throttle(sb_steering, sb_throttle);

	// Convient info for LocationServices, on the host PC:
	imu_daemon.set_extra_info(sb_steering, sb_throttle, moab_state, rc_radio_source);


#ifdef USER_DIGITAL_OUT_0
	if (moab_state == Manual) {
		if (rc_radio_source == RC_SBUS) {
			if (sbus_daemon.sbup.ch6 < 688) {
				rxParser.setRelay(1);
			} else {
				rxParser.setRelay(0);
			}

#ifdef _USE_RADIO169
		} else if (rc_radio_source == RC_R169) {
			if (r169_daemon.controller_values.logitech) {
				rxParser.setRelay(1);
			} else {
				rxParser.setRelay(0);
			}
#endif // _USE_RADIO169
		}
	}
#endif // USER_DIGITAL_OUT_0

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

	moab_state = Stop;

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

#ifdef _USE_RADIO169
	r169_daemon.attachCallback(&radio_callback);
	r169_daemon.Start();  // will start a separate thread
#endif // _USE_RADIO169

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
		uint16_t sbus_a = motorControl.get_steer_value();
		float pw_a = motorControl.get_pw_a();
		u_printf("steering: %d %f\n", sbus_a, pw_a);

		uint16_t sbus_b = motorControl.get_throt_value();
		float pw_b = motorControl.get_pw_b();
		u_printf("throttle: %d %f\n", sbus_b, pw_b);

	}

	// Close the socket and bring down the network interface
	tx_sock.close();
	net.disconnect();
	return 0;
}

