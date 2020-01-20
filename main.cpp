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


//#include "ROBOT_CONFIG_XDRIVE_OfficeLAN.hpp"
#include "ROBOT_CONFIG_XDRIVE_GLiNet.hpp"
//#include "ROBOT_CONFIG_XDRIVE_OfficeLAN.hpp"
#include "EVENT_FLAGS.hpp"


#include "SbusParser.hpp"
//#include "UbloxParser.hpp"
//#include "MotorControl.hpp"
#include "XWheels.hpp"
#include "ShaftEncoder.hpp"


#include "drivers/ST_LIS3MDL.hpp"
#include "drivers/BMP280.hpp"
#include "drivers/BNO055.hpp"

EventFlags event_flags;

uint16_t debug_port = 31337;
uint16_t sbus_port = 31338;
uint16_t button_port = 31345;

//uint16_t gps_port = 27110; // ublox
//uint16_t odometry_port = 27112;
uint16_t gps_port_nmea = 27113; // NMEA
uint16_t imu_port = 27114;
uint16_t imu_config_port = 27115;


bool NETWORK_IS_UP = false;
 
// Network interface
EthernetInterface net;
UDPSocket rx_sock; // one, single thread for RX
UDPSocket tx_sock; // tx will be completely non-blocking

Thread udp_rx_thread;
Thread sbus_reTx_thread;
Thread gps_reTx_thread;
Thread imu_thread;
Thread gp_interrupt_messages_thread;

// Heartbeat LED:
PwmOut hb_led(PA_6);

// Flight-mode LEDs:
DigitalOut myledR(LED3, 0);
DigitalOut myledG(LED1, 0);
DigitalOut myledB(LED2, 0);

// Motors:
//MotorControl motorControl(PD_14, PD_15);
RawSerial wheelUART(PD_1,PD_0,9600);
XWheels drive(&wheelUART);      // use XWheels class
float motorRPM[2];
ShaftEncoder shaft(PE_11);

/*****************************************
 * I2C Bus for external GPS module
 *****************************************/
//   on schematic it's called:  MAG_SDA and MAG_SCL
I2C mag_i2c(PB_11, PB_10);  // sda, then scl
ST_LIS3MDL compass(&mag_i2c);



/*****************************************
 * I2C Bus for on-board IMU and barometer
 *****************************************/
//   on schematic it's called:  BNO_SDA and BNO_SCL
//  ports are: PD_13 (sda) and PD_12 (scl)
I2C bno_i2c(PD_13, PD_12);  // sda, then scl
BMP280 bmp1(&bno_i2c);
BNO055 bno1(&bno_i2c);


// S.Bus is 100000Hz, 8E2, electrically inverted
RawSerial sbus_in(NC, PD_2, 100000);  // tx, then rx
RawSerial gps_in(PE_8, PE_7, 115200);  //tx, then rx
Serial pc(USBTX,USBRX,115200);                              // for print out something to PC
InterruptIn pgm_switch(PE_9, PullUp);

void u_printf(const char *fmt, ...) {
	va_list args;
	char buffer[1500];
	int bufLen;

	va_start(args, fmt);
	bufLen = vsnprintf(buffer, 1499, fmt, args);
	va_end(args);

	int retval = tx_sock.sendto(_BROADCAST_IP_ADDRESS, debug_port, buffer, bufLen);

	if (retval < 0 && NETWORK_IS_UP) {
		printf("socket error in u_printf() function\n");
		return;
	}
}


enum IMU_MODE {
	normal = 0,
	config_read = 1,
	config_write = 2,
} imu_mode;

char _imu_config[22];

uint16_t auto_ch1 = 1024;
uint16_t auto_ch2 = 1024;
float rpmR; // for X wheels used
float rpmL; // for X wheels used
uint16_t Raw_rpmR; // for X wheels used
uint16_t Raw_rpmL; // for X wheels used

void udp_rx_worker() {
    
	 // Here we receive throttle and steering control from the auto-pilot computer
	 
	SocketAddress sockAddr;
	char inputBuffer[33];
	inputBuffer[32] = 0;
	uint64_t _last_autopilot = 0;

	float *control = (float *) &(inputBuffer[0]);

	rx_sock.set_blocking(true);
	rx_sock.set_timeout(500);

	while (true) {

		int n = rx_sock.recvfrom(&sockAddr, inputBuffer, 64);
		uint64_t ts = rtos::Kernel::get_ms_count();
		if (ts - _last_autopilot > 500) {
			if (auto_ch1 != 1024 || auto_ch2 != 1024) {
				u_printf("Timeout: resetting auto sbus values\n");
				//auto_ch1 = 1024;
				//auto_ch2 = 1024;
                Raw_rpmR = 0;
                Raw_rpmL = 0;
			}
		}

		if (n == 2*sizeof(float)) {
			_last_autopilot = ts;
			//auto_ch1 = control[0];
			//auto_ch2 = control[1];
            rpmR = control[0];
            rpmL = control[1];
		} else if (n >= 7) {
			if (strncmp(inputBuffer, "moabCRI", 7) == 0) {
				u_printf("  ** config read imu\n");
				//imu_mode = config_read;

			} else if (strncmp(inputBuffer, "moabCWI", 7) == 0) {
				if (n == 30) {
					u_printf("  ** config write imu\n");
					memcpy(_imu_config, &(inputBuffer[8]), 22);
					//imu_mode = config_write;
				} else {
					u_printf("bad config write command: %d bytes \n", n);
				}

			} else {
				u_printf("unknown command\n");
			}
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

	//motorControl.set_steering(1024);
	//motorControl.set_throttle(352);
    drive.DriveWheels(0.0, 0.0);
}

void set_mode_stop() {
	myledR = 1;
	myledG = 0;
	myledB = 0;

	//motorControl.set_steering(1024);
	//motorControl.set_throttle(352);
    drive.DriveWheels(0.0, 0.0);
}

void set_mode_manual() {
	myledR = 0;
	myledG = 1;
	myledB = 0;

	//motorControl.set_steering(sbup.ch1);
	//motorControl.set_throttle(sbup.ch3);
    drive.vehicleControl(sbup.ch2, sbup.ch4, motorRPM);
    //pc.printf("ch2 %d\n", sbup.ch2);
    //pc.printf("ch4 %d\n", sbup.ch4);
    drive.DriveWheels(motorRPM[0],motorRPM[1]);
}

void set_mode_auto() {
	myledR = 0;
	myledG = 0;
	myledB = 1;

	//motorControl.set_steering(auto_ch1);
	//motorControl.set_throttle(auto_ch2);
	printf("rpmR: %f\n", rpmR);
	printf("rpmL: %f\n", rpmL);
    drive.DriveWheels(rpmR,rpmL);
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

				int retval = tx_sock.sendto(_BROADCAST_IP_ADDRESS, button_port,
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
int _gpsRxBufIdx = 0;

// Outgoing buffer, via network UDP:
char _gpsTxBuf[1500];
int gpsMessageLen;

void Gps_Rx_Interrupt() {
	int c;
	while (gps_in.readable()) {

		c = gps_in.getc();
		_gpsRxBuf[_gpsRxBufIdx] = c;
		_gpsRxBufIdx++;

		if ((c == 0x0a) || (_gpsRxBufIdx > 1400)) {
			memcpy(_gpsTxBuf, _gpsRxBuf, _gpsRxBufIdx);
			gpsMessageLen = _gpsRxBufIdx;
			event_flags.set(_EVENT_FLAG_GPS);
			_gpsRxBufIdx = 0;
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

			int retval = tx_sock.sendto(_BROADCAST_IP_ADDRESS, gps_port_nmea,
					_gpsTxBuf, gpsMessageLen);

			if (retval < 0 && NETWORK_IS_UP) {
				printf("UDP socket error in gps_reTx_worker\n");
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
            
			int retval = tx_sock.sendto(_AUTOPILOT_IP_ADDRESS, sbus_port,
					(char *) &sbup, sizeof(struct sbus_udp_payload));

			if (retval < 0 && NETWORK_IS_UP) {
				printf("UDP socket error in sbus_reTx_worker\n");
			}
            
		}
	}
}


void imu_worker() {

	struct multi_data {

		// 64 bits:
		int16_t compass_XYZ[3];  // external compass
		int16_t _padding1;  // the compiler seems to like 64-bit boundaries

		// 3 * 64 bits:
		char bnoData[22];  // internal IMU
		int16_t _padding2;  // the compiler seems to like 64-bit boundaries

		// 64 bits:
		float temperature; // degrees celsius, no need for high accuracy

		// Pressure:  typical sensor value is ~100000, with accuracy of +/- 12.0,
		// (don't forget to convert between Pa and hPa), so this is well
		// within the accuracy of float32
		float pressure;

		// 64 bits:
		uint16_t sbus_a;
		uint16_t sbus_b;
		uint16_t _padding3;  // 64-bit boundary
		uint16_t _padding4;  // 64-bit boundary

		// 64 bits:
		// TODO:  do we really need float64 for these numbers?
		double shaft_pps;

	} mData;

	memset(&mData, 0, sizeof(mData));

	int count = 0;
	while (true) {
		if (imu_mode == config_read) {
			u_printf(" --------------- inside imu worker, config read\n");
			char bno_config[22];
			memset(bno_config, 0, 22);

			int retval = bno1.get_config(bno_config);
			if (retval == 22) {
				int retval2 = tx_sock.sendto(_BROADCAST_IP_ADDRESS, imu_config_port,
					bno_config, 22);
			} else {
				u_printf("Failed to read imu config\n");
			}
			imu_mode = normal;
		}

		if (imu_mode == config_write) {
			u_printf(" --------------- inside imu worker, config write\n");

			int retval = bno1.write_config(_imu_config);
			if (retval == 22) {
				u_printf("Wrote new IMU config\n");
			} else {
				u_printf("Failed to write imu config\n");
			}
			imu_mode = normal;
		}


		wait_ms(10); // 100Hz   // wait_us has a timer problem with X Wheels class

		// Check the external compass at 20 Hz:
		if (count % 5 == 0) {
			// First we zero-out the numbers, to detect if the compass has been disconnected
			mData.compass_XYZ[0] = 0;
			mData.compass_XYZ[1] = 0;
			mData.compass_XYZ[2] = 0;

			int retval = compass.get_data(mData.compass_XYZ);

			// This could produce a lot of messages...
			//if (retval != 6) {
			//	printf("failed to get data from external compass\n");
			//}
		}


		// Check IMU at 50 Hz:
		if (count % 2 == 0) {
			int retval = bno1.get_data(mData.bnoData);

			if (retval == 22) {
				//mData.sbus_a = motorControl.get_value_a();
				//mData.sbus_b = motorControl.get_value_b();
				mData.shaft_pps = shaft.get_pps();
				int retval2 = tx_sock.sendto(_BROADCAST_IP_ADDRESS, imu_port,
					(char*) &mData, sizeof(mData));
			} else {
				// This could produce a lot of messages...
				//printf("failed to get data from external compass\n");
			}
		}

		// Check temp and pressure at 2 Hz:
		if (count % 50 == 0) {
			bmp1.get_data();
			mData.temperature = bmp1._temp;
			mData.pressure = bmp1._press;

			u_printf("bmp._temp: %f\n", bmp1._temp);
			u_printf("bmp._press: %f\n", bmp1._press);
		}


		count++;
		if (count > 500) {
			count = 0;

			// Every 5 seconds or so, we occasionally re-init the
			// external GPS and compass (just in case they get disconnected)

			compass.init();

			// UBLOX config command to enable 5 updates per second:
			const char UBX_CFG_RATE[] = {
					0xb5, 0x62, 0x06, 0x08, 0x06, 0x00,
					0x28, 0x00, 0x05, 0x00, 0x00, 0x00, 0x41, 0xb8};

			for (int i=0; i<14; i++) {
				gps_in.putc(UBX_CFG_RATE[i]);
			}
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

    // X Drive Initialize ///
    int initOK;
    initOK = drive.Init();
    if(initOK == 1)
    {
        pc.printf("Initialized OK!!!\n");
    }

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

    // Serial ports
	sbus_in.format(8, SerialBase::Even, 2);  // S.Bus is 8E2
	sbus_in.attach(&Sbus_Rx_Interrupt);
	gps_in.attach(&Gps_Rx_Interrupt);

	// UDP Sockets
	rx_sock.open(&net);
	rx_sock.bind(12346);

	tx_sock.open(&net);
	tx_sock.bind(12347);
	tx_sock.set_blocking(false);
    

	// Background threads
	udp_rx_thread.start(udp_rx_worker);
	sbus_reTx_thread.start(sbus_reTx_worker);
	gps_reTx_thread.start(gps_reTx_worker);
	imu_thread.start(imu_worker);


	pgm_switch.rise(&Gpin_Interrupt_Pgm);
	pgm_switch.fall(&Gpin_Interrupt_Pgm);

	hb_led.period(0.02);
	hb_led.write(0.0);

    
	// Look for the compass:
	if (compass.init() < 0) {
		u_printf("Failed to initialize compass\n");
	}

	// BMP280 barometer:
	if (bmp1.init() < 0) {
		u_printf("Failed to initialize barometer\n");
	};

	// BNO055 IMU:
	if (bno1.init() < 0) {
		u_printf("Failed to initialize BNO055 IMU\n");
	}
    
        
	for (int ct=0; true; ++ct){

		for (int i=0; i < 11; ++i) {
				float brightness = i/10.0;
				hb_led.write(brightness);
				wait_ms(20);     //wait_us(20000);  when use wait_us here, it has some timer problem with X Wheels class
				Check_Pgm_Button();
		}
		for (int i=0; i < 11; ++i) {
				float brightness = 1.0 - i/10.0;
				hb_led.write(brightness);
				wait_ms(20);     //wait_us(20000);  when use wait_us here, it has some timer problem with X Wheels class
				Check_Pgm_Button();
		}
        
		//u_printf("heeartbeatZ: %d\n", ct);
        /*
		// Report motor values (for convience when setting trim)
		uint16_t sbus_a = motorControl.get_value_a();
		float pw_a = motorControl.get_pw_a();
		u_printf("steering: %d %f\n", sbus_a, pw_a);

		uint16_t sbus_b = motorControl.get_value_b();
		float pw_b = motorControl.get_pw_b();
		u_printf("throttle: %d %f\n", sbus_b, pw_b);
        */

	}
    

    
	// Close the socket and bring down the network interface
	rx_sock.close();
	tx_sock.close();
	net.disconnect();
    
	return 0;
}

