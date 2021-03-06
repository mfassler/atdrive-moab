
#include "IMU_daemon.hpp"


extern void u_printf(const char *fmt, ...);  // Defined in main()



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


#ifdef _TWO_SHAFT_ENCODERS
ShaftEncoder shaft_a(PF_14);
ShaftEncoder shaft_b(PE_13);
#else // _TWO_SHAFT_ENCODERS
ShaftEncoder shaft(PE_11);
#endif // _TWO_SHAFT_ENCODERS




IMU_daemon::IMU_daemon(UDPSocket *tx_sock) {
	memset(&_mData, 0, sizeof(_mData));
	_mData.version = 1;

	_sock = tx_sock;

	_destSockAddr.set_ip_address(_BROADCAST_IP_ADDRESS);
	_destSockAddr.set_port(UDP_PORT_IMU);

	adc0 = new AnalogIn(PA_3);
}


void IMU_daemon::Start() {
	main_thread.start(callback(this, &IMU_daemon::main_worker));
}


void IMU_daemon::set_extra_info(
		uint16_t sbus_steering,
		uint16_t sbus_throttle,
		uint8_t moab_mode,
		uint8_t rc_radio_source) {

	// This is just convenient info to include here...  not really IMU-related per se...

	_mData.sbus_a = sbus_steering;
	_mData.sbus_b = sbus_throttle;
	_mData.moab_mode = moab_mode;
	_mData.rc_radio_source = rc_radio_source;
}


void IMU_daemon::main_worker() {
	ThisThread::sleep_for(1000ms);

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



	int count = 0;
	while (true) {
		ThisThread::sleep_for(10ms); // 100Hz

		// Check the external compass at 20 Hz:
		if (count % 5 == 0) {
			// First we zero-out the numbers, to detect if the compass has been disconnected
			_mData.compass_XYZ[0] = 0;
			_mData.compass_XYZ[1] = 0;
			_mData.compass_XYZ[2] = 0;

			int retval = compass.get_data(_mData.compass_XYZ);

			// This could produce a lot of messages...
			//if (retval != 6) {
			//	printf("failed to get data from external compass\r\n");
			//}
		}

		// Check the ADC at 5 Hz:
		if (count % 20 == 0) {
			_mData.adc0 = adc0->read_u16();
		}

		// Check IMU at 50 Hz:
		if (count % 2 == 0) {
			int retval = bno1.get_data(_mData.bnoData);

			if (retval == 22) {

#ifdef _TWO_SHAFT_ENCODERS
				_mData.shaft_a_pps = shaft_a.get_pps();
				_mData.shaft_b_pps = shaft_b.get_pps();
#else // _TWO_SHAFT_ENCODERS
				_mData.shaft_pps = shaft.get_pps();
#endif // _TWO_SHAFT_ENCODERS

				int retval2 = _sock->sendto(_destSockAddr, (char*) &_mData, sizeof(_mData));
			} else {
				// This could produce a lot of messages...
				//printf("failed to get data from external compass\r\n");
			}
		}

		// Check temp and pressure at 2 Hz:
		if (count % 50 == 0) {
			bmp1.get_data();
			_mData.temperature = bmp1._temp;
			_mData.pressure = bmp1._press;

			//u_printf("bmp._temp: %f\n", bmp1._temp);
			//u_printf("bmp._press: %f\n", bmp1._press);
		}


		count++;
		if (count > 500) {
			count = 0;

			// Every 5 seconds or so, we occasionally re-init the
			// external compass (just in case it gets disconnected)

			compass.init();
		}
	}
}

