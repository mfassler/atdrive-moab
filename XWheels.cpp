/*
 * firmware driver for a particular motor drive
 *
 * Reverse-engineering and original code was done by Rasheed Kittinanthapanya
 *
 *   see here:  https://github.com/rasheeddo/BrushlessDriveWheels
 * for info on the wheels protocol
 */

#include "XWheels.hpp"
#include "misc_math.hpp"

extern void u_printf(const char *fmt, ...);  // Defined in main()



XWheels::XWheels(PinName a, PinName b) {
	_tx_pin = a;
	_rx_pin = b;

	_rpm_motor_1 = 0;
	_rpm_motor_2 = 0;
	_init_count = 0;

	_serport = new BufferedSerial(_tx_pin, _rx_pin, 9600);
}


void XWheels::Start() {

	main_thread.start(callback(this, &XWheels::main_worker));
	timeout_thread.start(callback(this, &XWheels::timeout_worker));
}


void XWheels::main_worker() {

	u_printf(" *** XWHEELS starting main_worker()\n");

	char c;
	char pktLen;
	char pkt[20];

	_send_init();
	while (true) {

		_serport->read(&c, 1);

		if (c == 0) {

			_serport->read(&c, 1);
			_serport->read(&c, 1);
			_serport->read(&c, 1);
			_serport->read(&c, 1);
			_event_flags.set(_EVENT_FLAG_XWHEELS_RX);
			_send_init();
			u_printf("zeros\n");

		} else if (c == 0x81) {

			_serport->read(&pktLen, 1);
			if (pktLen == 6) {
				_serport->read(&(pkt[0]), 1);
				_serport->read(&(pkt[1]), 1);
				_serport->read(&(pkt[2]), 1);
				_serport->read(&(pkt[3]), 1);
				_event_flags.set(_EVENT_FLAG_XWHEELS_RX);
				_do_something();

				//  If we ever want to parse packets from the ESC, we could do this:
				//_do_something(0x81, pktLen, pkt);

			} else {
				u_printf("xwh: wtf?: %x %x\n", c, pktLen);
			}

		} else if (c == 0x82) {

			_serport->read(&pktLen, 1);
			if (pktLen == 6) {
				_serport->read(&(pkt[0]), 1);
				_serport->read(&(pkt[1]), 1);
				_serport->read(&(pkt[2]), 1);
				_serport->read(&(pkt[3]), 1);
				_event_flags.set(_EVENT_FLAG_XWHEELS_RX);
				_do_something();

				//  If we ever want to parse packets from the ESC, we could do this:
				//_do_something(0x82, pktLen, pkt);

			} else {
				u_printf("xwh: wtf?: %x %x\n", c, pktLen);
			}

		} else {

			u_printf("xwh: wtf?: %x\n", c);

			// empty out the input buffer:
			while(_serport->readable()) {
				_serport->read(&c, 1);
			}

		}
	}
}


void XWheels::timeout_worker() {
	uint32_t flags_read;

	while (true) {
		flags_read = _event_flags.wait_any(_EVENT_FLAG_XWHEELS_RX, 1000);

		if (flags_read & osFlagsError) {
			u_printf(" ** XWheels timeout.  Start init again\n");

			_init_count = 0;
			_send_init();
		}
	}
}


//  If we ever want to parse packets from the ESC, we could do this:
//void XWheels::_do_something(char pktType, char pkeLen, char* pkt) { }

void XWheels::_do_something(void) {
	if (_init_count < 20) {
		_send_init();
	} else {
		_send_motor_command();
	}
}


void XWheels::setRPMs(float rpm_motor_1, float rpm_motor_2) {
	_rpm_motor_1 = rpm_motor_1;
	_rpm_motor_2 = rpm_motor_2;
}


void XWheels::set_steering_and_throttle(uint16_t sb_steering, uint16_t sb_throttle) {
	float leftWheel = 0;
	float rightWheel = 0;

	calc_sbus_to_skid_mode(sb_steering, sb_throttle, &leftWheel, &rightWheel);

	float leftRPM = 144 * leftWheel;
	float rightRPM = 144 * rightWheel;

	//u_printf("manual motor: %f %f\n", leftRPM, rightRPM);
	setRPMs(rightRPM, leftRPM);
}


void XWheels::_send_init() {

	unsigned char data_packet[17] = {
		0x01, // header1
		0x11, // header2 -- looks like it might be packet size
		0x32, // Forward acceleration
		0x00, // forward delay
		0x0a, // brake distance
		0x32, // turning acceleration,
		0x01, // turn delay
		0x00, // acctimeofstart
		0x83, // senrocker
		0x14, // undervolt1
		0x05, // undervolt2
		0x0a, // start speed
		0x01, // drive mode
		0x03, // phaseLMotor
		0x04, // phaseRMotor
		0x07, // motor config
		0x00  // init checksum
	};

	unsigned char cksum = 0;
	for (int i=0; i<16; ++i) {
		cksum += data_packet[i];
	}
	data_packet[16] = cksum;

	//u_printf("sending init\n");
	_serport->write(data_packet, 17);
	_init_count += 1;
}



inline float _linear_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


unsigned int RPM_to_data(float rpm) {
	// Convert a real RPM value to the data format expected by the device

	// There seems to be a dead zone between -70 and 70
	// At max RPM (144 RPM), the data should be about 3200

	if (rpm > 0.0 && rpm < 40.0) {

		return (unsigned int)_linear_map(rpm, 0.0, 39.0, 70, 866);

	} else if (rpm < 0.0 && rpm > -40.0) {

		return (unsigned int)_linear_map(rpm, 0.0, -39.0, 32845, 33634); 

	} else if (rpm >= 40.0) {

		return (unsigned int)_linear_map(rpm, 40.0, 144.0, 888, 3200); 

	} else if (rpm <= -40.0) {

		return (unsigned int)_linear_map(rpm, -40.0, -144.0, 33656, 35968); 
	}

	return 0;
}


void XWheels::_send_motor_command() {
	unsigned char data_packet[9] = {
		0x02,  // header1
		0x09,  // header2 -- looks like it might be packet size

		0x00,  // motor1 speed MSB
		0x00,  // motor1 speed LSB
		0x00,  // motor2 speed MSB
		0x00,  // motor2 speed LSB

		//0xb4, // mode hibyte
		0x00, // mode hibyte
		0x00, // mode lobyte

		0x00 // checksum
	};

	uint16_t motor1_int = RPM_to_data(_rpm_motor_1);
	uint16_t motor2_int = RPM_to_data(_rpm_motor_2);

	uint8_t *Motor1SpeedByte;
	uint8_t *Motor2SpeedByte;

	// Type-casting trick to turn a 16 bit into a pair of 8 bits's
	Motor1SpeedByte = (uint8_t *) &motor1_int;
	Motor2SpeedByte = (uint8_t *) &motor2_int;

	data_packet[2] = Motor1SpeedByte[1];  // little-endian
	data_packet[3] = Motor1SpeedByte[0];
	data_packet[4] = Motor2SpeedByte[1];  // little-endian
	data_packet[5] = Motor2SpeedByte[0];

	unsigned char cksum = 0;
	for (int i=0; i<8; ++i) {
		cksum += data_packet[i];
	}
	data_packet[8] = cksum;

	_serport->write(data_packet, 9);
}



