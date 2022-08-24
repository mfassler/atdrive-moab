
#include "BMP390.hpp"

extern void u_printf(const char *fmt, ...);  // Defined in main()


BMP390::BMP390(I2C *i2c) {
	_i2c = i2c;
	_addr8bit = 0x77 << 1;
}

ssize_t BMP390::init() {
	// Chip ID
	read_reg_u8(0x00, &_chipId);
	if (_chipId != 0x60) {
		return -1;
	}

	u_printf("BMP390 init() successful!\n");

	read_calibration();

	// Reset the chip:
	_send_cmd(BMP3_CMD_softreset);

	// Set oversampling:
	_set_OSR(BMP3_OVERSAMPLING_32X, BMP3_OVERSAMPLING_2X);

	// Set IIR filter
	_set_IIR_filter_coefficient(BMP3_IIR_FILTER_COEFF_3);

	// By default, the BMP390 is already at the highest
	// sample rate (200Hz), so we won't change it.

	return 0;
}


void BMP390::_send_cmd(uint8_t cmd) {
	write_reg_u8(BMP3_REGSTER_CMD, cmd);
}

void BMP390::_set_OSR(uint8_t osr_p, uint8_t osr_t) {

	// Section 4.3.18 of the datasheet, OSR register

	uint8_t val = (osr_t << 3) | osr_p;

	// OSR register:  0x1c
	write_reg_u8(BMP3_REGSTER_OSR, val);
}


void BMP390::_set_IIR_filter_coefficient(uint8_t filter_coeff) {

	// Section 4.3.21 of the datasheet, CONFIG register (used for IIR coefficent)

	write_reg_u8(BMP3_REGISTER_CONFIG, filter_coeff);
}



int BMP390::get_data() {

	// CONTROL_REGISTER = 0x1b
	// Perform one measurement in forced mode
	write_reg_u8(0x1b, 0x13);

	// Wait for measurement to complete
	uint8_t STATUS_REGISTER = 0x03;
	uint8_t both_complete = 0x60;
	uint8_t retVal;
	while (true) {
		read_reg_u8(STATUS_REGISTER, &retVal);
		if ((retVal & both_complete) == both_complete) {
			break;
		} else {
			ThisThread::sleep_for(10ms);
		}
	}

	read_raw_data();

	compensate_temperature();
	compensate_pressure();

	return 0;

}

void BMP390::read_raw_data() {
	char txBuf[2] = {0x04, 0x00};
	char rxBuf[6];

	_i2c->write(_addr8bit, txBuf, 1);
	_i2c->read(_addr8bit, rxBuf, 6);

	uncomp_press = (rxBuf[2] << 16) | (rxBuf[1] << 8) | rxBuf[0];
	uncomp_temp = (rxBuf[5] << 16) | (rxBuf[4] << 8) | rxBuf[3];

}


void BMP390::compensate_temperature() {
	//This is section 8.5 of the data sheet
	
	float partial_data1;
	float partial_data2;

 	partial_data1 = (float)(uncomp_temp - par_t1);
 	partial_data2 = (float)(partial_data1 * par_t2);

	_temp = partial_data2 + (partial_data1 * partial_data1) * par_t3;

}


void BMP390::compensate_pressure() {
	// This is section 8.6 of the data sheet
	double T = _temp;
	double T2 = pow(_temp, 2);
	double T3 = pow(_temp, 3);

	float uPress = (float) uncomp_press;

	float partial_data1;
	float partial_data2;
	float partial_data3;
	float partial_data4;
	float partial_out1;
	float partial_out2;

	partial_data1 = par_p6 * T;
	partial_data2 = par_p7 * T2;
	partial_data3 = par_p8 * T3;
	partial_out1 = par_p5 + partial_data1 + partial_data2 + partial_data3;

	partial_data1 = par_p2 * T;
	partial_data2 = par_p3 * T2;
	partial_data3 = par_p4 * T3;
	partial_out2 = uPress * (par_p1 + partial_data1 + partial_data2 + partial_data3);

	partial_data1 = uPress * uPress;
	partial_data2 = par_p9 + par_p10 * T;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + (uPress * uPress * uPress) * par_p11;

	_press = partial_out1 + partial_out2 + partial_data4;
}


void BMP390::read_calibration() {

	// Python struct types:  "<HHbhhbbHHbbhbb"

	uint16_t T1;
	uint16_t T2;
	int8_t T3;
	int16_t P1;
	int16_t P2;
	int8_t P3;
	int8_t P4;
	uint16_t P5;
	uint16_t P6;
	int8_t P7;
	int8_t P8;
	int16_t P9;
	int8_t P10;
	int8_t P11;

	// We seem to need these delays  (2 ms) in order to get accurate
	// data from these registers.  TODO:  is this normal?

	read_reg_u16(0x31, &T1);
	ThisThread::sleep_for(2ms);
	read_reg_u16(0x33, &T2);
	ThisThread::sleep_for(2ms);
	read_reg_s8(0x35, &T3);
	ThisThread::sleep_for(2ms);
	read_reg_s16(0x36, &P1);
	ThisThread::sleep_for(2ms);
	read_reg_s16(0x38, &P2);
	ThisThread::sleep_for(2ms);
	read_reg_s8(0x3a, &P3);
	ThisThread::sleep_for(2ms);
	read_reg_s8(0x3b, &P4);
	ThisThread::sleep_for(2ms);
	read_reg_u16(0x3c, &P5);
	ThisThread::sleep_for(2ms);
	read_reg_u16(0x3e, &P6);
	ThisThread::sleep_for(2ms);
	read_reg_s8(0x40, &P7);
	ThisThread::sleep_for(2ms);
	read_reg_s8(0x41, &P8);
	ThisThread::sleep_for(2ms);
	read_reg_s16(0x42, &P9);
	ThisThread::sleep_for(2ms);
	read_reg_s8(0x44, &P10);
	ThisThread::sleep_for(2ms);
	read_reg_s8(0x45, &P11);
	ThisThread::sleep_for(2ms);

	u_printf("T1: %d\n", T1);
	u_printf("T2: %d\n", T2);
	u_printf("T3: %d\n", T3);

	u_printf("P1: %d\n", P1);
	u_printf("P2: %d\n", P2);
	u_printf("P3: %d\n", P3);
	u_printf("P4: %d\n", P4);
	u_printf("P5: %d\n", P5);
	u_printf("P6: %d\n", P6);
	u_printf("P7: %d\n", P7);
	u_printf("P8: %d\n", P8);
	u_printf("P9: %d\n", P9);
	u_printf("P10: %d\n", P10);
	u_printf("P11: %d\n", P11);

	// These equations are from Section 8.4 of the datasheet:
	par_t1 = T1 / pow(2, -8.0);
	par_t2 = T2 / pow(2, 30.0);
	par_t3 = T3 / pow(2, 48.0);

	par_p1 = (P1 - pow(2, 14.0)) / pow(2, 20.0);
	par_p2 = (P2 - pow(2, 14.0)) / pow(2, 29.0);
	par_p3 = P3 / pow(2, 32.0);
	par_p4 = P4 / pow(2, 37.0);
	par_p5 = P5 / pow(2, -3.0);
	par_p6 = P6 / pow(2, 6.0);
	par_p7 = P7 / pow(2, 8.0);
	par_p8 = P8 / pow(2, 15.0);
	par_p9 = P9 / pow(2, 48.0);
	par_p10 = P10 / pow(2, 48.0);
	par_p11 = P11 / pow(2, 65.0);

	u_printf("par_t1: %f\n", par_t1);
	u_printf("par_t2: %f\n", par_t2);
	u_printf("par_t3: %f\n", par_t3);

	u_printf("par_p1: %f\n", par_p1);
	u_printf("par_p2: %f\n", par_p2);
	u_printf("par_p3: %f\n", par_p3);
	u_printf("par_p4: %f\n", par_p4);
	u_printf("par_p5: %f\n", par_p5);
	u_printf("par_p6: %f\n", par_p6);
	u_printf("par_p7: %f\n", par_p7);
	u_printf("par_p8: %f\n", par_p8);
	u_printf("par_p9: %f\n", par_p9);
	u_printf("par_p10: %f\n", par_p10);
	u_printf("par_p11: %f\n", par_p11);
}






int BMP390::read_reg_u8(uint8_t reg, uint8_t *value) {
	char txBuf[2];
	char rxBuf[2] = {0, 0};

	txBuf[0] = reg;
	//_i2c->transfer(chipId, txBuf, 1, rxBuf, 2);
	_i2c->write(_addr8bit, txBuf, 1);
	_i2c->read(_addr8bit, rxBuf, 1);

	*value = rxBuf[0];

	return 0;
}

int BMP390::read_reg_s8(uint8_t reg, int8_t *value) {
	char txBuf[2];
	char rxBuf[2] = {0, 0};

	txBuf[0] = reg;
	//_i2c->transfer(chipId, txBuf, 1, rxBuf, 2);
	_i2c->write(_addr8bit, txBuf, 1);
	_i2c->read(_addr8bit, rxBuf, 1);

	*value = rxBuf[0];

	return 0;
}


int BMP390::write_reg_u8(uint8_t reg, uint8_t value) {
	char txBuf[2] = {reg, value};

	//txBuf[0] = reg;
	//txBuf[1] = value;
	_i2c->write(_addr8bit, txBuf, 2);

	return 0;
}


int BMP390::read_reg_u16(uint8_t reg, uint16_t *value) {
	char txBuf[2];
	char rxBuf[2] = {0, 0};

	txBuf[0] = reg;
	//_i2c->transfer(chipId, txBuf, 1, rxBuf, 2);
	_i2c->write(_addr8bit, txBuf, 1);
	_i2c->read(_addr8bit, rxBuf, 2);

	*value = rxBuf[0] | rxBuf[1] << 8;

	return 0;
}


int BMP390::read_reg_s16(uint8_t reg, int16_t *value) {
	char txBuf[2];
	char rxBuf[2] = {0, 0};

	txBuf[0] = reg;
	//_i2c->transfer(chipId, txBuf, 1, rxBuf, 2);
	_i2c->write(_addr8bit, txBuf, 1);
	_i2c->read(_addr8bit, rxBuf, 2);

	*value = (int16_t) (rxBuf[0] | rxBuf[1] << 8);

	return 0;
}


