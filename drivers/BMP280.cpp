
#include "BMP280.hpp"


BMP280::BMP280(I2C *i2c) {
	_i2c = i2c;
	_addr8bit = 0x76 << 1;
}


double BMP280::_compensate_T_double(int32_t adc_T) {
	double var1, var2, T;

	var1 = (((double)adc_T)/16384.0 - ((double)_dig_T1)/1024.0) * ((double)_dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)_dig_T1)/8192.0) *
	(((double)adc_T)/131072.0 - ((double) _dig_T1)/8192.0)) * ((double)_dig_T3);
	_t_fine = (int32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;
	return T;
}


double BMP280::_compensate_P_double(int32_t adc_P) {
	double var1, var2, p;
	var1 = ((double)_t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double)_dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)_dig_P5) * 2.0;
	var2 = (var2/4.0)+(((double)_dig_P4) * 65536.0);
	var1 = (((double)_dig_P3) * var1 * var1 / 524288.0 + ((double)_dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0)*((double)_dig_P1);
	if (var1 == 0.0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)_dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)_dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)_dig_P7)) / 16.0;
	return p;
}


int BMP280::read_reg_u8(uint8_t reg, uint8_t *value) {
	char txBuf[2];
	char rxBuf[2] = {0, 0};

	txBuf[0] = reg;
	//_i2c->transfer(chipId, txBuf, 1, rxBuf, 2);
	_i2c->write(_addr8bit, txBuf, 1);
	_i2c->read(_addr8bit, rxBuf, 1);

	*value = rxBuf[0];

	return 0;
}


int BMP280::write_reg_u8(uint8_t reg, uint8_t value) {
	char txBuf[2] = {reg, value};

	//txBuf[0] = reg;
	//txBuf[1] = value;
	_i2c->write(_addr8bit, txBuf, 2);

	return 0;
}


int BMP280::read_reg_u16(uint8_t reg, uint16_t *value) {
	char txBuf[2];
	char rxBuf[2] = {0, 0};

	txBuf[0] = reg;
	//_i2c->transfer(chipId, txBuf, 1, rxBuf, 2);
	_i2c->write(_addr8bit, txBuf, 1);
	_i2c->read(_addr8bit, rxBuf, 2);

	*value = rxBuf[0] | rxBuf[1] << 8;

	return 0;
}


int BMP280::read_reg_s16(uint8_t reg, int16_t *value) {
	char txBuf[2];
	char rxBuf[2] = {0, 0};

	txBuf[0] = reg;
	//_i2c->transfer(chipId, txBuf, 1, rxBuf, 2);
	_i2c->write(_addr8bit, txBuf, 1);
	_i2c->read(_addr8bit, rxBuf, 2);

	*value = (int16_t) (rxBuf[0] | rxBuf[1] << 8);

	return 0;
}



ssize_t BMP280::init() {
	// Chip ID
	read_reg_u8(0xd0, &_chipId);
	if (_chipId != 0x58) {
		return -1;
	}

	// We seem to need these delays  (2 ms) in order to get accurate
	// data from these registers.  TODO:  is this normal?

	read_reg_u16(0x88, &_dig_T1);
	ThisThread::sleep_for(2);
	read_reg_s16(0x8a, &_dig_T2);
	ThisThread::sleep_for(2);
	read_reg_s16(0x8c, &_dig_T3);
	ThisThread::sleep_for(2);
	read_reg_u16(0x8e, &_dig_P1);
	ThisThread::sleep_for(2);
	read_reg_s16(0x90, &_dig_P2);
	ThisThread::sleep_for(2);
	read_reg_s16(0x92, &_dig_P3);
	ThisThread::sleep_for(2);
	read_reg_s16(0x94, &_dig_P4);
	ThisThread::sleep_for(2);
	read_reg_s16(0x96, &_dig_P5);
	ThisThread::sleep_for(2);
	read_reg_s16(0x98, &_dig_P6);
	ThisThread::sleep_for(2);
	read_reg_s16(0x9a, &_dig_P7);
	ThisThread::sleep_for(2);
	read_reg_s16(0x9c, &_dig_P8);
	ThisThread::sleep_for(2);
	read_reg_s16(0x9e, &_dig_P9);
	ThisThread::sleep_for(2);


	// Start-up the device
	// set 0xf4 to put the chip into "normal" mode (0b11 == normal mode)
	// set 0xf5 to set the sample rate  (0b101 == 1 sample per second)

	// 0xf4 ctrl_meas
	//  0b???xxxxx - oversampling of temp data   0b001 == 1
	//  0bxxx???xx - oversampling of pres data   0b011 == 4
	//  0bxxxxxx?? - power mode  0b11 == normal
	write_reg_u8(0xf4, 0x2f);

	// 0xf5 config
	//  0b???xxxxx - t_sb   0b011 (stanbdy time is 0.25 seconds)
	//  0bxxx???xx - filter  (IIR filter, smaller is faster)  set to 4  (0b010?)
	//  0bxxxxxx?x - nothing
	//  0bxxxxxxx? - spi3w_en  (set to zero, we're not using spi)
	write_reg_u8(0xf5, 0x68);

	return 0;
}


int BMP280::get_data() {
	char txBuf[2];
	char rxBuf[3];

	txBuf[0] = 0xf7;
	_i2c->write(_addr8bit, txBuf, 1);
	_i2c->read(_addr8bit, rxBuf, 3);

	_raw_press = (rxBuf[0] << 12) | (rxBuf[1] << 4) | (rxBuf[2] >> 4);

	txBuf[0] = 0xfa;
	_i2c->write(_addr8bit, txBuf, 1);
	_i2c->read(_addr8bit, rxBuf, 3);

	_raw_temp = (rxBuf[0] << 12) | (rxBuf[1] << 4) | (rxBuf[2] >> 4);

	_temp = _compensate_T_double(_raw_temp);
	_press = _compensate_P_double(_raw_press);

	return 0;
}



