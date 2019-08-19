
#include "ST_LIS3MDL.hpp"


// ST_LIS3MDL chip is:  ST LIS3MDL

ST_LIS3MDL::ST_LIS3MDL(I2C *i2c) {
	_i2c = i2c;
	_ready = false;

	_addr8bit = 0x1e << 1;
}


void ST_LIS3MDL::_write_register(char reg, char value) {
	char txBuf[2] = {reg, value};
	_i2c->write(_addr8bit, txBuf, 2);
}


ssize_t ST_LIS3MDL::init() {
	char txBuf[2];
	char rxBuf[2] = {0, 0};


	txBuf[0] = 0x0f;  // WHO_AM_I
	_i2c->write(_addr8bit, txBuf, 1);
	_i2c->read(_addr8bit, rxBuf, 1);

	if (rxBuf[0] != 0x3d) {
		return -1;
	}

	_write_register(0x20, 0xfc);
	_write_register(0x21, 0x00);
	_write_register(0x22, 0x00);
	_write_register(0x23, 0x0c);
	_write_register(0x24, 0x40);

	_ready = true;
	return 0;
}

ssize_t ST_LIS3MDL::get_data(int16_t *xyz) {

	char txBuf[2] = {0x28, 0x0};

	if (!_ready) {
		return 0;
	}

	_i2c->write(_addr8bit, txBuf, 1);
	_i2c->read(_addr8bit, (char*) xyz, 6);

	return 6;
}


