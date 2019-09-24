
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

	//  0x20 -- CTRL_REG1
	// msb: 0b10000000  - temp sensor enable
	//      0b0xx00000  - X and Y operating mode  (11: ultra-high performance)
	//      0b000xxx00  - output rate  (ignored b/c of FAST_ODR)
	//      0b00000010  - FAST_ODR (data rate faster than 80 Hz)
	//      0b00000001  - self-test enabled
	_write_register(0x20, 0xfc);

	// 0x21 -- CTRL_REG2
	// msb: 0bx0000000  - msb must always be zero
	//      0b0xx00000  - fullscale range (00:  +/0 4 gauss)
	//      0b000x0000  - must always be zero
	//      0b0000x000  - reboot
	//      0b00000x00  - soft_rst
	//      0b000000xx  - must always be zero
	_write_register(0x21, 0x00);

	// 0x22 -- CTRL_REG3
	// 0b00X00000 - low-power
	// 0b00000X00 - SPI mode
	// 0b000000xx - operating mode (00: continuous conversion)
	//   all others must be set to zero
	_write_register(0x22, 0x00);

	// 0x23 -- CTRL_REG4
	// 0b0000xx00 - Z operating mode (11: ultra-high performance)
	// 0b000000x0 - endian:  (0: little-endian)
	//   all others must be set to zero
	_write_register(0x23, 0x0c);

	// 0x24 -- CTRL_REG5
	// 0bx0000000 - fast_read
	// 0b0x000000 - Block Data Update   1: both high-byte and low-byte of reg updated at same time
	//   all others must be set to zero
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


