
#include "Compass.hpp"


// Compass chip is:  ST LIS3MDL
// (old compass chip was: AK09916 from the HERE2 GPS)

Compass::Compass(PinName sda, PinName scl) {
	_i2c = new I2C(sda, scl);
	//_i2c.frequency(400000);
	_ready = false;

	_compass_addr8bit = 0x1e << 1;
	_leds_addr8bit = 0x55 << 1;
}


void Compass::_write_register(char reg, char value) {
	char txBuf[2] = {reg, value};
	_i2c->write(_compass_addr8bit, txBuf, 2);
}


ssize_t Compass::init() {
	char txBuf[2];
	char rxBuf[2] = {0, 0};


	txBuf[0] = 0x0f;  // WHO_AM_I
	_i2c->write(_compass_addr8bit, txBuf, 1);
	_i2c->read(_compass_addr8bit, rxBuf, 1);

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

ssize_t Compass::get_data(int16_t *xyz) {

	char txBuf[2] = {0x28, 0x0};

	if (!_ready) {
		return 0;
	}

	_i2c->write(_compass_addr8bit, txBuf, 1);
	_i2c->read(_compass_addr8bit, (char*) xyz, 6);

	return 6;
}


int Compass::set_leds(uint8_t red, uint8_t green, uint8_t blue) {

	// Inputs range from 0 to 15 (4 bits)

	char txBuf[5] = {0x01, 0,0,0, 0x83};

	//txBuf[0] = 0x01;  // pwm0 register
	txBuf[1] = blue & 0x0f;
	txBuf[2] = green & 0x0f;
	txBuf[3] = red & 0x0f;
	//txBuf[4] = 0x83;  // enable, and stop auto-incrementing sequential writes
	_i2c->write(_leds_addr8bit, txBuf, 5);

	return 0;
}



