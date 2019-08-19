#ifndef __ST_LIS3MDL_HPP_
#define __ST_LIS3MDL_HPP_

#include "mbed.h"


class ST_LIS3MDL {

public:
	ST_LIS3MDL(I2C *);
	ssize_t init();
	ssize_t get_data(int16_t*);

private:
	I2C *_i2c;
	bool _ready;

	int _addr8bit;

	void _write_register(char, char);
};

#endif  // __ST_LIS3MDL_HPP_
