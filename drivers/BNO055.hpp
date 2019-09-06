#ifndef __BNO055_HPP_
#define __BNO055_HPP_

#include "mbed.h"


class BNO055 {

public:
	BNO055(I2C*);
	ssize_t init();
	int get_data();

	char data[20];

private:
	I2C *_i2c;
	int _addr8bit;

	bool _ready = false;

	int read_reg_u8(uint8_t reg, uint8_t *value) ;
	int write_reg_u8(uint8_t reg, uint8_t value) ;

	int read_reg_u16(uint8_t reg, uint16_t *value) ;
	int read_reg_s16(uint8_t reg, int16_t *value) ;
};


#endif // __BNO055_HPP_
