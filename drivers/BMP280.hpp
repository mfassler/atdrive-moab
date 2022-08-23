#ifndef __BMP280_HPP_
#define __BMP280_HPP_

#include "mbed.h"


class BMP280 {

public:
	BMP280(I2C*);
	ssize_t init();
	int get_data();

	double _temp;
	double _press;


private:

	uint8_t _chipId;

	uint16_t _dig_T1;
	int16_t _dig_T2;
	int16_t _dig_T3;
	uint16_t _dig_P1;
	int16_t _dig_P2;
	int16_t _dig_P3;
	int16_t _dig_P4;
	int16_t _dig_P5;
	int16_t _dig_P6;
	int16_t _dig_P7;
	int16_t _dig_P8;
	int16_t _dig_P9;

	int32_t _raw_press;
	int32_t _raw_temp;

	int32_t _t_fine;


	I2C *_i2c;
	int _addr8bit;
	double _compensate_T_double(int32_t);
	double _compensate_P_double(int32_t);

	int read_reg_u8(uint8_t reg, uint8_t *value) ;
	int write_reg_u8(uint8_t reg, uint8_t value) ;

	int read_reg_u16(uint8_t reg, uint16_t *value) ;
	int read_reg_s16(uint8_t reg, int16_t *value) ;

};

#endif // __BMP280_HPP_
