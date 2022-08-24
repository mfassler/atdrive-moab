#ifndef __BMP390_HPP_
#define __BMP390_HPP_

#include "mbed.h"


// CMD register:
#define BMP3_REGSTER_CMD 0x7E
#define BMP3_CMD_softreset 0xB6
#define BMP3_CMD_fifo_flush 0xB0

// Section 4.3.18 of the datasheet
// OSR register:
#define BMP3_REGSTER_OSR 0x1C
#define BMP3_NO_OVERSAMPLING  0x00
#define BMP3_OVERSAMPLING_2X  0x01
#define BMP3_OVERSAMPLING_4X  0x02
#define BMP3_OVERSAMPLING_8X  0x03
#define BMP3_OVERSAMPLING_16X 0x04
#define BMP3_OVERSAMPLING_32X 0x05

// Section 4.3.21 of the datasheet
// CONFIG register (used for IIR filter):
#define BMP3_REGISTER_CONFIG 0x1F
#define BMP3_IIR_FILTER_DISABLE    0x00
#define BMP3_IIR_FILTER_COEFF_1    0x02
#define BMP3_IIR_FILTER_COEFF_3    0x04
#define BMP3_IIR_FILTER_COEFF_7    0x06
#define BMP3_IIR_FILTER_COEFF_15   0x08
#define BMP3_IIR_FILTER_COEFF_31   0x0A
#define BMP3_IIR_FILTER_COEFF_63   0x0C
#define BMP3_IIR_FILTER_COEFF_127  0x0E


class BMP390 {

public:
	BMP390(I2C*);
	ssize_t init();

	int get_data();

	double _temp;
	double _press;


private:
	uint8_t _chipId;

	void read_calibration();

	void _send_cmd(uint8_t);
	void _set_OSR(uint8_t, uint8_t);
	void _set_IIR_filter_coefficient(uint8_t);

	void read_raw_data();
	uint32_t uncomp_temp;
	uint32_t uncomp_press;

	I2C *_i2c;
	int _addr8bit;

	void compensate_temperature();
	void compensate_pressure();

	int read_reg_u8(uint8_t reg, uint8_t *value);
	int read_reg_s8(uint8_t reg, int8_t *value);

	int write_reg_u8(uint8_t reg, uint8_t value);

	int read_reg_u16(uint8_t reg, uint16_t *value);
	int read_reg_s16(uint8_t reg, int16_t *value);

	// Calibration values:
	double par_t1;
	double par_t2;
	double par_t3;
	double par_p1;
	double par_p2;
	double par_p3;
	double par_p4;
	double par_p5;
	double par_p6;
	double par_p7;
	double par_p8;
	double par_p9;
	double par_p10;
	double par_p11;

};


#endif // __BMP390_HPP_

