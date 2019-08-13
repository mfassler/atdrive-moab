#ifndef __COMPASS_HPP_
#define __COMPASS_HPP_

#include "mbed.h"

// This is for the HERE2 Compass, i2c portion

// The HERE2 compass has a GPS on Async serial.
// In i2c mode, it has a compass and an RGB LED.



//#define ADDR_7_BIT 0x0c
//#define ADDR_8_BIT 0x18  
//  ie:   0x0c << 1

class Compass {

public:
	Compass(PinName, PinName);
	ssize_t init();
	ssize_t get_data(int16_t*);

	int set_leds(uint8_t, uint8_t, uint8_t);

private:
	I2C *_i2c;
	bool _ready;

	// WTF, why can't I make this work?
	//const int _addr8bit = ADDR_8_BIT;
	//const int _addr8bit = 0x18;

	int _compass_addr8bit;
	int _leds_addr8bit;

	void _write_register(char, char);

};

#endif// __COMPASS_HPP_
