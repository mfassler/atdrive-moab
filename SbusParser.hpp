#ifndef __SBUS_PARSER_HPP
#define __SBUS_PARSER_HPP

#include <stdint.h>


struct sbus_udp_payload {
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint16_t ch4;
	uint16_t ch5;
	uint16_t ch6;
	uint16_t ch7;
	uint16_t ch8;
	uint16_t ch9;
	uint16_t ch10;
	uint16_t ch11;
	uint16_t ch12;
	uint16_t ch13;
	uint16_t ch14;
	uint16_t ch15;
	uint16_t ch16;
	bool failsafe;
	bool frame_lost;
};


class SbusParser {

private:
	char _rxBuf[32];
	int _rxIdx;
	bool _rxInBand;
	int _processSbusMessage();
	struct sbus_udp_payload *_sup;

public:
	SbusParser(struct sbus_udp_payload*);
	int rx_char(int);
};

#endif // __SBUS_PARSER_HPP
