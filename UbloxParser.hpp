#ifndef __UBLOX_PARSER_HPP
#define __UBLOX_PARSER_HPP

#include <stdint.h>


class UbloxParser {
public:
	UbloxParser();
	int rx_char(int);
	char _rxBuf[2048];  // TODO:  what is the max length of a ublox message?

private:
	int _rxIdx;

	bool _rxFirstChar;
	bool _rxSecondChar;
	char _msgClass;
	char _msgId;
	char _length_lower;
	uint16_t _payload_length;
	uint16_t _total_length;
	uint16_t _maxIdx;
};

#endif // __UBLOX_PARSER_HPP
