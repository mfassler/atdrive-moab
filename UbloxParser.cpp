//#include "mbed.h"

#include "UbloxParser.hpp"


UbloxParser::UbloxParser() {
	_rxIdx = 0;
	_rxFirstChar = false;
	_rxSecondChar = false;

	// First two sync chars are always the same:
	_rxBuf[0] = 0xb5;
	_rxBuf[1] = 0x62;
}


int UbloxParser::rx_char(int c) {

	if (!_rxFirstChar) {
		if (c == 0xb5) {
			_rxFirstChar = true;
		}
	} else if (!_rxSecondChar) {
		if (c == 0x62) {
			_rxSecondChar = true;
			_rxIdx = 2;
		} else {
			_rxFirstChar = false;
		}
	} else {
		_rxBuf[_rxIdx] = c;

		switch(_rxIdx) {
		case 2:
			//_msgClass = c;
			_rxIdx++;
			break;
		case 3:
			//_msgId = c;
			_rxIdx++;
			break;
		case 4:
			_length_lower = c;
			_rxIdx++;
			break;
		case 5:
			//_lenght_upper = c;
			_payload_length = _length_lower | (c << 8);
			if (_payload_length > 300) { // TODO:  what is the max payload length?
				_payload_length = 300;
			}
			_total_length = _payload_length + 8;
			_maxIdx = _payload_length + 8 - 1;
			_rxIdx++;
			break;
		default:
			if (_rxIdx < _maxIdx) {
				_rxIdx++;
			} else {
				_rxFirstChar = false;
				_rxSecondChar = false;
				
				return _total_length;
			}
			break;
		}

	}
	return 0;  // no new data yet...
}


