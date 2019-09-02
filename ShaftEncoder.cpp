
#include "ShaftEncoder.hpp"



ShaftEncoder::ShaftEncoder(PinName pinName, EthernetInterface *net) {

	_pin = new InterruptIn(pinName, PullUp);

	_tx_sock.open(net);
	_tx_sock.bind(51810);

	_pin->rise(callback(this, &ShaftEncoder::_rise_interrupt));
	_pin->fall(callback(this, &ShaftEncoder::_fall_interrupt));

	//last_fall = 0;
	//last_rise = 0;
	last_pulse = 0;  // rise or fall
	last_last_pulse = 0;  // rise or fall

	udpPacket.ms_since_last_event = 0;
}




void ShaftEncoder::start() {
	_shaft_thread.start(callback(this, &ShaftEncoder::_shaft_worker));
}


void ShaftEncoder::_rise_interrupt() {
	last_last_pulse = last_pulse;
	last_pulse = rtos::Kernel::get_ms_count();

	_event_flags.set(_EVENT_FLAG_SHAFT_RISE);
}


void ShaftEncoder::_fall_interrupt() {
	last_last_pulse = last_pulse;
	last_pulse = rtos::Kernel::get_ms_count();

	_event_flags.set(_EVENT_FLAG_SHAFT_FALL);
}


void ShaftEncoder::_shaft_worker() {
    uint32_t flags_read;
    while (true) {
        flags_read = _event_flags.wait_any(_EVENT_FLAG_SHAFT_FALL | _EVENT_FLAG_SHAFT_RISE, 100);

        // We will act only on 1 flag at a time, even if there are multiple
        //
        // (b/c: the interrupts *shouldn't* be so fast that they out-pace this 
        //  code, but if they are, we will ignore them, because that's noise 
        //  on the wires...)

        if (flags_read & osFlagsError) {

            //  Timeout

			udpPacket.mtype = 0; // timeout

			uint64_t tDelta = rtos::Kernel::get_ms_count() - last_pulse;
			if (tDelta > 65534) {
				udpPacket.ms_since_last_event = 65535;
			} else {
				udpPacket.ms_since_last_event = tDelta;
			}

			_tx_sock.sendto(_BROADCAST_IP_ADDRESS, SHAFT_PORT, (char*)&udpPacket, sizeof(udpPacket));

        } else if (flags_read & _EVENT_FLAG_SHAFT_FALL) {

			udpPacket.mtype = 1; // fall

			uint64_t tDelta = last_pulse - last_last_pulse;
			if (tDelta > 65534) {
				udpPacket.ms_since_last_event = 65535;
			} else {
				udpPacket.ms_since_last_event = tDelta;
			}

			_tx_sock.sendto(_BROADCAST_IP_ADDRESS, SHAFT_PORT, (char*)&udpPacket, sizeof(udpPacket));


        } else if (flags_read & _EVENT_FLAG_SHAFT_RISE) {

			udpPacket.mtype = 2; // rise

			uint64_t tDelta = last_pulse - last_last_pulse;
			if (tDelta > 65534) {
				udpPacket.ms_since_last_event = 65535;
			} else {
				udpPacket.ms_since_last_event = tDelta;
			}

			_tx_sock.sendto(_BROADCAST_IP_ADDRESS, SHAFT_PORT, (char*)&udpPacket, sizeof(udpPacket));

        }
    }
}


