#ifndef __ROBOT__CONFIG__HPP
#define __ROBOT__CONFIG__HPP


#define _MOAB_IP_ADDRESS "192.168.11.20"
#define _NETMASK "255.255.255.0"
#define _DEFUALT_GATEWAY "192.168.11.1"
#define _BROADCAST_IP_ADDRESS "192.168.11.255"
#define _AUTOPILOT_IP_ADDRESS "192.168.11.40"

//#define _STEERING_PW_CENTER 0.001424
//#define _STEERING_PW_RANGE 0.000391

#define IMU_STORED_CONFIG
const char _IMU_CONFIG[] = {
	// Config register:
	0x55,

	// 22 bytes of config data
	0x00, 0x00, 0xff, 0xff,
	0xfc, 0xff, 0xa2, 0x00,
	0x3b, 0x01, 0xd2, 0x01,
	0xfe, 0xff, 0x01, 0x00,
	0x00, 0x00, 0xe8, 0x03,
	0xaf, 0x02
};

#endif // __ROBOT__CONFIG__HPP