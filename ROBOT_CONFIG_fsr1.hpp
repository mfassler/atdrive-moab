#ifndef __ROBOT__CONFIG__HPP
#define __ROBOT__CONFIG__HPP


#define _MOAB_IP_ADDRESS "192.168.31.201"
#define _NETMASK "255.255.255.0"
#define _DEFUALT_GATEWAY "192.168.31.1"
#define _BROADCAST_IP_ADDRESS "192.168.31.255"

#define _AUTOPILOT_IP_ADDRESS "192.168.31.222"


#define _STEERING_PW_CENTER 0.001616
#define _STEERING_PW_RANGE 0.000350

#define IMU_STORED_CONFIG
const char _IMU_CONFIG[] = {
	// Config register:
	0x55,

	// 22 bytes of config data
	0xd4, 0xff, 0xb8, 0xff,
	0xe5, 0xff, 0x22, 0x00,
	0xd7, 0xff, 0x3c, 0xfe,
	0x01, 0x00, 0x01, 0x00,
	0x00, 0x00, 0xe8, 0x03,
	0x67, 0x03,
};


#endif // __ROBOT__CONFIG__HPP
