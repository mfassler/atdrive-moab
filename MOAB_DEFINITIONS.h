#ifndef __MOAB_DEFINITIONS_H
#define __MOAB_DEFINITIONS_H



// UBLOX messages from GPS module (not used right now):
#define UDP_PORT_GPS_UBLOX 27110

// not used:
//#define UDP_PORT_ODOMETRY 27112

// NMEA messages from GPS module (broadcast):
#define UDP_PORT_GPS_NMEA 27113

// custom binary protocol for IMU data (broadcast):
#define UDP_PORT_IMU 27114

// not used anymore:
//#define UDP_PORT_IMU_CONFIG 27115

// RTCM3 messages from GPS module (broadcast):
#define UDP_PORT_GPS_RTCM3 27117


// printf() style debug messages (broadcast):
#define UDP_PORT_DEBUG 31337

#define UDP_PORT_SBUS 31338
// 31339 is used for something else on atdrive-moab-tools...

#define UDP_PORT_RADIO169 31340

#define UDP_PORT_PUSHBUTTON 31345


enum Moab_State_t {
	NoSignal = 0,
	Stop = 1,  // stop with brakes
	Manual = 2,
	Auto = 3,
	Stop_no_brakes = 4, // stop without brakes
	Auto_no_autopilot = 5,  // Auto, but timeout for message from auto-pilot
	External_safety = 6  // External switch to disable the auto-pilot
}; 


#endif // __MOAB_DEFINITIONS_H
