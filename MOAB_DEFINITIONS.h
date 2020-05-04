#ifndef __MOAB_DEFINITIONS_H
#define __MOAB_DEFINITIONS_H



// UBLOX messages from GPS module (not used right now):
#define UDP_PORT_GPS_UBLOX 37110

// not used:
//#define UDP_PORT_ODOMETRY 27112

// NMEA messages from GPS module (broadcast):
#define UDP_PORT_GPS_NMEA 37113

// custom binary protocol for IMU data (broadcast):
#define UDP_PORT_IMU 27114

// not used anymore:
//#define UDP_PORT_IMU_CONFIG 27115


// printf() style debug messages (broadcast):
#define UDP_PORT_DEBUG 31337

#define UDP_PORT_SBUS 31338

#define UDP_PORT_PUSHBUTTON 31345


#endif // __MOAB_DEFINITIONS_H
