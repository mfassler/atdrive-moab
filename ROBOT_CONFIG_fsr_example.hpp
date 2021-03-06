#ifndef __ROBOT__CONFIG__HPP
#define __ROBOT__CONFIG__HPP


#define _MOAB_IP_ADDRESS "192.168.31.201"
#define _NETMASK "255.255.255.0"
#define _DEFUALT_GATEWAY "192.168.31.1"
#define _BROADCAST_IP_ADDRESS "192.168.31.255"

#define _AUTOPILOT_IP_ADDRESS "192.168.31.222"


#define _STEERING_PW_CENTER 0.001616
#define _STEERING_PW_RANGE 0.000350

// Throttle PWM center, range, max, min can be overwritten here:
//#define _THROTTLE_PW_CENTER 0.001515  // <--default
//#define _THROTTLE_PW_RANGE 0.000400  // <-- default
//#define _THROTTLE_PW_MAX .001915  // <-- default
#define _THROTTLE_PW_MAX .001735
//#define _THROTTLE_PW_MIN 0.001115  // <-- default

#define _SCALE_STEERING 1.0
#define _SCALE_THROTTLE 1.0
#define _SCALE_BRAKE 1.0


#define _TWO_SHAFT_ENCODERS

// On/Off digital out:
#define USER_DIGITAL_OUT_0 PF_15

#define _USE_RADIO169


// PWM Outputs
// -----------
// ** NOTE ***  Not all outputs can do PWM.  Not all pin combinations are possible.

// PWM-out using a Duty Cycle from 0 to 255 (0% to 100%):
//#define USER_PWM_OUT_0 PE_10
//#define USER_PWM_OUT_1 PE_12
//#define USER_PWM_OUT_2 PE_14

// PWM-out using a pulse-width in micro-seconds (Futaba servo style):
//#define USER_SERVO_OUT_0 PE_13


#endif // __ROBOT__CONFIG__HPP
