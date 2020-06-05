#ifndef __ROBOT__CONFIG__HPP
#define __ROBOT__CONFIG__HPP


#define _MOAB_IP_ADDRESS "192.168.32.201"
#define _NETMASK "255.255.255.0"
#define _DEFUALT_GATEWAY "192.168.32.1"
#define _BROADCAST_IP_ADDRESS "192.168.32.255"

#define _AUTOPILOT_IP_ADDRESS "192.168.32.222"


#define _STEERING_PW_CENTER 0.001664
#define _STEERING_PW_RANGE 0.000350

#define _TWO_SHAFT_ENCODERS


// On/Off digital out:
#define USER_DIGITAL_OUT_0 PF_15


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
