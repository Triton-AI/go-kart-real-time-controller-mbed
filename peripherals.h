#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "mbed.h"

//The throttle, steering and brake functions will be called at this period
#define THROTTLE_CONTROL_LOOP_PERIOD 10ms
#define STEERING_CONTROL_LOOP_PERIOD 10ms
#define BRAKE_CONTROL_LOOP_PERIOD 10ms

//Onboard LEDs
extern DigitalOut led1;
extern DigitalOut led2;
extern DigitalOut led3;

//CAN ports configured to 500Khz
//can1 used for the comm with the brake so far
extern CAN can1; //RX, TX
extern CAN can2; //RX, TX

//rs232 port
//used for the MRC (Main Robot Computer) - RTC comm
//confugured at 9600 bauds, but other frecuencies should work too
extern BufferedSerial rs232; //TX, Rx

//pwm port _____ used connected to the throttle
//It will output a pwm at frecuency ____
//The function Actuator::update_throttle() will update the duty cycle to the desired one
extern PwmOut throttle_pin;




#endif