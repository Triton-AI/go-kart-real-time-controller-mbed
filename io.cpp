#include "mbed.h"

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

//rs232 port
//used for the MRC (Main Robot Computer) - RTC comm
//confugured at 9600 bauds, but other frecuencies should work too
BufferedSerial rs232(PC_12, PD_2, 9600);

//pwm port PA_5 used connected to the throttle
//It will output a pwm at frecuency 10khz (configured on Actuator::Actuator)
//The function Actuator::update_throttle() will update the duty cycle to the desired one
PwmOut throttle_pwm(PA_5);

CAN can1(PD_0, PD_1, 500000); //RX, TX
CAN can2(PB_5, PB_6, 500000); //RX, TX