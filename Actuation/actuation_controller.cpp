#include "actuation_controller.hpp"
//#include "../peripherals.h"


DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

#define THROTTLE_PWM_PIN PA_5
PwmOut pin(THROTTLE_PWM_PIN);

#define THROTTLE_CONTROL_LOOP_PERIOD 10ms
#define STEERING_CONTROL_LOOP_PERIOD 10ms
#define BRAKE_CONTROL_LOOP_PERIOD 10ms



CAN can1(PD_0, PD_1, 500000); //RX, TX

//Constructor for Actuator class
//It runs when a new Actuator class is created
//It will make the tiker the class has to call the update_throttle, update_steering and update_brake functions at the frecuency configured on the global constants
Actuator::Actuator()
{
    queue_throttle.call_every(THROTTLE_CONTROL_LOOP_PERIOD,callback(this, &Actuator::update_throttle));
    thread_throttle.start(callback(this, &Actuator::run_queue_throttle));
    queue_steering.call_every(STEERING_CONTROL_LOOP_PERIOD,callback(this, &Actuator::update_steering));
    thread_steering.start(callback(this, &Actuator::run_queue_steering));
    queue_brake.call_every(BRAKE_CONTROL_LOOP_PERIOD ,callback(this, &Actuator::update_brake));
    thread_brake.start(callback(this, &Actuator::run_queue_brake));
    pin.period(0.0001f);
}    

//This function updates the throttle
//Interface:    pwmOut declared globaly as throttle_pin on peripherals.h
//Uses:         the private variable throttle, though the actuator's public function get_throttle(). That will be a float in the range [0, 100]
//Description:  It transforms the desired throttle from [0, 100] to the desired duty cytle. The transformation is complex, because the external device we use is not lineal, and this compensates for that to make the know outpur lineal
void    Actuator::update_throttle()
{
    /*Code to use the throttle*/
    float f = get_throttle();
    //int initial = 0;
    //if(initial == 0){
        
        //initial++;
    //}
    // here I am normalizing the input 
    f = f/100;
    if(f < 1 && f > 0){
        if(f > 0.5){
            f = exp(f*3.5);
            f = f/40;
        }
        else if(f < 0.5){
            f = exp(f*7);
            f = f/130;
        }
        else {
            f = exp(f);
            f = f/10;  
        }
    }  
        pin.write(f);
    
    /*Use get_throttle() instead of getting the input as an argument*/
    //printf("Throttle %d\n", get_throttle());
    led1 = !led1;
}

//This function updates the steering
//Interface:    uart(unbuffered serial) declared globaly as ____ on peripherals.h
//Uses          the private variable steering, though the actuator's public function get_steering(). The value is an angle in radians
//Desciption    it transforms the desired angle from the range [___] to an intiger in the range [___] to send to the servomotor controller
void    Actuator::update_steering()
{
    /*Code to use the steering*/
    /*Use get_steering() instead of getting the input as an argument*/
    //printf("Steering %d\n", get_steering());
    led2 = !led2;
}

//This function updates the brake
//Interface:    CAN bus declared globaly as can1 on peripherals.h
//Uses:         The private variable brake, though the actuator's public function get_brake(). The value is a float in the range [0, 1]
//Description   It transforms the desired braking power from the range [0, 1] to an int on the range [0, 2000], encodes it on a buffer as specified by the brake datasheet and sends it
//                  The message has to have the ID 0x00FF0000 and use CANExtended. That is the deffault value of the brake controller
void    Actuator::update_brake()
{
    /*Code to use the brake*/
    /*Use get_brake() instead of getting the input as an argument*/
    //printf("Brake %d\n", get_brake());

    int pos = get_brake() * 2300;
    //                                 //pos = 0, Auto replay Off {0x0F, 0x4A, 0xE8, 0xC3, 0, 0, 0, 0}
    static unsigned char message[8] = {0x0F, 0x4A, 0x00, 0xC0, 0, 0, 0, 0};
    if (pos > 2500)
        return;
    message[2] = pos & 0xFF;
    message[3] = 0xC0 | ((pos >> 8) & 0x1F);
    can1.write(CANMessage(0x00FF0000, message, 8, CANData, CANExtended)); //(id, &buffer, len)
    //printf("Repositioned to %u\n", pos);
    //printf("%x %x %x %x\n", message[0], message[1], message[2],message[3]);
    //ThisThread::sleep_for(100);

    led3 = !led3;
}
