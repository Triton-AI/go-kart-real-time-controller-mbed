#include "actuation_controller.hpp"

#define THROTTLE_CONTROL_LOOP_PERIOD 100ms
#define STEERING_CONTROL_LOOP_PERIOD 100ms
#define BRAKE_CONTROL_LOOP_PERIOD 100ms


Actuator::Actuator()
{
    ticker_throttle.attach(callback(this, &Actuator::update_throttle), THROTTLE_CONTROL_LOOP_PERIOD);
    ticker_steering.attach(callback(this, &Actuator::update_steering), STEERING_CONTROL_LOOP_PERIOD);
    ticker_brake.attach(callback(this, &Actuator::update_brake), BRAKE_CONTROL_LOOP_PERIOD);
}

void    Actuator::update_throttle()
{
    /*Code to use the throttle*/
    PwmOut pin(THROTTLE_PWM_PIN);
    void throttle(float  f = get_throttle()) {
int initial = 0;
if(initial == 0){
    pin.period(0.0001f);
    initial++;
}
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
}
    /*Use get_throttle() instead of getting the input as an argument*/
    printf("Throttle\n");
}

void    Actuator::update_steering()
{
    /*Code to use the steering*/
    /*Use get_steering() instead of getting the input as an argument*/
    printf("Steering\n");
}

void    Actuator::update_brake()
{
    /*Code to use the brake*/
    /*Use get_brake() instead of getting the input as an argument*/
    printf("Brake\n");
}
