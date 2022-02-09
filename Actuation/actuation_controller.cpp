#include "actuation_controller.hpp"

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

Actuator::Actuator()
{
    ticker_throttle.attach(callback(this, &Actuator::update_throttle), 100ms);
    ticker_steering.attach(callback(this, &Actuator::update_steering), 100ms);
    ticker_brake.attach(callback(this, &Actuator::update_brake), 100ms);
}

void    Actuator::update_throttle()
{
    /*Code to use the throttle*/
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
