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
