/**
 * @file controller.cpp
 * @author Jesus Fausto (jvfausto@ucsd.edu)
 * @brief
 * @version 0.1
 * @date 2022-07-17
 *
 * @copyright Copyright 2022 Triton AI
 *
 *
 */

#include "controller.hpp"
#include "mbed.h"
#include "PwmIn.h"
#define  PI 3.141592654

namespace tritonai {
namespace gkc{

struct Translation {

//The steering function takes in the steering duty cycle with float precision and returns the steering angle in Radians
//This may be a good place to implement the limit switch
float Steering(float Steering_Duty) {
     //50 degree maximum left and right
     double Steering_Ang;
     double Steering_Ang_rad;
     double a = -(50000/49)*.6;
     double b = -1000*.6;
     if(Steering_Duty <= 0.151) Steering_Ang = b * (Steering_Duty-0.151); //this is not correct uses 50 degrees max
     else if(Steering_Duty >= 0.151) Steering_Ang = a * (Steering_Duty-0.151);

    //ensure that Steering angle does not exceed 50 degrees
    //Max and Min steering angles are set to +- 20deg in config.hpp header file
     if(Steering_Ang > MAX__WHEEL_STEER_DEG) Steering_Ang = MAX__WHEEL_STEER_DEG;
     if(Steering_Ang <MIN__WHEEL_STEER_DEG) Steering_Ang = MIN__WHEEL_STEER_DEG;
     if ( -1 < Steering_Ang && Steering_Ang < 1 ) Steering_Ang = 0.0;

    Steering_Ang_rad = Steering_Ang * (PI/180);

    return Steering_Ang_rad;

    //return Steering_Duty;
}

//The trigger function takes in the trigger duty cycle and returns percentage of throttle output.
float Trigger(float Trigger_Duty){   //2000x-300=y
    float throttle;
    if (Trigger_Duty <= 0.151) throttle = 0.0;
   
    else throttle = (2000 * Trigger_Duty) - 300;
   
    if(throttle > 100) throttle = 100;

    throttle = throttle/100; //added this new line after leaving lab

    return throttle;
}

//Red is a boolean function that checks the range of the dutycycle argument
//if Red_Duty is not between 19% and 25% the return is false
bool Red(float Red_Duty){
    if (Red_Duty >= 0.19 && Red_Duty < .25) return true;
    else return false;
}

};

class RCController{
//The following functions can be used outside the RCController class
public:
    RCController();
    void getSensor();
//The following attributes(variables) can only be uned in the RCController class
private:
    float rolling_average;
    Controller* cont_p;

    PwmIn* steerVal;
    PwmIn* throttleVal;
    PwmIn* switchVal;

    Thread sensor_write;
    Translation Map;
};
}
}