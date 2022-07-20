/**
 * @file controller.cpp
 * @author Jesus Fausto (jvfausto@ucsd.edu)
 * @brief
 * @version 0.1
 * @date 2022-07-17
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#include "controller.hpp"
#include "mbed.h"
#include "PwmIn.h"
#define  PI 3.141592654

namespace tritonai {
namespace gkc{

struct Translation {

float Steering(float Steering_Duty) {
     //50 degree maximum left and right
     double Steering_Ang;
     double Steering_Ang_rad;
     double a = -(50000/49);
     double b = -1000;
     if(Steering_Duty <= 0.151) Steering_Ang = b * (Steering_Duty-0.151);
     else if(Steering_Duty >= 0.151) Steering_Ang = a * (Steering_Duty-0.151);

    //ensure that Steering angle does not exceed 50 degrees
     if(Steering_Ang > 50) Steering_Ang = 50;
     if(Steering_Ang <-50) Steering_Ang = -50;
     if ( -1 < Steering_Ang && Steering_Ang < 1 ) Steering_Ang = 0.0;

    Steering_Ang_rad = Steering_Ang * (PI/180);

    return Steering_Ang_rad;

    //return Steering_Duty;
}

float Trigger(float Trigger_Duty){   //2000x-300=y
    float throttle;
    if (Trigger_Duty <= 0.151) throttle = 0.0;
   
    else throttle = (2000 * Trigger_Duty) - 300;
   
    if(throttle > 100) throttle = 100;

    throttle = throttle/100; //added this new line after leaving lab

    return throttle;
}

bool Red(float Red_Duty){
    if (Red_Duty >= 0.19) return true;
    else return false;
}

};

class RCController{
public:
    RCController();
    void getSensor();

private:

    Controller* cont_p;
    PwmIn* steerVal;
    PwmIn* throttleVal;
    PwmIn* switchVal;

    Thread sensor_write;
    Translation Map;
    bool isRC;
};
}
}