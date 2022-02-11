#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "mbed.h"

class Actuator
{
    public:
        Actuator();

        float   get_throttle()          {return throttle;}
        float   get_steering()          {return steering;}
        float   get_brake()             {return brake;}
        void    set_throttle(float f)   {throttle = f;}
        void    set_steering(float f)   {steering = f;}
        void    set_brake(float f)      {brake = f;}


        void    update_throttle();
        void    update_steering();
        void    update_brake();

        Ticker  ticker_throttle;
        Ticker  ticker_steering;
        Ticker  ticker_brake;

        
    private:
        float   throttle = 0;
        float   steering = 0;
        float   brake = 0;
};

#endif