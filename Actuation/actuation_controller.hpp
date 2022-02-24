#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "mbed.h"

class Actuator
{
    public:
        Actuator();

        //Fucntions to update, or use the variables throttle, steering and brake
        float   get_throttle()          {return throttle;}
        float   get_steering()          {return steering;}
        float   get_brake()             {return brake;}
        void    set_throttle(float f)   {throttle = f;}
        void    set_steering(float f)   {steering = f;}
        void    set_brake(float f)      {brake = f;}

        //These functins will use the variables throttle, steering and brake to send the latest value to the actuators
        void    update_throttle();
        void    update_steering();
        void    update_brake();

        //this objects will call the functions at the desired frecuency. They are configured at the Actuator contructor
        Thread  thread_throttle;
        Thread  thread_steering;
        Thread  thread_brake;

        EventQueue queue_throttle;
        EventQueue queue_steering;
        EventQueue queue_brake;

        void    run_queue_throttle() {queue_throttle.dispatch_forever();}
        void    run_queue_steering() {queue_steering.dispatch_forever();}
        void    run_queue_brake() {queue_brake.dispatch_forever();}

        
    private:
        //This variables will be updated by the communication with the MRC (Main Robot Computer) and used by the functions update_throttle, update_steering and update_brake to update the different actuators
        float   throttle = 0;
        float   steering = 0;
        float   brake = 0;
};

#endif