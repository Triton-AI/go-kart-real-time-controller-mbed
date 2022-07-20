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

#include "RCController.hpp"
#include "config.hpp"

namespace tritonai {
namespace gkc{
    RCController::RCController(){

        cont_p = new Controller();
        cont_p->deactivate_controller();
        steerVal = new PwmIn(Steer_Pin);  
        throttleVal = new PwmIn(Throttle_Pin); 
        switchVal = new PwmIn(Red_Pin); 
        isRC = true;
        sensor_write.start(callback(this,&RCController::getSensor));
    }

    void RCController::getSensor(){
        float currSteer, currThrottle, currBreak = 0;
        bool currSwitch;

        float pwmSteer = steerVal->dutycycle();
        float pwmThrottle = throttleVal->dutycycle();
        float pwmSwitch = switchVal->dutycycle();

        currSteer = Map.Steering(pwmSteer);
        currThrottle = Map.Trigger(pwmThrottle);
        //currBreak = toBreak(pwmThrottle);
        currSwitch = Map.Red(pwmSwitch);

        if(currSwitch){
            if(!isRC){
                cont_p->deactivate_controller();
                isRC = true;
            }
            cont_p->set_actuation_values(currSteer, currThrottle, currBreak);
        }
        else{
            if(isRC){
                cont_p->activate_controller();
                isRC = false;
            }       
        }
    }

}

}