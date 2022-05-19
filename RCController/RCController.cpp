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

namespace tritonai {
namespace gkc{
    RCController::RCController(PwmIn* chan1, 
                PwmIn* chan2, PwmIn* chan3){
        cont_p = new Controller();
        cont_p->deactivate_controller();
        steerVal = chan1;
        throttleVal = chan2;
        switchVal = chan3;
        isRC = true;
        sensor_write.start(callback(this,&RCController::getSensor));
    }

    void RCController::getSensor(){
        float currSteer, currThrottle, currBreak;
        bool currSwitch;

        float pwmSteer = steerVal->dutycycle();
        float pwmThrottle = throttleVal->dutycycle();
        float pwmSwitch = switchVal->dutycycle();

        currSteer = toAngle(pwmSteer);
        currThrottle = toThrottle(pwmThrottle);
        currBreak = toBreak(pwmThrottle);
        currSwitch = toBool(pwmSwitch);

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
    float RCController::toAngle(float aPWMVal){
        return 0.0;
    }
    float RCController::toThrottle(float aPWMVal){
        return 0.0;
    }
    float RCController::toBreak(float aPWMVal){
        return 0.0;
    }
    bool RCController::toBool(float aPWMVal){
        return false;
    }
}

}