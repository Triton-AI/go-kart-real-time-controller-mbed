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
PwmIn steer(Steer_Pin); 
PwmIn throttle(Throttle_Pin);
PwmIn red(Red_Pin);

namespace tritonai {
namespace gkc{
    RCController::RCController(){

        cont_p = new Controller();
        cont_p->deactivate_controller();
        std::cout << "Initializing RCController class" << std::endl;
        sensor_write.start(callback(this,&RCController::getSensor));
        rolling_average = 0;
    }

    void RCController::getSensor(){
        bool isRC = true;

        while(1!=0){
        float currSteer, currThrottle, currBreak = 0;
        bool currSwitch;
        // std::cout << currSteer << std::endl;
        // std::cout << currThrottle << std::endl;
        // std::cout << currBreak << std::endl;
        float pwmSteer = steer.dutycycle();
        float pwmThrottle = throttle.dutycycle();
        float pwmSwitch = red.dutycycle();
        
        pwmSteer = pwmSteer*.5+rolling_average*.5;
        rolling_average = pwmSteer;
        currSteer = Map.Steering(pwmSteer);
        currThrottle = Map.Trigger(pwmThrottle);
        //currBreak = toBreak(pwmThrottle);
        currSwitch = Map.Red(pwmSwitch);
        //std::cout << pwmSwitch << std::endl;
        // std::cout << currSteer << std::endl;
        // std::cout << currThrottle << std::endl;
        // std::cout << currSwitch << std::endl << std::endl;
       

        // for(int i =0 ; i < 5 ; i++)
        // {
        //     std::cout << "input ?" << std::endl;
        //     ThisThread::sleep_for(3000ms);
        //     pwmSwitch = red.dutycycle();
        //     currSwitch = Map.Red(pwmSwitch);
        //     std::cout << currSwitch << std::endl;

        
            if(currSwitch == 1){
                if(isRC == false){
                    std::cout << "deactivated " << isRC << std::endl;
                    cont_p->deactivate_controller();
                    isRC = true;
                }
                //std::cout << currSteer << ", " << currThrottle << ", " << currBreak << endl;
                cont_p->set_actuation_values(currSteer, currThrottle, currBreak);
                //std::cout << "switch should be  " << currSwitch << std::endl << std::endl;
                //std::cout << "hello world" << std::endl;

            }
            else{
                if(isRC){
                    cont_p->activate_controller();
                    isRC = false;
                }       
            }
            ThisThread::sleep_for(100ms);
            
        }
    }
}

}