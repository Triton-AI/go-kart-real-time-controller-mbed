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
//Include RCController header file
#include "RCController.hpp"
//Include communication configuration header file
//config.hpp defines some communication parameters,Watchdog parameters and allocates throttle, braking and steering to specific CAN busses.
//Currently set to UART Serial
#include "config.hpp"

//Define PWM inputs for signals from steering, throttle and duty cycle
PwmIn steer(Steer_Pin); 
PwmIn throttle(Throttle_Pin);
PwmIn red(Red_Pin);

namespace tritonai {
namespace gkc{

//Initalize RCController class
    RCController::RCController(){

        cont_p = new Controller();
        cont_p->deactivate_controller();
        //std::cout << "Initializing RCController class" << std::endl;
        sensor_write.start(callback(this,&RCController::getSensor));
        rolling_average = 0;
    }
    //Gets PWM 
    void RCController::getSensor(){
        bool isRC = true;
        time_t secondsOG = time(NULL);
        float is_pos = 0;
        while(1!=0){
        float currSteer, currThrottle, currBreak = 0;
        bool currSwitch;

        // std::cout << currSteer << std::endl;
        //std::cout << currThrottle << std::endl;
        // std::cout << currBreak << std::endl;
        float pwmSteer = steer.dutycycle();
        float pwmThrottle = throttle.dutycycle();
        float pwmSwitch = red.dutycycle();
        
        pwmSteer = pwmSteer*.5+rolling_average*.5;
        rolling_average = pwmSteer;
        if(time(NULL) - secondsOG > 5){
            secondsOG = time(NULL);
            is_pos++;
            if(is_pos > 1)
                is_pos-=3;
        }
        currSteer = 20*is_pos;//Map.Steering(pwmSteer);
        std::cout << currSteer << std::endl;
        currThrottle = 0;//Map.Trigger(pwmThrottle);
        //currBreak = toBreak(pwmThrottle);
        currSwitch = 0;//Map.Red(pwmSwitch);
        //std::cout << pwmSwitch << std::endl;
        //std::cout << currSteer << std::endl;
        //std::cout << currThrottle << std::endl;
        //std::cout << currSwitch << std::endl << std::endl;
       
        if(1){//currSwitch == 1){
            if(isRC == false){
                //std::cout << "deactivated " << isRC << std::endl;
                cont_p->deactivate_controller();
                isRC = true;
            }
            //std::cout << "ready to run " << std::endl;
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