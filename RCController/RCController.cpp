/**
 * @file RCController.cpp
 * @author Jesus Fausto (jvfausto@ucsd.edu)
 *
 *
 *
 *@version 0.1
 * @date 2022-07-17
 *
 * @copyright Copyright 2022 Triton AI
 *
 *@brief This Header file is used for an implementation of an RC controller
 * to send signals controlling the Steering, Throttle and Braking of the EVGoKart
 * The RCController class reads these signals as a PWM input and maps the values to the actuators
 * using the set_acuation_values() function. The switch is always being checked and
 * if the switch is off, then the controller is deactivated.


 */
//Include RCController header file
#include "RCController.hpp"
//Include communication configuration header file
//config.hpp defines some communication parameters,Wa tchdog parameters and allocates throttle, braking and steering to specific CAN busses.
//Currently set to UART Serial
#include "config.hpp"
#include <iomanip>
PwmIn steer(Steer_Pin); 
PwmIn throttle(Throttle_Pin);
PwmIn red(Red_Pin);
elrc_receiver theReceiver(REMOTE_UART_RX_PIN, REMOTE_UART_TX_PIN);

namespace tritonai {
namespace gkc{

//Initalize RCController class
    RCController::RCController(){

        cont_p = new Controller();
        cont_p->deactivate_controller();
        noMsgCounter = 0;
        //std::cout << "Initializing RCController class" << std::endl;
        currSteer = 0;
        currThrottle = 0;
        currBreak = 0;
        sensor_write.call_every(50ms, this, &RCController::getSensor);
        sensor_write.dispatch_forever();
    }
    //Gets PWM
    void RCController::getSensor(){
        bool isRC = true;
        time_t secondsOG = time(NULL);
        //while(1!=0){
        bool emoOn;
        bool whoControlls;
        //std::cout << "bad luck" << std::endl;
        if(theReceiver.gatherData()){
            // std::cout << "Gathering" << std::endl;
            currThrottle = Map.Throttle(theReceiver.busData()[throttlePad]);
            currSteer = Map.Steering(theReceiver.busData()[steerPad]);
            emoOn = Map.emoVal(theReceiver.busData()[emoPadLeft], 
                theReceiver.busData()[emoPadRight], theReceiver.busData()[rightTriSwitch]);
            whoControlls = Map.whoControlls(theReceiver.busData()[rightTriSwitch]);
            //cont_p->deactivate_controller();
            //cont_p->activate_controller();
            noMsgCounter = 0;
        }
        else {
            // std::cout << "Not Gathering" << std::endl;
            noMsgCounter++;
        }
        
        if(emoOn || noMsgCounter > 10) {
            currThrottle = 0;
            currSteer = 0;
        };
        // std::cout << std::setprecision(2) << currThrottle << " " << std::setprecision(2) << currSteer << " " << emoOn << std::endl;

        cont_p->set_actuation_values(currSteer, currThrottle, currBreak);
    }
}

}
