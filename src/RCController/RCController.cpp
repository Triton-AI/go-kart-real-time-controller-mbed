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
#include <iomanip>
/*
PwmIn steer(Steer_Pin);
PwmIn throttle(Throttle_Pin);
PwmIn red(Red_Pin);
*/
elrc_receiver theReceiver(REMOTE_UART_RX_PIN, REMOTE_UART_TX_PIN);

namespace tritonai {
namespace gkc {
RCController::RCController() {

  cont_p = new Controller();
  cont_p->deactivate_controller();
  noMsgCounter = 0;
  emoCounter = 0;

  currSteer = 0;
  currThrottle = 0;
  currBreak = 0;
  remoteControls = false;
  sensor_write.call_every(50ms, this, &RCController::getSensor);
  sensor_write.dispatch_forever();
}

void RCController::getSensor() {
  bool isRC = true;
  time_t secondsOG = time(NULL);
  bool emoOn = false;
  bool whoControlls = false;

  if (theReceiver.gatherData()) {
    currThrottle = Map.Throttle(theReceiver.busData()[throttlePad]);
    currSteer = Map.Steering(theReceiver.busData()[steerPad]);
    emoOn = Map.emoVal(theReceiver.busData()[emoPadLeft],
                       theReceiver.busData()[emoPadRight],
                       theReceiver.busData()[rightTriSwitch]);
    whoControlls = Map.whoControlls(theReceiver.busData()[rightTriSwitch]);
    noMsgCounter = 0;

    if (emoOn)
      emoCounter++;
    else
      emoCounter = 0;
  } else {
    noMsgCounter++;
  }

  // if (emoCounter > 10 || noMsgCounter > 100) {
  //   std::cout << "Deactivated\n";
  //   currThrottle = 0;
  //   cont_p->deactivate_controller();
  //   cont_p->set_actuation_values(currSteer, currThrottle, currBreak);

  // } else  /* if (whoControlls) */ {
   //  if (remoteControls) {
      cont_p->deactivate_controller();
    // }

    remoteControls = false;
    cont_p->set_actuation_values(currSteer, currThrottle, currBreak);

  //} 
  // else {
  //   if (remoteControls == false) {
  //     cont_p->activate_controller();
  //   }

  //   remoteControls = true;
  // }
}
} // namespace gkc
} // namespace tritonai