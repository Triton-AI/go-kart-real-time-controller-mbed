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

  //mbed::Watchdog &watchdog = mbed::Watchdog::get_instance();
  //watchdog.start(DEFAULT_ACTUATION_INTERVAL_MS);

  sensor_write.call_every(50ms, this, &RCController::getSensor);
  sensor_write.dispatch_forever();
}

void RCController::getSensor() {
  bool emoOn = false;

  bool tempBool = false;

  if (theReceiver.gatherData()) {
    currThrottle = Map.Throttle(theReceiver.busData()[throttlePad]);
    currSteer = Map.Steering(theReceiver.busData()[steerPad]);
    emoOn = Map.emoVal(theReceiver.busData()[emoPadLeft],
                       theReceiver.busData()[emoPadRight],
                       theReceiver.busData()[rightTriSwitch]);
    remoteControl = Map.whoControlls(theReceiver.busData()[rightTriSwitch]);
    noMsgCounter = 0;

    if (emoOn)
      emoCounter++;
    else
      emoCounter = 0;
  } else {
    noMsgCounter++;
  }

  if (emoOn || noMsgCounter > 100) {
    currThrottle = 0;
    currSteer = 0;
    cont_p->deactivate_controller();
    cont_p->set_actuation_values(currSteer, currThrottle, currBreak);

  } else if (remoteControl) {
    //mbed::Watchdog::get_instance().kick();
    cont_p->deactivate_controller();
    cont_p->set_actuation_values(currSteer, currThrottle, currBreak);
    tempBool = true;

  } else if (!remoteControl) {
    if (tempBool) {
      currThrottle = 0;
      currSteer = 0;
      cont_p->set_actuation_values(currSteer, currThrottle, currBreak);
      tempBool = false;
    }

    //mbed::Watchdog::get_instance().kick();
    cont_p->activate_controller();
  }
}
} // namespace gkc
} // namespace tritonai