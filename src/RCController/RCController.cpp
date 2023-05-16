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
CAN CAN_1(CAN1_RX, CAN1_TX, CAN1_BAUDRATE);

namespace tritonai {
namespace gkc {
RCController::RCController() {

  cont_p = new Controller();
  cont_p->deactivate_controller();

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

    CANMessage cMsg;
    if(currThrottle == 0) {
      uint8_t message[4] = {2, 0, 0, 0};
    } else {
      uint8_t message[4] = {3, 0, 0, 0};
    }
    cMsg = CANMessage(LIGHT_CAN_ID, &message[0], sizeof(message), 
                        CANData, CANExtended);
    if(!CAN1.send(msg)) {
      CAN1.reset();
      CAN_1.frequency(CAN1_BAUDRATE);
    }

    if (emoOn) {
      emoCounter++;
      CANMessage cMsg;
      uint8_t message[4] = {0, 0, 0, 0};
      cMsg = CANMessage(LIGHT_CAN_ID, &message[0], sizeof(message), 
                        CANData, CANExtended);
      if(!CAN1.send(msg)) {
        CAN1.reset();
        CAN_1.frequency(CAN1_BAUDRATE);
      }
    } 
    else {
      emoCounter = 0;
      CANMessage cMsg;
      uint8_t message[4] = {1, 0, 0, 0};
      cMsg = CANMessage(LIGHT_CAN_ID, &message[0], sizeof(message), 
                        CANData, CANExtended);
      if(!CAN1.send(msg)) {
        CAN1.reset();
        CAN_1.frequency(CAN1_BAUDRATE);
      }
    }
  } else {
    noMsgCounter++;
  }

  if (emoCounter > 10 || noMsgCounter > 100) {
    currThrottle = 0;
    currSteer = 0;
    cont_p->deactivate_controller();
    cont_p->set_actuation_values(currSteer, currThrottle, currBreak);

  } else if (remoteControl) {
    cont_p->deactivate_controller();
    cont_p->set_actuation_values(currSteer, currThrottle, currBreak);
    tempBool = true;

  } else {
    if (tempBool) {
      currThrottle = 0;
      currSteer = 0;
      cont_p->set_actuation_values(currSteer, currThrottle, currBreak);
      tempBool = false;
    }
    cont_p->activate_controller();
  }
}
} // namespace gkc
} // namespace tritonai