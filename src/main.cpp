// Mbed
#include "mbed.h"
#include "PinNames.h"
#include "ThisThread.h"

// Project specific
#include "Controller/controller.hpp"
#include "RCController/RCController.hpp"

using tritonai::gkc::RCController;
InterruptIn button(BUTTON1);

int main() {
  button.rise(&NVIC_SystemReset);
  new RCController();
  while(1);
}
