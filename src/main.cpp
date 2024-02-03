// Mbed
#include "PinNames.h"
#include "mbed.h"

// Project specific
#include "Controller/controller.hpp"
#include "RCController/RCController.hpp"

using tritonai::gkc::RCController;
InterruptIn button(BUTTON1);

int main() {
  button.rise(&NVIC_SystemReset);

  new RCController();
  while (1)
    ;

    // hello
}