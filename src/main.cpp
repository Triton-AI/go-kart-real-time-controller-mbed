// Mbed
#include "PinNames.h"
#include "mbed.h"

// Project specific
#include "Controller/controller.hpp"

InterruptIn button(BUTTON1);

int main() {
  button.rise(&NVIC_SystemReset);

  auto cont_p = new tritonai::gkc::Controller();
  while (1){
    ThisThread::sleep_for(1000ms);
  }
    ;

    // hello
}