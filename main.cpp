#include <iostream>
#include <string>

#include "PinNames.h"
#include "controller.hpp"
#include "mbed.h"

using tritonai::gkc::Controller;
InterruptIn button(BUTTON1);

int main() {
  std::cout << "[Triton AI Go-Kart Real-Time Controller]" << std::endl;
  std::cout << "[Copyright Triton AI 2022]" << std::endl;
  button.rise(&NVIC_SystemReset);
  Controller controller;
  while (1)
    ;
}
