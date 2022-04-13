/**
 * @file main.cpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-04-10
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "PinNames.h"
#include "ThisThread.h"
#include "controller.hpp"
#include "mbed.h"
//h
using tritonai::gkc::Controller;
InterruptIn button(BUTTON1);

//The main doesn't do much, the main thing it does is creating the controller. The controller class has everything the RTC is.
int main() {
  std::cout << "[Triton AI Go-Kart Real-Time Controller]" << std::endl;
  std::cout << "[Copyright Triton AI 2022]" << std::endl;
  button.rise(&NVIC_SystemReset);
  //Create a new Controller. Controler has all the important things. Is is created with new which is like malloc in C. When creating it Controller::Controller() is called
  auto controller = new Controller();
  //Infinite loop, do nothig
  while (1)
    ;
}
