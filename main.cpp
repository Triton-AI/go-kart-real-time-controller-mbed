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
#include "RCController.hpp"
#include "mbed.h"

using tritonai::gkc::RCController;
InterruptIn button(BUTTON1);

int main() {
//   std::cout << "[Triton AI Go-Kart Real-Time Controller]" << std::endl;
//   std::cout << "[Copyright Triton AI 2022]" << std::endl;
  button.rise(&NVIC_SystemReset);
  auto controller = new RCController();
  while(1);
}
