// /**
//  * @file main.cpp
//  * @author Haoru Xue (haoru.xue@autoware.org)
//  * @brief
//  * @version 0.1
//  * @date 2022-04-10
//  *
//  * @copyright Copyright 2022 Triton AI
//  *
//  */

// #include <chrono>
// #include <iostream>
// #include <memory>
// #include <string>

// #include "PinNames.h"
#include "ThisThread.h"
#include "controller.hpp"
#include "RCController.hpp"
// #include "mbed.h"

using tritonai::gkc::RCController;
// InterruptIn button(BUTTON1);

// int main() {
// //   std::cout << "[Triton AI Go-Kart Real-Time Controller]" << std::endl;
// //   std::cout << "[Copyright Triton AI 2022]" << std::endl;
//   button.rise(&NVIC_SystemReset);
//   auto controller = new RCController();
//   while(1);
// }


#include "InterfaceCAN.h"
#include <iostream>
#include "mbed.h"


#define VESC_RPM_EXTENDED_ID 0x03
#define VESC_CURRENT_EXTENDED_ID 0x01
#define VESC_POSITION_EXTENDED_ID 4
#define VESC_STATUS_EXTENDED_ID_PACKET_1 0x09
#define VESC_STATUS_EXTENDED_ID_PACKET_4 0x10


DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
InterruptIn button(PC_13);


#define CAN1_RX PD_0    // RX port for CAN1
#define CAN1_TX PD_1    // TX port for 
#define VESC_ID 125

CAN can_demo(CAN1_RX, CAN1_TX, 500000);

enum demo_mode 
{   nothing = 0,
    current = 1, 
    position_0 = 2, 
    position_180 = 3,
    rc = 4
};

static int current_state = demo_mode::nothing;


static constexpr uint32_t VESC_CURRENT_ID(const uint32_t &vesc_id)
{
  return (static_cast<uint32_t>(VESC_CURRENT_EXTENDED_ID)
          << sizeof(uint8_t) * 8) |
         static_cast<uint32_t>(vesc_id);
}

static constexpr uint32_t VESC_POSITION_ID(const uint32_t &vesc_id)
{
  return (static_cast<uint32_t>(VESC_POSITION_EXTENDED_ID)
          << sizeof(uint8_t) * 8) |
         static_cast<uint32_t>(vesc_id);
}

void onButtonAction(){
    int prev_press = 0;
    int new_press = 0;

    new_press = button;

    if(new_press == 1 && prev_press == 0) {
        // if(current_state == demo_mode::rc)
        //     current_state = demo_mode::nothing;
        // else
            current_state++;
    }

    switch(current_state) {
        case demo_mode::nothing:
            led1 = false;
            led2 = false;
            led3 = false;
            break;

        case demo_mode::current:
            led1 = true;
            led2 = false;
            led3 = false;
            break;

        case demo_mode::position_0:
            led1 = false;
            led2 = true;
            led3 = false;
            break;

        case demo_mode::position_180:
            led1 = false;
            led2 = false;
            led3 = true;
            break;

        case demo_mode::rc:
            led1 = true;
            led2 = true;
            led3 = true;
            break;
    }
}


int main() {
//   std::cout << "[Triton AI Go-Kart Real-Time Controller]" << std::endl;
//   std::cout << "[Copyright Triton AI 2022]" << std::endl;
//   button.rise(&NVIC_SystemReset);
//   auto controller = new RCController();
//   while(1);

 

    button.rise(onButtonAction);
    RCController *controller;

    Thread thread;

    while(1) {
        std::cout << current_state << "\n";
        
        uint8_t message[4] = {0, 0, 0, 0};
        auto data = 0;
        auto mode = 0;
        int32_t idx = 0;

        switch(current_state) {
            case demo_mode::nothing:
                break;
            case demo_mode::current:
                data = 2500;
                mode = VESC_CURRENT_ID(VESC_ID);
                break;

            case demo_mode::position_0:
                data = 0;
                mode = VESC_POSITION_ID(VESC_ID);
                break;

            case demo_mode::position_180:
                data = 180 * 1000000;
                mode = VESC_POSITION_ID(VESC_ID);
                break;

            case demo_mode::rc:
                controller = new RCController();
                break;

            default:
                std::cout << "No more states, press reset!";
                break;
        }
        
        message[(idx)++] = data >> 24;
        message[(idx)++] = data >> 16;
        message[(idx)++] = data >> 8;
        message[(idx)++] = data;
        
        CANMessage msg (mode, &message[0], sizeof(message), CANData, CANExtended);
        can_demo.write(msg);
    }
}