#include "mbed.h"
//#include "QEI.hpp"
#include <iostream>
#include <string>
#include "Actuation/actuation_controller.hpp"
#include "tai_gokart_packet/include/tai_gokart_packet/gkc_packet_factory.hpp"
#include "tai_gokart_packet/include/tai_gokart_packet/gkc_packet_subscriber.hpp"
#include "tai_gokart_packet/subscriber.hpp"
//#include "peripherals.h"




//Onboard LEDs
//DigitalOut led1(LED1);
//DigitalOut led2(LED2);
//DigitalOut led3(LED3);

//CAN ports configured to 500Khz
//can1 used for the comm with the brake so far
//CAN can1(PD_0, PD_1, 500000); //RX, TX
//CAN can2(PB_5, PB_6, 500000); //RX, TX

//rs232 port
//used for the MRC (Main Robot Computer) - RTC comm
//confugured at 9600 bauds, but other frecuencies should work too
BufferedSerial rs232(PC_12, PD_2, 9600); //TX, Rx

//pwm port _____ used connected to the throttle
//It will output a pwm at frecuency ____
//The function Actuator::update_throttle() will update the duty cycle to the desired one
#define THROTTLE_PWM_PIN PA_5
//PwmOut throttle_pin(THROTTLE_PWM_PIN);


Actuator act;
Thread thread_comm;


void thread_comm_function()
{
    auto packet_sub = myGkcPacketSubscriber();
    packet_sub.set_actuator(&act);
    auto factory = tritonai::gkc::GkcPacketFactory(&packet_sub, tritonai::gkc::GkcPacketUtils::debug_cout);
    int bytes_read;
    std::vector<uint8_t> recv_buffer(18, 0);
    while (1)
    {
        for (int i = 0; i < 18; i++)
        {
            bytes_read = rs232.read(&(recv_buffer.data()[i % 18]), 1); //it reads one byte at a 
            if (bytes_read < 0)
                ;//error
        }
        factory.Receive(recv_buffer);
    }
}

int main()
{
    thread_comm.start(thread_comm_function);
    while (1)
        ;
}

