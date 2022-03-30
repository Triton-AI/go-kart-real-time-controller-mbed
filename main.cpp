#include "mbed.h"
//#include "QEI.hpp"
#include <iostream>
#include <string>
#include "Actuation/actuation_controller.hpp"
#include "tai_gokart_packet/include/tai_gokart_packet/gkc_packet_factory.hpp"
#include "tai_gokart_packet/include/tai_gokart_packet/gkc_packet_subscriber.hpp"
#include "tai_gokart_packet/subscriber.hpp"




extern BufferedSerial rs232; //TX, Rx



//PwmOut throttle_pin(THROTTLE_PWM_PIN);


Actuator act;
Thread thread_comm;


void thread_comm_function()
{
    auto packet_sub = myGkcPacketSubscriber();
    packet_sub.set_actuator(&act);
    auto factory = tritonai::gkc::GkcPacketFactory(&packet_sub, tritonai::gkc::GkcPacketUtils::debug_cout);
    int bytes_read;

    /*std::vector<uint8_t> recv_buffer(1, 0);
    while (1)
    {
        bytes_read = rs232.read(recv_buffer.data(), 1); //it reads one byte at a time
        if (bytes_read < 0)
            ;//error
        factory.Receive(recv_buffer);
    }*/
    std::vector<uint8_t> recv_buffer(2, 0);
    while (1)
    {
        for (int i = 0; i < 2; i++)
        {
            bytes_read = rs232.read(&(recv_buffer.data()[i]), 1); //it reads one byte at a 
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

