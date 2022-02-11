#include "mbed.h"
//#include "QEI.hpp"
#include <iostream>
#include <string>
#include "Actuation/actuation_controller.hpp"
#include "tai_gokart_packet/include/tai_gokart_packet/gkc_packet_factory.hpp"
#include "tai_gokart_packet/include/tai_gokart_packet/gkc_packet_subscriber.hpp"
#include "tai_gokart_packet/subscriber.hpp"

Actuator act;

BufferedSerial rs232(PC_12, PD_2, 9600);
int main()
{
    /*QEI* encoder = new QEI(PC_6, PC_8, NC, 1200);
    DigitalIn pt1(PC_8, PullUp); 
    DigitalIn pt2(PC_6, PullUp);    
    while (true) {
        float vel = encoder->getVelocity();
        printf("Vel: %f\n", vel);
        ThisThread::sleep_for(50ms);
    }*/
    auto packet_sub = myGkcPacketSubscriber();
    packet_sub.set_actuator(&act);
    auto factory = tritonai::gkc::GkcPacketFactory(&packet_sub, tritonai::gkc::GkcPacketUtils::debug_cout);
    int bytes_read = 1;
    std::vector<uint8_t> recv_buffer(18, 0);
    while (bytes_read)
    {
        rs232.read(recv_buffer.data(), 18);
        factory.Receive(recv_buffer);
    }
}

