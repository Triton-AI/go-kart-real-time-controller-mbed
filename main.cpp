#include <iostream>
#include <string>

#include "mbed.h"

#include "QEI.hpp"
#include "watchdog.hpp"

void test_encoder()
{
    QEI* encoder = new QEI(PC_6, PC_8, NC, 1200);
    DigitalIn pt1(PC_8, PullUp); 
    DigitalIn pt2(PC_6, PullUp);    
    while (true) {
        float vel = encoder->getVelocity();
        printf("Vel: %f\n", vel);
        ThisThread::sleep_for(50ms);
    }
}

void test_watchdog()
{
    tritonai::gkc::Watchdog watchdog(5, 20, 5);
    watchdog.arm();
    // Add your own watchable object here
    // Then add your object to the watchdog's watchlist
    // Then do some magic to trigger the watchdog
    watchdog.disarm();
}

// main() runs in its own thread in the OS
int main()
{
    // test_encoder();
    test_watchdog();
}
