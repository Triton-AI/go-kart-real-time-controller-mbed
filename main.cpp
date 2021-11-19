#include "mbed.h"
#include "QEI.hpp"
#include <iostream>
#include <string>

// main() runs in its own thread in the OS
int main()
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

