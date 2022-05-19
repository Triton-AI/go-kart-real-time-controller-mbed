/**
 * @file controller.cpp
 * @author Jesus Fausto (jvfausto@ucsd.edu)
 * @brief
 * @version 0.1
 * @date 2022-07-17
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#include "controller.hpp"
#include "mbed.h"
#include "PwmIn.h"

namespace tritonai {
namespace gkc{

class RCController{
public:
    RCController(PwmIn* chan1, PwmIn* chan2, PwmIn* chan3);
    void getSensor();

private:
    float toAngle(float aPWMVal);
    float toThrottle(float aPWMVal);
    float toBreak(float aPWMVal);
    bool toBool(float aPWMVal);

    Controller* cont_p;
    PwmIn* steerVal;
    PwmIn* throttleVal;
    PwmIn* switchVal;

    Thread sensor_write;

    bool isRC;
};
}
}