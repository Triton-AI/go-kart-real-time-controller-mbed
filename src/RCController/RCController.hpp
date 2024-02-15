#ifndef RC_CONTROLLER_HPP_
#define RC_CONTROLLER_HPP_

#include "config.hpp"
#include "math.h"
#include "elrs_receiver.hpp"
#include "tai_gokart_packet/gkc_packets.hpp"
#include "Tools/rc_control.hpp"
#include <Thread.h>

namespace tritonai::gkc
{

struct Translation
{
    double normalize(int analogValue);
    double steering(int steerVal);
    double throttle(int throttleVal);
    double brake(int brakeVal);
    bool is_active(int swith1, int swith2);
    AutonomyMode getAutonomyMode(int rightTriVal);
};

class RCController
{
    public:
    RCController();
    const RCControlGkcPacket& getPacket(){ 
        _is_ready = false;
        return _packet;
    }

    protected:
    void update();
    Translation Map;
    Thread _rc_thread{osPriorityNormal, OS_STACK_SIZE*2, nullptr, "rc_thread"};
    

    private:
    elrc_receiver _receiver;
    RCControlGkcPacket _packet;
    bool _is_ready;
    
};

} // namespace tritonai::gkc

#endif  // RC_CONTROLLER_HPP_