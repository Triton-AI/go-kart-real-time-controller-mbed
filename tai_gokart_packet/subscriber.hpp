#ifndef MY_GKC_PACKET_SUBSCRIBER
#define MY_GKC_PACKET_SUBSCRIBER


#include "mbed.h"
#include "include/tai_gokart_packet/gkc_packets.hpp"
#include "include/tai_gokart_packet/gkc_packet_factory.hpp"
#include "include/tai_gokart_packet/gkc_packet_subscriber.hpp"
#include "../../Actuation/actuation_controller.hpp"

using namespace tritonai::gkc;

class myGkcPacketSubscriber: public GkcPacketSubscriber
{
public:
    void packet_callback(const ControlGkcPacket & packet)
    {
        printf("%f %f %f\n", packet.throttle, packet.steering, packet.brake); //to be removed
        act->set_throttle(packet.throttle);
        act->set_steering(packet.steering);
        act->set_brake(packet.brake);
    }
    void packet_callback(const Handshake1GkcPacket & packet) {;}
    void packet_callback(const Handshake2GkcPacket & packet) {;}
    void packet_callback(const GetFirmwareVersionGkcPacket & packet) {;}
    void packet_callback(const FirmwareVersionGkcPacket & packet) {;}
    void packet_callback(const ResetMcuGkcPacket & packet)  {;}
    void packet_callback(const HeartbeatGkcPacket & packet) {;}
    void packet_callback(const ConfigGkcPacket & packet) {;}
    void packet_callback(const StateTransitionGkcPacket & packet) {;}

    void packet_callback(const SensorGkcPacket & packet) {;}
    void packet_callback(const Shutdown1GkcPacket & packet) {;}
    void packet_callback(const Shutdown2GkcPacket & packet) {;}
    void packet_callback(const LogPacket & packet) {;}

    void set_actuator(Actuator  *act)
    {
        this->act = act;
    }

private:
    Actuator *act;
};

#endif
