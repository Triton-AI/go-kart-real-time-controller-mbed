#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "Comm/comm.hpp"
#include "tai_gokart_packet/gkc_packet_subscriber.hpp"
namespace tritonai::gkc
{
  class Controller :
    public GkcPacketSubscriber
  {
    public:
      Controller();

    protected:
      void packet_callback(const Handshake1GkcPacket & packet);
      void packet_callback(const Handshake2GkcPacket & packet);
      void packet_callback(const GetFirmwareVersionGkcPacket & packet);
      void packet_callback(const FirmwareVersionGkcPacket & packet);
      void packet_callback(const ResetMcuGkcPacket & packet);
      void packet_callback(const HeartbeatGkcPacket & packet);
      void packet_callback(const ConfigGkcPacket & packet);
      void packet_callback(const StateTransitionGkcPacket & packet);
      void packet_callback(const ControlGkcPacket & packet);
      void packet_callback(const SensorGkcPacket & packet);
      void packet_callback(const Shutdown1GkcPacket & packet);
      void packet_callback(const Shutdown2GkcPacket & packet);
      void packet_callback(const LogPacket & packet);

    private:
      CommManager _comm;
  };
}

#endif // CONTROLLER_HPP