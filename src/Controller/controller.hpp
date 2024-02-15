#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "Comm/comm.hpp"
#include "tai_gokart_packet/gkc_packet_subscriber.hpp"
#include "Watchdog/watchdog.hpp"
#include "Sensor/sensor_reader.hpp"
#include "Actuation/actuation_controller.hpp"
namespace tritonai::gkc
{
  class Controller :
    public GkcPacketSubscriber,
    public Watchable,
    public ILogger
  {
    public:
      Controller();

      void watchdog_callback();
      void keep_alive();

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

      void send_log(const LogPacket::Severity &severity, 
                    const std::string &what) override;

    private:
      CommManager _comm;
      Watchdog _watchdog;
      SensorReader _sensor_reader;
      ActuationController _actuation;

      Thread _keep_alive_thread;
  };
}

#endif // CONTROLLER_HPP