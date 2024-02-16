#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "Comm/comm.hpp"
#include "tai_gokart_packet/gkc_packet_subscriber.hpp"
#include "Watchdog/watchdog.hpp"
#include "Sensor/sensor_reader.hpp"
#include "Actuation/actuation_controller.hpp"
#include "RCController/RCController.hpp"
#include "StateMachine/state_machine.hpp"

namespace tritonai::gkc
{
  class Controller :
    public GkcPacketSubscriber,
    public Watchable,
    public ILogger,
    public GkcStateMachine
  {
    public:
      Controller();

      // For testing purposes
      void keep_alive();

    protected:
      // GkcPacketSubscriber API
      void packet_callback(const Handshake1GkcPacket & packet);
      void packet_callback(const Handshake2GkcPacket & packet);
      void packet_callback(const GetFirmwareVersionGkcPacket & packet);
      void packet_callback(const FirmwareVersionGkcPacket & packet);
      void packet_callback(const ResetRTCGkcPacket & packet);
      void packet_callback(const HeartbeatGkcPacket & packet);
      void packet_callback(const ConfigGkcPacket & packet);
      void packet_callback(const StateTransitionGkcPacket & packet);
      void packet_callback(const ControlGkcPacket & packet);
      void packet_callback(const SensorGkcPacket & packet);
      void packet_callback(const Shutdown1GkcPacket & packet);
      void packet_callback(const Shutdown2GkcPacket & packet);
      void packet_callback(const LogPacket & packet);
      void packet_callback(const RCControlGkcPacket & packet);

      // ILogger API
      void send_log(const LogPacket::Severity &severity, 
                    const std::string &what) override;
      LogPacket::Severity _severity;

      // Watchable API
      void watchdog_callback();

      // GkcStateMachine API
      StateTransitionResult on_initialize(const GkcLifecycle &last_state) override;
      StateTransitionResult on_deactivate(const GkcLifecycle &last_state) override;
      StateTransitionResult on_activate(const GkcLifecycle &last_state) override;
      StateTransitionResult on_emergency_stop(const GkcLifecycle &last_state) override;
      StateTransitionResult on_reinitialize(const GkcLifecycle &last_state) override;

    private:
      CommManager _comm;
      Watchdog _watchdog;
      SensorReader _sensor_reader;
      ActuationController _actuation;
      RCController _rc_controller;

      Thread _keep_alive_thread{osPriorityNormal, OS_STACK_SIZE, nullptr, "keep_alive_thread"};

      bool _rc_commanding{false};
  };
}

#endif // CONTROLLER_HPP