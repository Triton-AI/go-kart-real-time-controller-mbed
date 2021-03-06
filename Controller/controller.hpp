/**
 * @file controller.hpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-02-10
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <memory>

#include "mbed.h"

#include "tai_gokart_packet/gkc_packet_factory.hpp"
#include "tai_gokart_packet/gkc_packet_utils.hpp"
#include "tai_gokart_packet/gkc_packets.hpp"

#include "comm.hpp"
#include "state_machine.hpp"
#include "watchable.hpp"
#include "watchdog.hpp"
#include "actuation_controller.hpp"

namespace tritonai {
namespace gkc {
class Controller : public GkcStateMachine,
                   public GkcPacketSubscriber,
                   public Watchable {
public:
  Controller();

  // GkcPacketSubscriber API
  void packet_callback(const Handshake1GkcPacket &packet);
  void packet_callback(const Handshake2GkcPacket &packet);
  void packet_callback(const GetFirmwareVersionGkcPacket &packet);
  void packet_callback(const FirmwareVersionGkcPacket &packet);
  void packet_callback(const ResetMcuGkcPacket &packet);
  void packet_callback(const HeartbeatGkcPacket &packet);
  void packet_callback(const ConfigGkcPacket &packet);
  void packet_callback(const StateTransitionGkcPacket &packet);
  void packet_callback(const ControlGkcPacket &packet);
  void packet_callback(const SensorGkcPacket &packet);
  void packet_callback(const Shutdown1GkcPacket &packet);
  void packet_callback(const Shutdown2GkcPacket &packet);
  void packet_callback(const LogPacket &packet);

protected:
  CommManager comm_;
  ActuationController actuation_;
  SensorReader sensor_;
  ConfigGkcPacket::Configurables configs_;
  Watchable pc_hb_watcher_;
  Watchable ctl_cmd_watcher_;
  tritonai::gkc::Watchdog watchdog_;

  Thread initialize_thread;
  Thread heartbeat_thread;
  Thread sensor_poll_thread;

  void watchdog_callback();
  void initialize_thread_callback();
  void send_log(const LogPacket::Severity &severity, const std::string &what);
  void heartbeat_thread_callback();
  void sensor_poll_thread_callback();

  // GkcStateMachine API
  StateTransitionResult on_initialize(const GkcLifecycle &last_state);
  StateTransitionResult on_deactivate(const GkcLifecycle &last_state);
  StateTransitionResult on_activate(const GkcLifecycle &last_state);
  StateTransitionResult on_shutdown(const GkcLifecycle &last_state);
  StateTransitionResult on_emergency_stop(const GkcLifecycle &last_state);
  StateTransitionResult on_reinitialize(const GkcLifecycle &last_state);

  // Data
};
} // namespace gkc
} // namespace tritonai
#endif // CONTROLLER_HPP_