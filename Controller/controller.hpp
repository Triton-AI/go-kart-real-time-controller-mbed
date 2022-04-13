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

//This file is the header file for the class Controller. It doesn't define any function, it only declares what the class Controller has. It doesnt define any function/method, it only states they exist.

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
  /*
  This is the declaration of Controller. It inheritates from GkcStateMachine, GkcPacketSubscriber and Watchable.
  That means Controller will have evrything those classes have. We have all the functions/methods those classes have but we also 'promise' to define the behaviour of some functions. Those functions are the ones declared as virtual on the original class.
  */
class Controller : public GkcStateMachine,      //If has all the tools for the state machine. More info about it on https://github.com/Triton-AI/go-kart-real-time-controller-mbed/blob/master/Design/state_machine.md
                   public GkcPacketSubscriber,  //It has to do with the communication library.
                   public Watchable  {          //for the watchdog
public:
  Controller();

  // This functions that we 'promise' to define for the GkcPacketSubscriber API
  // Whenever a packet is read one of these functions is ejecuted
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
  //This class will take care of reading the sensors (besides the sensors needed for the acutators)
  SensorReader sensor_;
  //It has the configuration parameters. It has some default values, and it is overwritten if we receive a configuration packet
  ConfigGkcPacket::Configurables configs_;
  //Watchdog fot the hartbeat. IT 'watched' the harbeat packets. On the communication protocol there is one type of packet which only porpuse is only checking comm is working. This checks this type of packet.
  Watchable pc_hb_watcher_;
  //Wachdog for the controll packets. Checks if we stop reveiving istructions on how to move from the main comouter.
  Watchable ctl_cmd_watcher_;
  //?
  tritonai::gkc::Watchdog watchdog_;

  //?
  Thread initialize_thread;
  //This thread sends hearbeat packets to the main computer
  Thread heartbeat_thread;
  void heartbeat_thread_callback();
  //This thread sends sensor packets to the main computer
  Thread sensor_poll_thread;
  void sensor_poll_thread_callback();

  void watchdog_callback();
  void initialize_thread_callback();
  void send_log(const LogPacket::Severity &severity, const std::string &what);
 

  // This functions that we 'promise' to define for the GkcStateMachine API
  // Whenever the state machine changes state these functions are called.
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