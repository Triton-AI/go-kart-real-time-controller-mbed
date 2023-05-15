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

#include "Sensor/sensor_reader.hpp"
#include "tai_gokart_packet/gkc_packet_factory.hpp"
#include "tai_gokart_packet/gkc_packet_utils.hpp"
#include "tai_gokart_packet/gkc_packets.hpp"

#include "Actuation/actuation_controller.hpp"
#include "Comm/comm.hpp"
#include "Tools/logger.hpp"
#include "StateMachine/state_machine.hpp"
#include "Watchdog/watchable.hpp"
#include "Watchdog/watchdog.hpp"

namespace tritonai {
namespace gkc {

/**
 * @brief  The Controller class is responsible for managing the go kart (unless it is beeing controlled as RC
 * that RCcontroller is used). It has to handle the communication with the MRC (with the GkcPacketSubscriber).
 * It also handles the state machine that enables or disables the kart (GkcStateMachine).
 * To interact with the kart it has an ActuationController instance which takes care of interacting with the actuators.
 * The Controller class is responsible for performing a function when a specific command is received from the MRC.
 * 
 * The GkcPacketSubscriber class is a base class that provides functionality for receiving packets of various
 * types from the MRC. GkcPacketSubscriber is an abstract class so the Controller class implements the packet_callback
 * functions to define how to handle each type of packet received.
 * Each message type has its own `packet_callback()` function.
 * For example, `packet_callback(const ControlGkcPacket &packet)` responds to a control command from the MRC.
 *
 * The GkcStateMachine class is a base class that provides the state machine functionality for the Controller class.
 * Some packet_callback() functions trigger state changes to activate the kart for example.
* Or the estop interrups sets the sate to emergecy stop.
 * on_initialize() on_deactivate() on_activate() on_shutdown() on_emergency_stop() on_reinitialize() need to be defined
 * because Controller inheris from GkcStateMachine. This functions are called when the state is changed to
 * the corresponding state.
 *
 * When the car is used as an RC car, instead of communicating with the MRC the class RCController uses
 * the methods activate_controller() and set_actuation_values() to bypass the MRC messages needed to
 * activate the kart and give it commands.
 *
 * To send the sensor information to the MRC it has the sensor_poll_thread. To do so it has the SensorReader sensor_ instance
 * which connects to the ActuationController actuation_.
 * TODO: check why it inherits from ISensorProvider, I believe it is not used.
 *
 * send_log sends a LogPacket to the MRC. The terminal at the MRC will print the severity
 * and the message, and it can be configured to only show logs with severyty higher that a threshold.
 * It is quite usefull for debuguing as a "cout <<" will print the text on Mbed which actually takes quite long and could
 * mess up how the code works.
 */
class Controller : public GkcStateMachine,
                   public GkcPacketSubscriber,
                   public Watchable,
                   public ISensorProvider,
                   public ILogger {
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
  
  //This is to make rc controller work bypassing the state machine
  void deactivate_controller();
  void set_actuation_values(float steerVal, float throttleVal, float breakVal);
  void activate_controller();

  // ISensorProvider API
  virtual bool is_ready();
  virtual void populate_reading(SensorGkcPacket &pkt);

  void send_log(const LogPacket::Severity &severity, const std::string &what);

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
  Thread state_poll_thread;

  InterruptIn estop_interrupt;


  void watchdog_callback();
  void initialize_thread_callback();
  void heartbeat_thread_callback();
  void poll_state();
  void sensor_poll_thread_callback();
  void estop_interrupt_callback();

  // GkcStateMachine API
  StateTransitionResult on_initialize(const GkcLifecycle &last_state);
  StateTransitionResult on_deactivate(const GkcLifecycle &last_state);
  StateTransitionResult on_activate(const GkcLifecycle &last_state);
  StateTransitionResult on_shutdown(const GkcLifecycle &last_state);
  StateTransitionResult on_emergency_stop(const GkcLifecycle &last_state);
  StateTransitionResult on_reinitialize(const GkcLifecycle &last_state);
};
} // namespace gkc
} // namespace tritonai
#endif // CONTROLLER_HPP_