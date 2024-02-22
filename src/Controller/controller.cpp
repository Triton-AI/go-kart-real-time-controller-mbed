#include "Controller/controller.hpp"
#include "config.h"

#include "tai_gokart_packet/gkc_packets.hpp"
#include "tai_gokart_packet/gkc_packet_utils.hpp"
#include "tai_gokart_packet/version.hpp"

#include <chrono>
#include <iostream>

namespace tritonai::gkc
{
  void Controller::agx_heartbeat()
  {
    HeartbeatGkcPacket packet;
    std::string state;
    std::string old_state;

    //TODO: (Moises) TEMP
    GkcStateMachine::initialize();
    //TODO: (Moises) TEMP
    
    while(1){
      ThisThread::sleep_for(std::chrono::milliseconds(100));
      packet.rolling_counter++;
      packet.state = get_state();
      _comm.send(packet); // Send the heartbeat packet
      this->inc_count(); // Increment the watchdog count for the controller

      
      switch(get_state())
      {
        case GkcLifecycle::Uninitialized:
          state = "Uninitialized";
          break;
        case GkcLifecycle::Initializing:
          state = "Initializing";
          break;
        case GkcLifecycle::Inactive:
          state = "Inactive";
          break;
        case GkcLifecycle::Active:
          state = "Active";
          break;
        case GkcLifecycle::Emergency:
          state = "Emergency";
          break;
        default:
          state = "Unknown";
          break;
    }

    if(state != old_state){
      old_state = state;
      send_log(LogPacket::Severity::WARNING, "Controller state: " + state);
    }
    }
  }

  void Controller::on_rc_disconnect()
  {
    send_log(LogPacket::Severity::INFO, "Controller heartbeat lost");

    if(get_state() != GkcLifecycle::Active)
    {
      send_log(LogPacket::Severity::INFO, "Controller is not active, ignoring RC controller heartbeat lost");
      return;
    }

    send_log(LogPacket::Severity::FATAL, "RC controller heartbeat lost");
    _actuation.set_throttle_cmd(new float(0.0));
    _actuation.set_steering_cmd(new float(0.0));
    _actuation.set_brake_cmd(new float(0.2));
    emergency_stop();
  }

  // Controller initialization
  Controller::Controller() :
    Watchable(DEFAULT_CONTROLLER_POLL_INTERVAL_MS, DEFAULT_CONTROLLER_POLL_LOST_TOLERANCE_MS, "Controller"), // Initializes the controller with default values
    GkcStateMachine(), // Initializes the state machine
    _severity(LogPacket::Severity::WARNING), // Initializes the severity of the logger
    _comm(this), // Passes the controller as the subscriber to the comm manager
    _watchdog(DEFAULT_WD_INTERVAL_MS, DEFAULT_WD_MAX_INACTIVITY_MS, DEFAULT_WD_WAKEUP_INTERVAL_MS), // Initializes the watchdog with default values
    _sensor_reader(), // Initializes the sensor reader
    _actuation(this), // Passes the controller as the logger to the actuation controller
    _rc_controller(this), // Passes the controller as the packet subscriber to the RC controller
    _rc_heartbeat(DEFAULT_RC_HEARTBEAT_INTERVAL_MS, DEFAULT_RC_HEARTBEAT_LOST_TOLERANCE_MS, "RCControllerHeartBeat") // Initializes the RC controller  heartbeat with default values
  {
    // Attaches the watchdog callback to the controller
    attach(callback(this, &Controller::watchdog_callback));

    _keep_alive_thread.start(callback(this, &Controller::agx_heartbeat));

    // Adds the all the objects to the watchlist
    _watchdog.add_to_watchlist(this); // Adds the controller to the watchlist
    _watchdog.add_to_watchlist(&_comm); // Adds the comm manager to the watchlist
    _watchdog.add_to_watchlist(&_sensor_reader); // Adds the sensor reader to the watchlist
    _watchdog.add_to_watchlist(&_rc_controller); // Adds the RC controller to the watchlist
    if(_stop_on_rc_disconnect){
      _rc_heartbeat.attach(callback(this, &Controller::on_rc_disconnect)); // Attaches the RC disconnect callback to rc heartbeat
      _watchdog.add_to_watchlist(&_rc_heartbeat); // Adds the RC heartbeat to the watchlist
    }

    send_log(LogPacket::Severity::INFO, "Controller initialized");
  }

  // Wathdog API IMPLEMENTATION
  void Controller::watchdog_callback()
  {
    send_log(LogPacket::Severity::FATAL, "Controller watchdog trigger");
    NVIC_SystemReset();
  }

  // ILogger API IMPLEMENTATION
  // TODO: (Moises) send_log partially implemented, complete the implementation
  void Controller::send_log(const LogPacket::Severity &severity, const std::string &what)
  {
    
    if(severity == LogPacket::Severity::FATAL && _severity <= severity)
      std::cerr << "Fatal: " << what << std::endl;
    else if(severity == LogPacket::Severity::ERROR && _severity <= severity)
      std::cerr << "Error: " << what << std::endl;
    else if(severity == LogPacket::Severity::WARNING && _severity <= severity)
      std::cerr << "Warning: " << what << std::endl;
    else if(severity == LogPacket::Severity::INFO && _severity <= severity)
      std::cout << "Info: " << what << std::endl;
    else if(severity == LogPacket::Severity::BEBUG && _severity <= severity)
      std::cout << "Debug: " << what << std::endl;

  }

  // PACKET CALLBACKS API IMPLEMENTATION
  void Controller::packet_callback(const Handshake1GkcPacket &packet)
  {
    send_log(LogPacket::Severity::INFO, "Handshake1GkcPacket received");
    Handshake2GkcPacket response;
    response.seq_number = packet.seq_number + 1;
    _comm.send(response);

  }

  void Controller::packet_callback(const Handshake2GkcPacket &packet)
  {
    send_log(LogPacket::Severity::INFO, "Handshake2GkcPacket received, should not happen, ignoring.");
  }

  void Controller::packet_callback(const GetFirmwareVersionGkcPacket &packet)
  {
    send_log(LogPacket::Severity::INFO, "GetFirmwareVersionGkcPacket received");
    FirmwareVersionGkcPacket response;
    response.major = GkcPacketLibVersion::MAJOR;
    response.minor = GkcPacketLibVersion::MINOR;
    response.patch = GkcPacketLibVersion::PATCH;

    _comm.send(response);
  }

  void Controller::packet_callback(const FirmwareVersionGkcPacket &packet)
  {
    send_log(LogPacket::Severity::INFO, "FirmwareVersionGkcPacket received, should not happen, ignoring.");
  }

  void Controller::packet_callback(const ResetRTCGkcPacket &packet)
  {
    send_log(LogPacket::Severity::FATAL, "ResetRTCGkcPacket received");
    NVIC_SystemReset();
  }

  // TODO: (Moises) Implement the heartbeat packet callback
  void Controller::packet_callback(const HeartbeatGkcPacket &packet)
  {
    send_log(LogPacket::Severity::INFO, "HeartbeatGkcPacket received");
  }

  // TODO: (Moises) Implement the config packet callback
  void Controller::packet_callback(const ConfigGkcPacket &packet)
  {
    std::cout << "ConfigGkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const StateTransitionGkcPacket &packet)
  {
    send_log(LogPacket::Severity::INFO, "StateTransitionGkcPacket received");

    switch(packet.requested_state)
    {
      case GkcLifecycle::Uninitialized:
        send_log(LogPacket::Severity::INFO, "Controller transitioning to Uninitialized");
        emergency_stop();
        break;
      case GkcLifecycle::Initializing:
        send_log(LogPacket::Severity::INFO, "Controller asked to transition to Initializing, ignoring");
        break;
      case GkcLifecycle::Inactive:
        send_log(LogPacket::Severity::INFO, "Controller transitioning to Inactive");
        if(get_state() == GkcLifecycle::Uninitialized)
          initialize();
        else if(get_state() == GkcLifecycle::Active)
          GkcStateMachine::deactivate();
        break;
      case GkcLifecycle::Active:
        send_log(LogPacket::Severity::INFO, "Controller transitioning to Active");
        GkcStateMachine::activate();
        break;
      case GkcLifecycle::Emergency:
        send_log(LogPacket::Severity::INFO, "Controller transitioning to Emergency");
        emergency_stop();
        break;
      default:
        send_log(LogPacket::Severity::ERROR, "Controller asked to transition to unknown state, ignoring");
        break;
    }
  }

  // TODO: (Moises) Implement the control packet callback, partially done
  void Controller::packet_callback(const ControlGkcPacket &packet)
  {
    if(get_state() != GkcLifecycle::Active){
      send_log(LogPacket::Severity::INFO, "Controller is not active, ignoring ControlGkcPacket");
      return;
    }

    if(_rc_commanding){
      send_log(LogPacket::Severity::WARNING, "RC is commanding, ignoring ControlGkcPacket");
      return;
    }

    send_log(LogPacket::Severity::INFO, "ControlGkcPacket received: throttle: " + std::to_string((int)(packet.throttle * 100)) + "%, " +
            "steering: " + std::to_string((int)(packet.steering * 100)) + "%, " +
            "brake: " + std::to_string((int)(packet.brake * 100)) + "%"
    );
    
    set_actuation_values(packet.throttle, packet.steering, packet.brake);
  }

  // TODO: (Moises) Implement the sensor packet callback
  void Controller::packet_callback(const SensorGkcPacket &packet)
  {
    send_log(LogPacket::Severity::INFO, "SensorGkcPacket received");
  }

  // TODO: (Moises) Implement the shutdown1 packet callbacks
  void Controller::packet_callback(const Shutdown1GkcPacket &packet)
  {
    send_log(LogPacket::Severity::INFO, "Shutdown1GkcPacket received");
  }

  // TODO: (Moises) Implement the shutdown2 packet callbacks
  void Controller::packet_callback(const Shutdown2GkcPacket &packet)
  {
    send_log(LogPacket::Severity::INFO, "Shutdown2GkcPacket received");
  }

  // TODO: (Moises) Implement the log packet callback
  void Controller::packet_callback(const LogPacket &packet)
  {
    send_log(packet.level, packet.what);
  }

  void Controller::packet_callback(const RCControlGkcPacket &packet)
  {
    _rc_heartbeat.inc_count(); // Increment the RC heartbeat count

    if (get_state() == GkcLifecycle::Uninitialized)
    {
      send_log(LogPacket::Severity::WARNING, "Controller is uninitialized, ignoring RCControlGkcPacket");
      return;
    }


    if(!packet.is_active && get_state() != GkcLifecycle::Inactive){
      send_log(LogPacket::Severity::FATAL, "RCControlGkcPacket is not active, calling emergency_stop()");
      set_actuation_values(0.0, 0.0, 0.2); // Set the actuation values to stop the car (brake at 20% pressure
      emergency_stop();
      return;
    }

    if(packet.is_active && get_state() == GkcLifecycle::Inactive){
      send_log(LogPacket::Severity::INFO, "Controller transitioning to Active");
      GkcStateMachine::activate();
      return;
    }

    if(packet.autonomy_mode == AUTONOMOUS){
        _rc_commanding = false; // Clear the RC commanding flag
        send_log(LogPacket::Severity::INFO, "RCControlGkcPacket is in autonomous mode, ignoring");
        return;
      }

    // If throttle, steering, and brake are zero, then the RC is not commanding
    if(packet.throttle == 0.0 && packet.steering == 0.0 && packet.brake == 0.0){
      _rc_commanding = packet.autonomy_mode == AutonomyMode::MANUAL; // Set the RC commanding flag
      if(!_rc_commanding)
        return;
    }

    if(packet.autonomy_mode == AutonomyMode::AUTONOMOUS_OVERRIDE || packet.autonomy_mode == AutonomyMode::MANUAL){
      _rc_commanding = true; // Set the RC commanding flag
    }

    send_log(LogPacket::Severity::INFO, 
            "RCControlGkcPacket received: throttle: " + std::to_string((int)(packet.throttle * 100)) + "%, " +
            "steering: " + std::to_string((int)(packet.steering * 100)) + "%, " +
            "brake: " + std::to_string((int)(packet.brake * 100)) + "%, " +
            "autonomy_mode: " + std::to_string(packet.autonomy_mode) + ", " +
            "is_active: " + std::to_string(packet.is_active)
    );

    // No problems detected, setting the actuation commands
    set_actuation_values(packet.throttle, packet.steering, packet.brake);

    if(packet.autonomy_mode == AutonomyMode::AUTONOMOUS || packet.autonomy_mode == AutonomyMode::AUTONOMOUS_OVERRIDE)
          _rc_commanding = false; // Clear the RC commanding flag

  }

  // GkcStateMachine API IMPLEMENTATION
  // TODO: (Moises) Implement on_initialize
  StateTransitionResult Controller::on_initialize(const GkcLifecycle &last_state)
  {
    send_log(LogPacket::Severity::INFO, "Controller initializing");
    _watchdog.arm(); // Arms the watchdog
    set_actuation_values(0.0, 0.0, 0.2); // Set the actuation values to stop the car (brake at 20% pressure
    return StateTransitionResult::SUCCESS;
  }

  // TODO: (Moises) Implement on_deactivate
  StateTransitionResult Controller::on_deactivate(const GkcLifecycle &last_state)
  {
    send_log(LogPacket::Severity::INFO, "Controller deactivating");
    return StateTransitionResult::SUCCESS;
  }

  // TODO: (Moises) Implement on_activate
  StateTransitionResult Controller::on_activate(const GkcLifecycle &last_state)
  {
    send_log(LogPacket::Severity::INFO, "Controller activating");
    return StateTransitionResult::SUCCESS;
  }

  // TODO: (Moises) Implement on_emergency_stop
  StateTransitionResult Controller::on_emergency_stop(const GkcLifecycle &last_state)
  {
    send_log(LogPacket::Severity::INFO, "Controller emergency stopping");
    set_actuation_values(0.0, 0.0, 0.2); // Set the actuation values to stop the car (brake at 20% pressure
    return StateTransitionResult::SUCCESS;
  }

  // TODO: (Moises) Implement on_reinitialize
  StateTransitionResult Controller::on_reinitialize(const GkcLifecycle &last_state)
  {
    send_log(LogPacket::Severity::INFO, "Controller reinitializing");
    set_actuation_values(0.0, 0.0, 0.2); // Set the actuation values to stop the car (brake at 20% pressure
    return StateTransitionResult::SUCCESS;
  }

  void Controller::set_actuation_values(float throttle, float steering, float brake)
  {
    if(get_state() != GkcLifecycle::Active){
      send_log(LogPacket::Severity::INFO, "Controller is not active, ignoring set_actuation_values");
      _actuation.set_throttle_cmd(new float(0.0));
      _actuation.set_steering_cmd(new float(0.0));
      _actuation.set_brake_cmd(new float(brake));
      return;
    }
    _actuation.set_throttle_cmd(new float(throttle));
    _actuation.set_steering_cmd(new float(steering));
    _actuation.set_brake_cmd(new float(brake));
  }
} // namespace tritonai::gkc