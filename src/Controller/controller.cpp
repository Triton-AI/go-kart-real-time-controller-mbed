#include "Controller/controller.hpp"

#include <iostream>

namespace tritonai::gkc
{
  Controller::Controller() :
    _comm(this)
  {
    std::cout << "Controller created" << std::endl;
  }

  void Controller::packet_callback(const Handshake1GkcPacket &packet)
  {
    std::cout << "Handshake1GkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const Handshake2GkcPacket &packet)
  {
    std::cout << "Handshake2GkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const GetFirmwareVersionGkcPacket &packet)
  {
    std::cout << "GetFirmwareVersionGkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const FirmwareVersionGkcPacket &packet)
  {
    std::cout << "FirmwareVersionGkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const ResetMcuGkcPacket &packet)
  {
    std::cout << "ResetMcuGkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const HeartbeatGkcPacket &packet)
  {
    std::cout << "HeartbeatGkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const ConfigGkcPacket &packet)
  {
    std::cout << "ConfigGkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const StateTransitionGkcPacket &packet)
  {
    std::cout << "StateTransitionGkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const ControlGkcPacket &packet)
  {
    std::cout << "ControlGkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const SensorGkcPacket &packet)
  {
    std::cout << "SensorGkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const Shutdown1GkcPacket &packet)
  {
    std::cout << "Shutdown1GkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const Shutdown2GkcPacket &packet)
  {
    std::cout << "Shutdown2GkcPacket received" << std::endl;
  }

  void Controller::packet_callback(const LogPacket &packet)
  {
    std::cout << "LogPacket received" << std::endl;
  }
} // namespace tritonai::gkc