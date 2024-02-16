/**
 * @file gkc_packets.hpp
 * @author Haoru Xue (hxue@ucsd.edu)
 * @brief Packet structures
 * @version 0.1
 * @date 2021-10-29
 *
 * @copyright Copyright (c) 2021 [Triton AI]
 *
 */

#ifndef TAI_GOKART_PACKET__GKC_PACKETS_HPP_
#define TAI_GOKART_PACKET__GKC_PACKETS_HPP_

#include <optional>
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>

#include "tai_gokart_packet/gkc_packet_subscriber.hpp"
#include "tai_gokart_packet/gkc_packet_utils.hpp"
#include "Tools/rc_control.hpp"

namespace tritonai
{
namespace gkc
{

using GkcBuffer = std::vector<uint8_t>;
class RawGkcPacket
{
public:
  static constexpr uint8_t START_BYTE = 0x02;
  static constexpr uint8_t END_BYTE = 0x03;

  RawGkcPacket();

  /**
   * @brief Construct a new Raw Gkc Packet with payload and calculates checksum
   *
   * @param payload payload of the packet
   */

  explicit RawGkcPacket(const GkcBuffer & payload);
  RawGkcPacket(const RawGkcPacket & packet);
  RawGkcPacket & operator=(const RawGkcPacket & packet);

  std::shared_ptr<GkcBuffer> encode();

  uint8_t payload_size;
  uint16_t checksum;
  GkcBuffer payload;

  typedef std::shared_ptr<RawGkcPacket> SharedPtr;
  typedef std::unique_ptr<RawGkcPacket> UniquePtr;
};

class GkcPacket
{
public:
  typedef std::shared_ptr<GkcPacket> SharedPtr;
  typedef std::unique_ptr<GkcPacket> UniquePtr;
  uint64_t timestamp = 0;
  virtual ~GkcPacket();
  virtual RawGkcPacket::SharedPtr encode() const;
  virtual void decode(const RawGkcPacket & raw);
  virtual void publish(GkcPacketSubscriber & sub);
};

class Handshake1GkcPacket : public GkcPacket
{
public:
  static constexpr uint8_t FIRST_BYTE = 0x4;
  uint32_t seq_number = 0;
  RawGkcPacket::SharedPtr encode() const;
  void decode(const RawGkcPacket & raw);
  void publish(GkcPacketSubscriber & sub) {return sub.packet_callback(*this);}
};

class Handshake2GkcPacket : public GkcPacket
{
public:
  static constexpr uint8_t FIRST_BYTE = 0x5;
  uint32_t seq_number = 0;
  RawGkcPacket::SharedPtr encode() const;
  void decode(const RawGkcPacket & raw);
  void publish(GkcPacketSubscriber & sub) {sub.packet_callback(*this);}
};

class GetFirmwareVersionGkcPacket : public GkcPacket
{
public:
  static constexpr uint8_t FIRST_BYTE = 0x6;
  RawGkcPacket::SharedPtr encode() const;
  void decode(const RawGkcPacket & raw);
  void publish(GkcPacketSubscriber & sub) {sub.packet_callback(*this);}
};

class FirmwareVersionGkcPacket : public GkcPacket
{
public:
  static constexpr uint8_t FIRST_BYTE = 0x7;
  uint8_t major = 0;
  uint8_t minor = 0;
  uint8_t patch = 0;
  RawGkcPacket::SharedPtr encode() const;
  void decode(const RawGkcPacket & raw);
  void publish(GkcPacketSubscriber & sub) {sub.packet_callback(*this);}
};

class ResetRTCGkcPacket : public GkcPacket
{
public:
  static constexpr uint8_t FIRST_BYTE = 0xFF;
  uint32_t magic_number = 0;
  RawGkcPacket::SharedPtr encode() const;
  void decode(const RawGkcPacket & raw);
  void publish(GkcPacketSubscriber & sub) {sub.packet_callback(*this);}
};

class HeartbeatGkcPacket : public GkcPacket
{
public:
  static constexpr uint8_t FIRST_BYTE = 0xAA;
  uint8_t rolling_counter = 0;
  uint8_t state = 0;
  RawGkcPacket::SharedPtr encode() const;
  void decode(const RawGkcPacket & raw);
  void publish(GkcPacketSubscriber & sub) {sub.packet_callback(*this);}
};

class ConfigGkcPacket : public GkcPacket
{
public:
  static constexpr uint8_t FIRST_BYTE = 0xA0;
  struct __attribute__((packed)) Configurables
  {
    // steering config (refers to average front wheel angle in radian)
    float max_steering_left;
    float max_steering_right;
    float neutral_steering;  // should be between max and min

    // throttle config (unit is implementation-dependant, typically unit-less out of 1.0)
    float max_throttle;
    float min_throttle;
    float zero_throttle;  // should be smaller than min

    // brake config (in psi)
    float max_brake;
    float min_brake;
    float zero_brake;  // should be smaller than min

    // watchdog timeouts (in millisecond)
    uint32_t control_timeout_ms;  // timeout for control packets
    uint32_t comm_timeout_ms;  // timeout for heartbeat packets
    uint32_t sensor_timeout_ms;  // timeout between two sensor pollings
  } values;

  RawGkcPacket::SharedPtr encode() const;
  void decode(const RawGkcPacket & raw);
  void publish(GkcPacketSubscriber & sub) {sub.packet_callback(*this);}
};

class StateTransitionGkcPacket : public GkcPacket
{
public:
  static constexpr uint8_t FIRST_BYTE = 0xA1;
  uint8_t requested_state = 0;
  RawGkcPacket::SharedPtr encode() const;
  void decode(const RawGkcPacket & raw);
  void publish(GkcPacketSubscriber & sub) {sub.packet_callback(*this);}
};

class ControlGkcPacket : public GkcPacket
{
public:
  static constexpr uint8_t FIRST_BYTE = 0xAB;
  float throttle;  // paddle percentage out of 1.0
  float steering;  // average front wheel angle in radian
  float brake;  // target brake pressure in psi
  RawGkcPacket::SharedPtr encode() const;
  void decode(const RawGkcPacket & raw);
  void publish(GkcPacketSubscriber & sub) {sub.packet_callback(*this);}
};
class SensorGkcPacket : public GkcPacket
{
public:
  static constexpr uint8_t FIRST_BYTE = 0xAC;
  struct __attribute__((packed)) SensorValues
  {
    float wheel_speed_fl;  // wheel speeds in rpm
    float wheel_speed_fr;
    float wheel_speed_rl;
    float wheel_speed_rr;

    float voltage;  // battery voltage in volt
    float amperage;  // battery current draw in amp

    float brake_pressure;  // brake pressure in psi
    float throttle_pos;  // throttle paddle position out of 1.0
    float steering_angle_rad;  // (left +, right -) average wheel angle of the front wheels in rad
    float servo_angle_rad;  // (left +, right -) servo offset from center in rad

    bool fault_brake;  // fault flag in actuation subsystem
    bool fault_throttle;
    bool fault_steering;

    bool fault_fatal;  // fault flag with severity level
    bool fault_error;
    bool fault_warning;
    bool fault_info;
  } values;
  RawGkcPacket::SharedPtr encode() const;
  void decode(const RawGkcPacket & raw);
  void publish(GkcPacketSubscriber & sub) {sub.packet_callback(*this);}
};
class RCControlGkcPacket : public GkcPacket
{
public:
  static constexpr uint8_t FIRST_BYTE = 0xAE;
  float throttle;  // paddle percentage out of 1.0
  float steering;  // average front wheel angle in radian
  float brake;  // target brake pressure in psi
  bool is_active;  // whether the emergency stop is active
  AutonomyMode autonomy_mode; // the autonomy mode
  RawGkcPacket::SharedPtr encode() const;
  void decode(const RawGkcPacket & raw);
  void publish(GkcPacketSubscriber & sub) {sub.packet_callback(*this);}
};

class Shutdown1GkcPacket : public GkcPacket
{
public:
  static constexpr uint8_t FIRST_BYTE = 0xA2;
  uint32_t seq_number;
  RawGkcPacket::SharedPtr encode() const;
  void decode(const RawGkcPacket & raw);
  void publish(GkcPacketSubscriber & sub) {sub.packet_callback(*this);}
};

class Shutdown2GkcPacket : public GkcPacket
{
public:
  static constexpr uint8_t FIRST_BYTE = 0xA3;
  uint32_t seq_number;
  RawGkcPacket::SharedPtr encode() const;
  void decode(const RawGkcPacket & raw);
  void publish(GkcPacketSubscriber & sub) {sub.packet_callback(*this);}
};

class LogPacket : public GkcPacket
{
public:
  static constexpr uint8_t FIRST_BYTE = 0xAD;
  enum Severity
  {
    INFO = 0,
    WARNING = 1,
    ERROR = 2,
    FATAL = 3
  } level;
  std::string what;
  RawGkcPacket::SharedPtr encode() const;
  void decode(const RawGkcPacket & raw);
  void publish(GkcPacketSubscriber & sub) {sub.packet_callback(*this);}
};
}  // namespace gkc
}  // namespace tritonai
#endif  // TAI_GOKART_PACKET__GKC_PACKETS_HPP_
