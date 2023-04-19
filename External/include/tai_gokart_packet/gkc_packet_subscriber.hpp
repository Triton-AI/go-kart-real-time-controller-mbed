/**
 * @file gkc_packet_subscriber.hpp
 * @author Haoru Xue (hxue@ucsd.edu)
 * @brief
 * @version 0.1
 * @date 2021-11-04
 *
 * @copyright Copyright (c) 2021 [Triton AI]
 *
 */
#ifndef TAI_GOKART_PACKET__GKC_PACKET_SUBSCRIBER_HPP_
#define TAI_GOKART_PACKET__GKC_PACKET_SUBSCRIBER_HPP_
namespace tritonai
{
namespace gkc
{
class GkcPacketFactory;
class Handshake1GkcPacket;
class Handshake2GkcPacket;
class GetFirmwareVersionGkcPacket;
class FirmwareVersionGkcPacket;
class ResetMcuGkcPacket;
class HeartbeatGkcPacket;
class ConfigGkcPacket;
class StateTransitionGkcPacket;
class ControlGkcPacket;
class SensorGkcPacket;
class Shutdown1GkcPacket;
class Shutdown2GkcPacket;
class LogPacket;
/**
 * @brief Subclass this to receive GkcPackets from GkcPacketFactory
 *
 */
class GkcPacketSubscriber
{
public:
  friend GkcPacketFactory;
  virtual void packet_callback(const Handshake1GkcPacket & packet) = 0;
  virtual void packet_callback(const Handshake2GkcPacket & packet) = 0;
  virtual void packet_callback(const GetFirmwareVersionGkcPacket & packet) = 0;
  virtual void packet_callback(const FirmwareVersionGkcPacket & packet) = 0;
  virtual void packet_callback(const ResetMcuGkcPacket & packet) = 0;
  virtual void packet_callback(const HeartbeatGkcPacket & packet) = 0;
  virtual void packet_callback(const ConfigGkcPacket & packet) = 0;
  virtual void packet_callback(const StateTransitionGkcPacket & packet) = 0;
  virtual void packet_callback(const ControlGkcPacket & packet) = 0;
  virtual void packet_callback(const SensorGkcPacket & packet) = 0;
  virtual void packet_callback(const Shutdown1GkcPacket & packet) = 0;
  virtual void packet_callback(const Shutdown2GkcPacket & packet) = 0;
  virtual void packet_callback(const LogPacket & packet) = 0;
};
}  // namespace gkc
}  // namespace tritonai

#endif  // TAI_GOKART_PACKET__GKC_PACKET_SUBSCRIBER_HPP_
