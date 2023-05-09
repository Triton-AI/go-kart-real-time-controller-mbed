/**
 * @file gkc_packet_factory.hpp
 * @author Haoru Xue (hxue@ucsd.edu)
 * @brief Encode and decode packets
 * @version 0.1
 * @date 2021-10-29
 *
 * @copyright Copyright (c) 2021 [Triton AI]
 *
 */

#ifndef TAI_GOKART_PACKET__GKC_PACKET_FACTORY_HPP_
#define TAI_GOKART_PACKET__GKC_PACKET_FACTORY_HPP_

#include <iostream>
#include <memory>
#include <unordered_map>
#include <string>
#include <vector>
#include "tai_gokart_packet/gkc_packets.hpp"

namespace tritonai
{
namespace gkc
{
class GkcPacketFactory
{
public:
  GkcPacketFactory(GkcPacketSubscriber * sub, void(*debug)(std::string));

  void Receive(const GkcBuffer & buffer);
  void Receive(const RawGkcBuffer & buffer);
  std::shared_ptr<GkcBuffer> Send(const GkcPacket::SharedPtr & packet);
  std::shared_ptr<GkcBuffer> Send(const GkcPacket & packet);

private:
  typedef GkcPacket::SharedPtr (* Creator)();
  const std::unordered_map<uint8_t, Creator> fb_lookup = {
    {Handshake1GkcPacket::FIRST_BYTE,
      GkcPacketUtils::CreatePacket<Handshake1GkcPacket>},
    {Handshake2GkcPacket::FIRST_BYTE,
      GkcPacketUtils::CreatePacket<Handshake2GkcPacket>},
    {GetFirmwareVersionGkcPacket::FIRST_BYTE,
      GkcPacketUtils::CreatePacket<GetFirmwareVersionGkcPacket>},
    {FirmwareVersionGkcPacket::FIRST_BYTE,
      GkcPacketUtils::CreatePacket<FirmwareVersionGkcPacket>},
    {ResetMcuGkcPacket::FIRST_BYTE,
      GkcPacketUtils::CreatePacket<ResetMcuGkcPacket>},
    {HeartbeatGkcPacket::FIRST_BYTE,
      GkcPacketUtils::CreatePacket<HeartbeatGkcPacket>},
    {ConfigGkcPacket::FIRST_BYTE,
      GkcPacketUtils::CreatePacket<ConfigGkcPacket>},
    {StateTransitionGkcPacket::FIRST_BYTE,
      GkcPacketUtils::CreatePacket<StateTransitionGkcPacket>},
    {ControlGkcPacket::FIRST_BYTE,
      GkcPacketUtils::CreatePacket<ControlGkcPacket>},
    {SensorGkcPacket::FIRST_BYTE,
      GkcPacketUtils::CreatePacket<SensorGkcPacket>},
    {Shutdown1GkcPacket::FIRST_BYTE,
      GkcPacketUtils::CreatePacket<Shutdown1GkcPacket>},
    {Shutdown2GkcPacket::FIRST_BYTE,
      GkcPacketUtils::CreatePacket<Shutdown2GkcPacket>},
    {LogPacket::FIRST_BYTE,
      GkcPacketUtils::CreatePacket<LogPacket>},
  };

  void (* _debug)(std::string);
  GkcBuffer _buffer = GkcBuffer();
  GkcPacketSubscriber * _sub;
};

}  // namespace gkc
}  // namespace tritonai
#endif  // TAI_GOKART_PACKET__GKC_PACKET_FACTORY_HPP_
