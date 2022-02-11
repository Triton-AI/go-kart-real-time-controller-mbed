/**
 * @file version.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-01-26
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef TAI_GOKART_PACKET__VERSION_HPP_
#define TAI_GOKART_PACKET__VERSION_HPP_

#include <cstdint>

namespace tritonai
{
namespace gkc
{
class GkcPacketLibVersion
{
public:
  static constexpr uint8_t MAJOR = 0;
  static constexpr uint8_t MINOR = 1;
  static constexpr uint8_t PATCH = 0;
};
}  // namespace gkc
}  // namespace tritonai

#endif  // TAI_GOKART_PACKET__VERSION_HPP_
