/**
 * @file vesc_can_helper.cpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-05-03
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#ifndef VESC_CAN_HELPER_HPP_
#define VESC_CAN_HELPER_HPP_
#include <cstdint>


inline void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

inline void buffer_append_uint16(uint8_t *buffer, uint16_t number, int32_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

inline void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

inline void buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

inline void buffer_append_float16(uint8_t *buffer, float number, float scale,
                           int32_t *index) {
  buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

inline void buffer_append_float32(uint8_t *buffer, float number, float scale,
                           int32_t *index) {
  buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

inline int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
  int16_t res =
      ((uint16_t)buffer[*index]) << 8 | ((uint16_t)buffer[*index + 1]);
  *index += 2;
  return res;
}

inline uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index) {
  uint16_t res =
      ((uint16_t)buffer[*index]) << 8 | ((uint16_t)buffer[*index + 1]);
  *index += 2;
  return res;
}

inline int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
  int32_t res =
      ((uint32_t)buffer[*index]) << 24 | ((uint32_t)buffer[*index + 1]) << 16 |
      ((uint32_t)buffer[*index + 2]) << 8 | ((uint32_t)buffer[*index + 3]);
  *index += 4;
  return res;
}

inline uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index) {
  uint32_t res =
      ((uint32_t)buffer[*index]) << 24 | ((uint32_t)buffer[*index + 1]) << 16 |
      ((uint32_t)buffer[*index + 2]) << 8 | ((uint32_t)buffer[*index + 3]);
  *index += 4;
  return res;
}

inline float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index) {
  return (float)buffer_get_int16(buffer, index) / scale;
}

inline float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index) {
  return (float)buffer_get_int32(buffer, index) / scale;
}

#endif // VESC_CAN_HELPER_HPP_