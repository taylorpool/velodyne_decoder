#pragma once

#include "velodyne_decoder/point.hpp"

#include <bit>
#include <cstdint>
#include <numbers>

namespace velodyne_decoder {

const float kDEG_TO_RAD = std::numbers::pi_v<float> / 180.0f;
const float kTWO_PI = std::numbers::pi_v<float> * 2.0f;
const float kCENTI_TO_UNIT = 0.01f;
const float kMILLI_TO_UNIT = 0.001f;

const std::size_t kPACKET_SIZE = 1206;
const std::size_t kFLAG_SIZE = 2;
const std::size_t kAZIMUTH_SIZE = 2;
const std::size_t kRANGE_SIZE = 2;
const std::size_t kINTENSITY_SIZE = 1;

using VelodynePacket = uint8_t[kPACKET_SIZE];

template <std::size_t N>
  requires(N == 1)
uint8_t getBytes(const uint8_t bytes[1]) {
  return bytes[0];
}

template <std::size_t N, std::endian Endian = std::endian::native>
  requires(N == 2 && Endian == std::endian::little)
uint16_t getBytes(const uint8_t bytes[2]) {
  return (static_cast<uint16_t>(bytes[1]) << 8) |
         static_cast<uint16_t>(bytes[0]);
}

template <std::size_t N, std::endian Endian = std::endian::native>
  requires(N == 2 && Endian == std::endian::big)
uint16_t getBytes(const uint8_t bytes[2]) {
  return (static_cast<uint16_t>(bytes[0]) << 8) |
         static_cast<uint16_t>(bytes[1]);
}

}; // namespace velodyne_decoder