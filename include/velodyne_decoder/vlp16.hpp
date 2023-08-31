#pragma once

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <ranges>
#include <vector>

namespace vlp16 {
const float kDEG_TO_RAD = std::numbers::pi_v<float> / 180.0f;
const float kCENTI_TO_UNIT = 0.01f;
const float kMILLI_TO_UNIT = 0.001f;
const size_t kNUM_CHANNELS = 16;
const size_t kNUM_DATA_BLOCKS = 12;
const float kFIRING_TIME = 55.296f;
const float kCYCLE_TIME = 2.304f;
const size_t kPACKET_SIZE = 1206;
const size_t kFLAG_SIZE = 2;
const size_t kAZIMUTH_SIZE = 2;
const size_t kRANGE_SIZE = 2;
const size_t kINTENSITY_SIZE = 1;
const std::array<float, kNUM_CHANNELS> verticalAngles{
    -15.0f * kDEG_TO_RAD, 1.0f * kDEG_TO_RAD,   -13.0f * kDEG_TO_RAD,
    3.0f * kDEG_TO_RAD,   -11.0f * kDEG_TO_RAD, 5.0f * kDEG_TO_RAD,
    -9.0f * kDEG_TO_RAD,  7.0f * kDEG_TO_RAD,   -7.0f * kDEG_TO_RAD,
    9.0f * kDEG_TO_RAD,   -5.0f * kDEG_TO_RAD,  11.0f * kDEG_TO_RAD,
    -3.0f * kDEG_TO_RAD,  13.0f * kDEG_TO_RAD,  -1.0f * kDEG_TO_RAD,
    15.0f * kDEG_TO_RAD};
const std::array<float, kNUM_CHANNELS> verticalCorrection{
    11.2f * kMILLI_TO_UNIT, -0.7f * kMILLI_TO_UNIT, 9.7f * kMILLI_TO_UNIT,
    -2.2f * kMILLI_TO_UNIT, 8.1f * kMILLI_TO_UNIT,  -3.7f * kMILLI_TO_UNIT,
    6.6f * kMILLI_TO_UNIT,  -5.1f * kMILLI_TO_UNIT, 5.1f * kMILLI_TO_UNIT,
    -6.6f * kMILLI_TO_UNIT, 3.7f * kMILLI_TO_UNIT,  -8.1f * kMILLI_TO_UNIT,
    2.2f * kMILLI_TO_UNIT,  -9.7f * kMILLI_TO_UNIT, 0.7f * kMILLI_TO_UNIT,
    -11.2f * kMILLI_TO_UNIT};
template <size_t sequenceIndex, size_t pointIndex>
const float computeTimeOffset() {
  return kFIRING_TIME * static_cast<float>(sequenceIndex) +
         kCYCLE_TIME * static_cast<float>(pointIndex);
}

struct PointXYZICT {
  float x;
  float y;
  float z;
  uint8_t intensity;
  uint8_t channel;
  float timeOffset;
};

template <size_t N>
requires(N == 1) uint8_t getBytes(const uint8_t bytes[1]) { return bytes[0]; }

template <size_t N, std::endian Endian = std::endian::native>
requires(N == 2 && Endian == std::endian::little) uint16_t
    getBytes(const uint8_t bytes[2]) {
  return (static_cast<uint16_t>(bytes[1]) << 8) |
         static_cast<uint16_t>(bytes[0]);
}

template <size_t N, std::endian Endian = std::endian::native>
requires(N == 2 && Endian == std::endian::big) uint16_t
    getBytes(const uint8_t bytes[2]) {
  return (static_cast<uint16_t>(bytes[0]) << 8) |
         static_cast<uint16_t>(bytes[1]);
}

using VelodynePacket = uint8_t[kPACKET_SIZE];

void appendToCloud(const VelodynePacket &packet,
                   std::vector<PointXYZICT> &cloud);

} // namespace vlp16