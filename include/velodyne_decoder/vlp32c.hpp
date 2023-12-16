#pragma once

#include "velodyne_decoder/velodyne_decoder.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <ranges>
#include <vector>
#include <iostream>

namespace velodyne_decoder::vlp32c {

class VelodyneDecoder {
private:
  constexpr static size_t kNUM_CHANNELS = 32;
  std::array<float, kNUM_CHANNELS> m_channelToSinVerticalAngle;
  std::array<float, kNUM_CHANNELS> m_channelToCosVerticalAngle;
  float m_azimuth;

public:
  [[nodiscard]] VelodyneDecoder();

  [[nodiscard]] size_t decodePacket(VelodynePacket packet) noexcept;

  template <typename T>
    requires(std::ranges::input_range<T>)
  [[nodiscard]] std::vector<velodyne_decoder::PointXYZICT>
  decode(const T &packets) {
    std::vector<velodyne_decoder::PointXYZICT> cloud;
    cloud.reserve(packets.size());
    std::ranges::for_each(packets, [&cloud, this](const auto &packet) {
      constexpr float kTWO_PI = std::numbers::pi_v<float> * 2.0f;
      constexpr float kCENTI_TO_UNIT = 0.01f;
      constexpr float kMILLI_TO_UNIT = 0.001f;
      constexpr std::size_t kFLAG_SIZE = 2;
      constexpr std::size_t kAZIMUTH_SIZE = 2;
      constexpr std::size_t kRANGE_SIZE = 2;
      constexpr std::size_t kINTENSITY_SIZE = 1;
      constexpr size_t kNUM_BLOCKS = 12;
      constexpr size_t kSEQUENCES_PER_BLOCK = 1;
      constexpr size_t kBLOCK_SIZE = kFLAG_SIZE + kAZIMUTH_SIZE +
                                     kSEQUENCES_PER_BLOCK * kNUM_CHANNELS *
                                         (kRANGE_SIZE + kINTENSITY_SIZE);
      constexpr float kCHANNEL_TIME = 2.304e-6f;
      constexpr float kRECHARGE_TIME = 18.432e-6f;
      constexpr size_t kNUM_FIRINGS = 16;
      constexpr float kSEQUENCE_TIME =
          kCHANNEL_TIME * kNUM_FIRINGS + kRECHARGE_TIME;
      constexpr float kBLOCK_TIME = kSEQUENCE_TIME * kSEQUENCES_PER_BLOCK;

      size_t index = 0;
      for (size_t block = 0; block < kNUM_BLOCKS; ++block) {
        index += kFLAG_SIZE;
        const float blockAzimuth =
            static_cast<float>(getBytes<kAZIMUTH_SIZE>(&packet[index])) *
            kCENTI_TO_UNIT * kDEG_TO_RAD +
            std::numbers::pi_v<float> / 2.0f;

        float azimuthRate = 0.0;
        if (m_azimuth >= 0.0) {
          azimuthRate = (blockAzimuth - m_azimuth);
          if (azimuthRate < 0) {
            azimuthRate += kTWO_PI;
          }
          azimuthRate /= kSEQUENCE_TIME;
        }

        index += kAZIMUTH_SIZE;
        for (size_t sequence = 0; sequence < kSEQUENCES_PER_BLOCK; ++sequence) {
          for (size_t channel = 0; channel < kNUM_CHANNELS; ++channel) {
            const float range =
                static_cast<float>(static_cast<uint16_t>(2) *
                                   getBytes<kRANGE_SIZE>(&packet[index])) *
                kMILLI_TO_UNIT;
            index += kRANGE_SIZE;
            const uint8_t intensity = getBytes<kINTENSITY_SIZE>(&packet[index]);
            index += kINTENSITY_SIZE;
            const float timeOffset =
                kBLOCK_TIME * static_cast<float>(block) +
                kSEQUENCE_TIME * static_cast<float>(sequence) +
                kCHANNEL_TIME * static_cast<float>(channel);
            std::cout << timeOffset << "\n";
            float preciseAzimuth =
                blockAzimuth +
                azimuthRate * (kCHANNEL_TIME * static_cast<float>(channel) +
                               kSEQUENCE_TIME * sequence);

            const float xyRange = range * m_channelToCosVerticalAngle[channel];
            cloud.push_back({.x = xyRange * std::sin(preciseAzimuth),
                             .y = xyRange * std::cos(preciseAzimuth),
                             .z = range * m_channelToSinVerticalAngle[channel],
                             .intensity = intensity,
                             .channel = static_cast<uint8_t>(channel),
                             .timeOffset = timeOffset});
          }
        }
      }
    });
    return cloud;
  }
};
} // namespace velodyne_decoder::vlp32c
