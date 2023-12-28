#pragma once

#include "velodyne_decoder/velodyne_decoder.hpp"

#include <array>
#include <ranges>
#include <vector>

namespace velodyne_decoder::vlp16 {

template <typename T>
concept VelodyneScanInput =
    std::ranges::input_range<T> &&
    std::is_same<std::ranges::range_value_t<T>, VelodynePacket>::value;

class VelodyneDecoder {
private:
  constexpr static size_t kNUM_CHANNELS = 16;
  constexpr static float kMILLI_TO_UNIT = 0.001;
  std::array<float, kNUM_CHANNELS> m_channelToSinVerticalAngle;
  std::array<float, kNUM_CHANNELS> m_channelToCosVerticalAngle;

  float m_azimuth;

public:
  [[nodiscard]] VelodyneDecoder() noexcept;

  std::vector<pcl_types::PointXYZICT>
  decode(const std::ranges::input_range auto &packets) {
    std::vector<pcl_types::PointXYZICT> cloud;
    std::ranges::for_each(packets, [this, &cloud](const auto &packet) {
      constexpr std::array<float, kNUM_CHANNELS> channelToVerticalCorrection{
          11.2f * kMILLI_TO_UNIT, -0.7f * kMILLI_TO_UNIT,
          9.7f * kMILLI_TO_UNIT,  -2.2f * kMILLI_TO_UNIT,
          8.1f * kMILLI_TO_UNIT,  -3.7f * kMILLI_TO_UNIT,
          6.6f * kMILLI_TO_UNIT,  -5.1f * kMILLI_TO_UNIT,
          5.1f * kMILLI_TO_UNIT,  -6.6f * kMILLI_TO_UNIT,
          3.7f * kMILLI_TO_UNIT,  -8.1f * kMILLI_TO_UNIT,
          2.2f * kMILLI_TO_UNIT,  -9.7f * kMILLI_TO_UNIT,
          0.7f * kMILLI_TO_UNIT,  -11.2f * kMILLI_TO_UNIT};
      constexpr size_t kNUM_BLOCKS = 12;
      constexpr size_t kSEQUENCES_PER_BLOCK = 2;
      constexpr size_t kFLAG_SIZE = 2;
      constexpr size_t kAZIMUTH_SIZE = 2;
      constexpr size_t kRANGE_SIZE = 2;
      constexpr size_t kINTENSITY_SIZE = 1;
      constexpr size_t kBLOCK_SIZE = kFLAG_SIZE + kAZIMUTH_SIZE +
                                     kNUM_CHANNELS * kSEQUENCES_PER_BLOCK *
                                         (kRANGE_SIZE + kINTENSITY_SIZE);

      constexpr float kCHANNEL_TIME = 2.304e-6f;
      constexpr float kRECHARGE_TIME = 18.43e-6f;
      constexpr float kSEQUENCE_TIME =
          kCHANNEL_TIME * kNUM_CHANNELS + kRECHARGE_TIME;
      constexpr float kBLOCK_TIME = kSEQUENCE_TIME * kSEQUENCES_PER_BLOCK;
      size_t index = 0;
      for (size_t block = 0; block < kNUM_BLOCKS; ++block) {
        index += kFLAG_SIZE;
        constexpr float kCENTI_TO_UNIT = 0.01;
        const float blockAzimuth =
            static_cast<float>(getBytes<kAZIMUTH_SIZE>(&packet[index])) *
                kCENTI_TO_UNIT * kDEG_TO_RAD +
            std::numbers::pi_v<float> / 2.0f;

        float azimuthRate = 0.0;
        if (m_azimuth >= 0.0) {
          azimuthRate = (blockAzimuth - m_azimuth);
          if (azimuthRate < 0) {
            constexpr float kTWO_PI = std::numbers::pi_v<float> * 2;
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
            const float preciseAzimuth =
                blockAzimuth +
                azimuthRate * (kCHANNEL_TIME * static_cast<float>(channel) +
                               kSEQUENCE_TIME * sequence);

            const float xyRange = range * m_channelToCosVerticalAngle[channel];
            cloud.push_back({.x = xyRange * std::sin(preciseAzimuth),
                             .y = xyRange * std::cos(preciseAzimuth),
                             .z = range * m_channelToSinVerticalAngle[channel] +
                                  channelToVerticalCorrection[channel],
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

} // namespace velodyne_decoder::vlp16
