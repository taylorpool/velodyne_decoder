#include "velodyne_decoder/velodyne_decoder.hpp"

#include <algorithm>
#include <ranges>

namespace velodyne_decoder {

VelodyneParams VelodyneParams::vlp16() {
  VelodyneParams params;
  params.firingToChannel = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10,
                            11, 12, 13, 14, 15, 0,  1,  2,  3,  4, 5,
                            6,  7,  8,  9,  10, 11, 12, 13, 14, 15};
  params.channelToElevation = {
      -15.0f * kDEG_TO_RAD, 1.0f * kDEG_TO_RAD,   -13.0f * kDEG_TO_RAD,
      3.0f * kDEG_TO_RAD,   -11.0f * kDEG_TO_RAD, 5.0f * kDEG_TO_RAD,
      -9.0f * kDEG_TO_RAD,  7.0f * kDEG_TO_RAD,   -7.0f * kDEG_TO_RAD,
      9.0f * kDEG_TO_RAD,   -5.0f * kDEG_TO_RAD,  11.0f * kDEG_TO_RAD,
      -3.0f * kDEG_TO_RAD,  13.0f * kDEG_TO_RAD,  -1.0f * kDEG_TO_RAD,
      15.0f * kDEG_TO_RAD};
  std::for_each(params.channelToElevation.cbegin(),
                params.channelToElevation.cend(),
                [&params](const auto &elevation) {
                  params.channelToSinElevation.push_back(std::sin(elevation));
                  params.channelToCosElevation.push_back(std::cos(elevation));
                });
  return params;
}

VelodyneParams VelodyneParams::vlp32c() {
  VelodyneParams params;
  params.firingToChannel = {0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10,
                            11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
                            22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
  params.channelToElevation = {
      -25.0f,  1.0f,   -1.667f, -15.639f, -11.31f, 0.0f,   -0.667f, -8.843f,
      -7.254f, 0.333f, -0.333f, -6.148f,  -5.333f, 1.333f, 0.667f,  -4.0f,
      -4.667f, 1.667f, 1.0f,    -3.667f,  -3.333f, 3.333f, 2.333f,  -2.667f,
      -3.0f,   7.0f,   4.667f,  -2.333f,  -2.0f,   15.0f,  10.333f, -1.333f};
  std::for_each(params.channelToElevation.begin(),
                params.channelToElevation.end(),
                [](auto &elevationDeg) { elevationDeg *= kDEG_TO_RAD; });
  std::for_each(params.channelToElevation.cbegin(),
                params.channelToElevation.cend(),
                [&params](const auto &elevation) {
                  params.channelToSinElevation.push_back(std::sin(elevation));
                  params.channelToCosElevation.push_back(std::cos(elevation));
                });
  return params;
}

void appendToCloud(const VelodynePacket &packet,
                   std::vector<PointXYZICT> &cloud,
                   const VelodyneParams &params) {
  auto cloudInserter = std::back_inserter(cloud);
  std::ranges::for_each(
      std::ranges::views::transform(
          std::ranges::views::iota(0, kNUM_BLOCKS),
          [](const auto &blockNum) -> auto { return kBLOCK_SIZE * blockNum; }),
      [&packet, &cloudInserter, &params](const auto &blockIndex) {
        const float blockAzimuth = static_cast<float>(getBytes<kAZIMUTH_SIZE>(
                                       &packet[blockIndex + kFLAG_SIZE])) *
                                   kDEG_TO_RAD;
        const float blockTime = blockIndex * kBLOCK_TIME_NS;
        std::ranges::copy(
            std::ranges::views::transform(
                std::ranges::views::iota(0, kLIDAR_SCAN_LINES),
                [&blockAzimuth, &blockTime, &packet, &blockIndex,
                 &params](const auto &scanIndex) -> PointXYZICT {
                  float azimuth = blockAzimuth +
                                  kAZIMUTH_FIXED_OFFSET[scanIndex] +
                                  kAZIMUTH_TIME_OFFSET[scanIndex];
                  azimuth += M_PI_2;
                  azimuth -= kTWO_PI * std::signbit(azimuth - kTWO_PI);
                  azimuth += kTWO_PI * std::signbit(-azimuth);
                  const float range =
                      static_cast<float>(getBytes<kRANGE_SIZE>(
                          &packet[blockIndex + kFLAG_SIZE + kAZIMUTH_SIZE +
                                  kPOINT_SIZE * scanIndex])) *
                      kRANGE_TO_M;
                  const size_t channel = 0;
                  const float planarRange =
                      range * params.channelToCosElevation[channel];

                  return {
                      .x = planarRange * std::sin(azimuth),
                      .y = planarRange * std::cos(azimuth),
                      .z = range * params.channelToSinElevation[channel],
                      .intensity = static_cast<uint8_t>(
                          packet[blockIndex + kPOINT_SIZE * scanIndex +
                                 kRANGE_SIZE]),
                      .channel = static_cast<uint8_t>(scanIndex),
                      .timeOffset = static_cast<float>(
                          static_cast<double>(blockTime + kFIRING_TIME_NS *
                                                              (scanIndex / 2)) /
                          1e9)};
                }),
            cloudInserter);
      });
}

} // namespace velodyne_decoder
