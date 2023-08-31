#include "velodyne_decoder/velodyne_decoder.hpp"

#include <algorithm>
#include <ranges>

namespace velodyne_decoder {

void appendToCloud(const VelodynePacket &packet,
                   std::vector<PointXYZICT> &cloud) {
  auto cloudInserter = std::back_inserter(cloud);
  std::ranges::for_each(
      std::ranges::views::transform(
          std::ranges::views::iota(0, kNUM_BLOCKS),
          [](const auto &blockNum) -> auto { return kBLOCK_SIZE * blockNum; }),
      [&packet, &cloudInserter](const auto &blockIndex) {
        const float blockAzimuth = static_cast<float>(getBytes<kAZIMUTH_SIZE>(
                                       &packet[blockIndex + kFLAG_SIZE])) *
                                   kDEG_TO_RAD;
        const float blockTime = blockIndex * kBLOCK_TIME_NS;
        std::ranges::copy(
            std::ranges::views::transform(
                std::ranges::views::iota(0, kLIDAR_SCAN_LINES),
                [&blockAzimuth, &blockTime, &packet,
                 &blockIndex](const auto &scanIndex) -> PointXYZICT {
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
                  const float &elevation = kELEVATION_ANGLE[scanIndex];
                  const float planarRange =
                      range * kCOS_ELEVATION_ANGLE[scanIndex];

                  return {
                      .x = planarRange * std::sin(azimuth),
                      .y = planarRange * std::cos(azimuth),
                      .z = range * kSIN_ELEVATION_ANGLE[scanIndex],
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
