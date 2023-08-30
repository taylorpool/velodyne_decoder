#include "velodyne_decoder/velodyne_decoder.hpp"

namespace velodyne_decoder {

SphericalPoint::Vector decodeVelodynePacket(const VelodynePacket &data) {
  float blockAzimuth;
  SphericalPoint point;
  SphericalPoint::Vector points;
  int index = 0;
  int blockTime;

  for (int blockIndex = 0; blockIndex < kNUM_BLOCKS; ++blockIndex) {
    index += kFLAG_SIZE;
    blockAzimuth = static_cast<float>(data[index] | (data[index + 1] << 8));
    blockAzimuth *= kDEG_TO_RAD;
    index += kAZIMUTH_SIZE;

    blockTime = blockIndex * kBLOCK_TIME_NS;

    for (int channelIndex = 0; channelIndex < kLIDAR_SCAN_LINES;
         ++channelIndex) {
      point.azimuth = blockAzimuth + kAZIMUTH_FIXED_OFFSET[channelIndex] +
                      kAZIMUTH_TIME_OFFSET[channelIndex];
      point.azimuth += M_PI_2;
      point.azimuth -= kTWO_PI * std::signbit(point.azimuth - kTWO_PI);
      point.azimuth += kTWO_PI * std::signbit(-point.azimuth);

      point.range = static_cast<float>(data[index] | (data[index + 1] << 8));
      point.range *= kRANGE_TO_M;
      index += kRANGE_SIZE;

      point.elevation = kELEVATION_ANGLE[channelIndex];

      point.intensity = static_cast<float>(data[index]);
      index += kINTENSITY_SIZE;

      point.laserId = kLASER_ORDERING[channelIndex];

      point.timeNs = blockTime + kFIRING_TIME_NS * (channelIndex / 2);

      points.push_back(point);
    }
  }

  return points;
}

void appendToCloud(const VelodynePacket &packet,
                   std::vector<PointXYZICT> &cloud) {
  auto cloudInserter = std::back_inserter(cloud);
  std::ranges::for_each(
      std::ranges::views::transform(
          std::ranges::views::iota(0, kNUM_BLOCKS),
          [](const auto &blockNum) -> auto { return kBLOCK_SIZE * blockNum; }),
      [&packet, &cloudInserter](const auto &blockIndex) {
        const float blockAzimuth =
            static_cast<float>(packet[blockIndex + kFLAG_SIZE] |
                               (packet[blockIndex + kFLAG_SIZE + 1] << 8)) *
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
                      static_cast<float>(
                          packet[blockIndex + kFLAG_SIZE + kAZIMUTH_SIZE +
                                 kPOINT_SIZE * scanIndex] |
                          (packet[blockIndex + kFLAG_SIZE + kAZIMUTH_SIZE +
                                  kPOINT_SIZE * scanIndex + 1]
                           << 8)) *
                      kRANGE_TO_M;
                  const float &elevation = kELEVATION_ANGLE[scanIndex];
                  const float planarRange = range * std::cos(elevation);

                  return {
                      .x = planarRange * std::sin(azimuth),
                      .y = planarRange * std::cos(azimuth),
                      .z = range * std::sin(elevation),
                      .intensity = static_cast<uint8_t>(
                          packet[blockIndex + 3 * scanIndex + 2]),
                      .channel = static_cast<uint8_t>(scanIndex),
                      .timeOffset = static_cast<float>(
                          static_cast<double>(blockTime + kFIRING_TIME_NS *
                                                              (scanIndex / 2)) /
                          1e9)};
                }),
            cloudInserter);
      });
}

void toCloud(const VelodynePacket &packet, std::vector<PointXYZIT> &cloud) {
  float blockAzimuth;
  SphericalPoint point;
  int index = 0;
  int blockTime;

  for (int blockIndex = 0; blockIndex < kNUM_BLOCKS; ++blockIndex) {
    index += kFLAG_SIZE;
    blockAzimuth = static_cast<float>(packet[index] | (packet[index + 1] << 8));
    blockAzimuth *= kDEG_TO_RAD;
    index += kAZIMUTH_SIZE;

    blockTime = blockIndex * kBLOCK_TIME_NS;

    for (int channelIndex = 0; channelIndex < kLIDAR_SCAN_LINES;
         ++channelIndex) {
      point.azimuth = blockAzimuth + kAZIMUTH_FIXED_OFFSET[channelIndex] +
                      kAZIMUTH_TIME_OFFSET[channelIndex];
      point.azimuth += M_PI_2;
      point.azimuth -= kTWO_PI * std::signbit(point.azimuth - kTWO_PI);
      point.azimuth += kTWO_PI * std::signbit(-point.azimuth);

      point.range =
          static_cast<float>(packet[index] | (packet[index + 1] << 8));
      point.range *= kRANGE_TO_M;
      index += kRANGE_SIZE;

      point.elevation = kELEVATION_ANGLE[channelIndex];

      point.intensity = static_cast<float>(packet[index]);
      index += kINTENSITY_SIZE;

      point.laserId = kLASER_ORDERING[channelIndex];

      point.timeNs = blockTime + kFIRING_TIME_NS * (channelIndex / 2);

      cloud.push_back(toPointXYZIT(point));
    }
  }
}

PointXYZIT toPointXYZIT(const SphericalPoint &sphericalPoint) {
  const float planarRange =
      sphericalPoint.range * std::cos(sphericalPoint.elevation);
  return PointXYZIT{
      .x = planarRange * std::sin(sphericalPoint.azimuth),
      .y = planarRange * std::cos(sphericalPoint.azimuth),
      .z = sphericalPoint.range * std::sin(sphericalPoint.elevation),
      .intensity = sphericalPoint.intensity,
      .timeOffset =
          static_cast<float>(static_cast<float>(sphericalPoint.timeNs) / 1e9)};
}

RectangularPoint toRectangularPoint(const SphericalPoint &sphericalPoint) {
  RectangularPoint rectPoint;

  const float planarRange =
      sphericalPoint.range * std::cos(sphericalPoint.elevation);
  rectPoint.timeOffset =
      static_cast<float>(static_cast<float>(sphericalPoint.timeNs) / 1e9);
  rectPoint.x = planarRange * std::sin(sphericalPoint.azimuth);
  rectPoint.y = planarRange * std::cos(sphericalPoint.azimuth);
  rectPoint.z = sphericalPoint.range * std::sin(sphericalPoint.elevation);
  rectPoint.intensity = sphericalPoint.intensity;

  return rectPoint;
}

PointXYZIT::Vector toCloud(const VelodynePacket &packet) {
  float blockAzimuth;
  SphericalPoint point;
  PointXYZIT::Vector points;
  int index = 0;
  int blockTime;

  for (int blockIndex = 0; blockIndex < kNUM_BLOCKS; ++blockIndex) {
    index += kFLAG_SIZE;
    blockAzimuth = static_cast<float>(packet[index] | (packet[index + 1] << 8));
    blockAzimuth *= kDEG_TO_RAD;
    index += kAZIMUTH_SIZE;

    blockTime = blockIndex * kBLOCK_TIME_NS;

    for (int channelIndex = 0; channelIndex < kLIDAR_SCAN_LINES;
         ++channelIndex) {
      point.azimuth = blockAzimuth + kAZIMUTH_FIXED_OFFSET[channelIndex] +
                      kAZIMUTH_TIME_OFFSET[channelIndex];
      point.azimuth += M_PI_2;
      point.azimuth -= kTWO_PI * std::signbit(point.azimuth - kTWO_PI);
      point.azimuth += kTWO_PI * std::signbit(-point.azimuth);

      point.range =
          static_cast<float>(packet[index] | (packet[index + 1] << 8));
      point.range *= kRANGE_TO_M;
      index += kRANGE_SIZE;

      point.elevation = kELEVATION_ANGLE[channelIndex];

      point.intensity = static_cast<float>(packet[index]);
      index += kINTENSITY_SIZE;

      point.laserId = kLASER_ORDERING[channelIndex];

      point.timeNs = blockTime + kFIRING_TIME_NS * (channelIndex / 2);

      points.push_back(toPointXYZIT(point));
    }
  }
  return points;
}

} // namespace velodyne_decoder
