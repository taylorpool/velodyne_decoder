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

      point.intensity = data[index];
      index += kINTENSITY_SIZE;

      point.laserId = kLASER_ORDERING[channelIndex];

      point.timeNs = blockTime + kFIRING_TIME_NS * (channelIndex / 2);

      points.push_back(point);
    }
  }

  return points;
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

} // namespace velodyne_decoder
