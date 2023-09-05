#include "velodyne_decoder/vlp16.hpp"

#include <iostream>

namespace velodyne_decoder::vlp16 {

void appendToCloud(const VelodynePacket &packet,
                   std::vector<PointXYZICT> &cloud, float &azimuth) {
  size_t index = 0;
  size_t sequence = 0;
  for (size_t block = 0; block < kNUM_BLOCKS; ++block) {
    index += kFLAG_SIZE;
    const float blockAzimuth =
        static_cast<float>(getBytes<kAZIMUTH_SIZE>(&packet[index])) *
        kCENTI_TO_UNIT * kDEG_TO_RAD;

    // float azimuthRate = (blockAzimuth - azimuth);
    // if (azimuthRate < 0) {
    //   azimuthRate += kTWO_PI;
    // }
    // azimuthRate /= kFIRING_TIME;
    const float azimuthRate = 0.0f;

    index += kAZIMUTH_SIZE;
    int channelCount = 0;
    for (const auto &channel : firingToChannel) {
      const float range =
          static_cast<float>(static_cast<uint16_t>(2) *
                             getBytes<kRANGE_SIZE>(&packet[index])) *
          kMILLI_TO_UNIT;
      index += kRANGE_SIZE;
      const uint8_t intensity = getBytes<kINTENSITY_SIZE>(&packet[index]);
      index += kINTENSITY_SIZE;
      const float timeOffset = kFIRING_TIME * static_cast<float>(sequence) +
                               kCYCLE_TIME * static_cast<float>(channel);
      const float preciseAzimuth = blockAzimuth + azimuthRate * timeOffset;
      const float xyRange = range * channelToCosVerticalAngle[channel];
      cloud.push_back({.x = xyRange * std::sin(preciseAzimuth),
                       .y = xyRange * std::cos(preciseAzimuth),
                       .z = range * channelToSinVerticalAngle[channel] +
                            channelToVerticalCorrection[channel],
                       .intensity = intensity,
                       .channel = static_cast<uint8_t>(channel),
                       .timeOffset = timeOffset});
      ++sequence;
      ++channelCount;
    }
    azimuth = blockAzimuth;
  }
}

void VelodyneDecoder::appendToCloud(const VelodynePacket &packet,
                                    std::vector<PointXYZICT> &cloud) {
  size_t index = 0;
  size_t sequence = 0;
  for (size_t block = 0; block < kNUM_BLOCKS; ++block) {
    index += kFLAG_SIZE;
    const float blockAzimuth =
        static_cast<float>(getBytes<kAZIMUTH_SIZE>(&packet[index])) *
        kCENTI_TO_UNIT * kDEG_TO_RAD;

    float azimuthRate = 0.0;
    if (m_azimuth >= 0.0) {
      azimuthRate = (blockAzimuth - m_azimuth);
      if (azimuthRate < 0) {
        azimuthRate += kTWO_PI;
      }
      std::cout << "azimuth diff: " << azimuthRate << std::endl;
      azimuthRate /= kFIRING_TIME;
    }
    std::cout << "azimuth rate: " << azimuthRate << std::endl;

    index += kAZIMUTH_SIZE;
    int channelCount = 0;
    for (const auto &channel : firingToChannel) {
      const float range =
          static_cast<float>(static_cast<uint16_t>(2) *
                             getBytes<kRANGE_SIZE>(&packet[index])) *
          kMILLI_TO_UNIT;
      index += kRANGE_SIZE;
      const uint8_t intensity = getBytes<kINTENSITY_SIZE>(&packet[index]);
      index += kINTENSITY_SIZE;
      const float timeOffset = kFIRING_TIME * static_cast<float>(sequence) +
                               kCYCLE_TIME * static_cast<float>(channel);
      float preciseAzimuth = blockAzimuth + azimuthRate * kCYCLE_TIME *
                                                static_cast<float>(channel);
      if (channelCount > 15) {
        preciseAzimuth += azimuthRate * kFIRING_TIME;
      }
      const float xyRange = range * channelToCosVerticalAngle[channel];
      cloud.push_back({.x = xyRange * std::sin(preciseAzimuth),
                       .y = xyRange * std::cos(preciseAzimuth),
                       .z = range * channelToSinVerticalAngle[channel] +
                            channelToVerticalCorrection[channel],
                       .intensity = intensity,
                       .channel = static_cast<uint8_t>(channel),
                       .timeOffset = timeOffset});
      ++sequence;
      ++channelCount;
    }
    m_azimuth = blockAzimuth;
  }
}
} // namespace velodyne_decoder::vlp16
