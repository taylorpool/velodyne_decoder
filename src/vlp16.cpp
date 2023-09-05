#include "velodyne_decoder/vlp16.hpp"

#include <iostream>

namespace velodyne_decoder::vlp16 {

void VelodyneDecoder::appendToCloud(const VelodynePacket &packet,
                                    std::vector<PointXYZICT> &cloud) {
  size_t index = 0;
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
        const float timeOffset = kBLOCK_TIME * static_cast<float>(block) +
                                 kSEQUENCE_TIME * static_cast<float>(sequence) +
                                 kCHANNEL_TIME * static_cast<float>(channel);
        float preciseAzimuth =
            blockAzimuth +
            azimuthRate * (kCHANNEL_TIME * static_cast<float>(channel) +
                           kSEQUENCE_TIME * sequence);

        const float xyRange = range * channelToCosVerticalAngle[channel];
        cloud.push_back({.x = xyRange * std::sin(preciseAzimuth),
                         .y = xyRange * std::cos(preciseAzimuth),
                         .z = range * channelToSinVerticalAngle[channel] +
                              channelToVerticalCorrection[channel],
                         .intensity = intensity,
                         .channel = static_cast<uint8_t>(channel),
                         .timeOffset = timeOffset});
      }
    }
  }
}
} // namespace velodyne_decoder::vlp16
