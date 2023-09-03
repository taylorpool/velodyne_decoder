#include "velodyne_decoder/vlp16.hpp"

namespace velodyne_decoder::vlp16 {

void appendToCloud(const VelodynePacket &packet,
                   std::vector<PointXYZICT> &cloud) {
  size_t index = 0;
  size_t sequence = 0;
  for (size_t block = 0; block < kNUM_DATA_BLOCKS; ++block) {
    index += kFLAG_SIZE;
    const float blockAzimuth =
        static_cast<float>(getBytes<kAZIMUTH_SIZE>(&packet[index])) *
        kCENTI_TO_UNIT * kDEG_TO_RAD;
    index += kAZIMUTH_SIZE;
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
      sequence += 1;
      const float azimuth = blockAzimuth;
      const float xyRange = range * channelToCosVerticalAngle[channel];
      cloud.push_back({.x = xyRange * std::sin(azimuth),
                       .y = xyRange * std::cos(azimuth),
                       .z = range * channelToSinVerticalAngle[channel] +
                            channelToVerticalCorrection[channel],
                       .intensity = intensity,
                       .channel = static_cast<uint8_t>(channel),
                       .timeOffset = timeOffset});
    }
  }
}
} // namespace velodyne_decoder::vlp16
