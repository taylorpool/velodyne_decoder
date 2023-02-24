#include "velodyne_decoder/velodyne_decoder_ros.hpp"

namespace velodyne_decoder {
RectangularPoint::Vector decodeVelodynePackets(
    const std::vector<velodyne_msgs::VelodynePacket> &packets) {
  SphericalPoint::Vector sphericalPoints;
  RectangularPoint::Vector pointCloud;
  RectangularPoint rectangularPoint;

  for (const auto &packet : packets) {
    sphericalPoints = decodeVelodynePacket(toStdArray(packet.data));
    for (const auto &sphericalPoint : sphericalPoints) {
      rectangularPoint = toRectangularPoint(sphericalPoint);
      pointCloud.push_back(rectangularPoint);
    }
  }
  return pointCloud;
}
} // namespace velodyne_decoder
