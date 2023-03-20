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

void decode(const velodyne_msgs::VelodyneScan::ConstPtr &msg,
            sensor_msgs::PointCloud2 &cloud) {
  cloud.header = msg->header;
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.fields.resize(4);

  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[0].count = 1;

  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = cloud.fields[0].datatype;
  cloud.fields[1].count = 1;

  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = cloud.fields[0].datatype;
  cloud.fields[2].count = 1;

  cloud.fields[3].name = "intensity";
  cloud.fields[3].offset = 12;
  cloud.fields[3].datatype = sensor_msgs::PointField::UINT16;
  cloud.fields[3].count = 1;

  cloud.is_bigendian = true;
  cloud.is_bigendian = kIS_BIG_ENDIAN;
  const auto &points = velodyne_decoder::decodeVelodynePackets(msg->packets);
  cloud.point_step = sizeof(points[0]);
  cloud.row_step = points.size();
  cloud.width = points.size();
  for (const auto &point : points) {
    for (const auto byte : point.data) {
      cloud.data.push_back(byte);
    }
  }
}
} // namespace velodyne_decoder
