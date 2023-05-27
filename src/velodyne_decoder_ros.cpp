#include "velodyne_decoder/velodyne_decoder_ros.hpp"

namespace velodyne_decoder {
RectangularPoint::Vector decodeVelodynePackets(
    const std::vector<velodyne_msgs::VelodynePacket> &packets) {
  SphericalPoint::Vector sphericalPoints;
  RectangularPoint::Vector pointCloud;
  RectangularPoint rectangularPoint;

  for (const auto &packet : packets) {
    sphericalPoints = decodeVelodynePacket(packet.data.elems);
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

  const auto points = velodyne_decoder::decodeVelodynePackets(msg->packets);

  cloud.height = 1;

  cloud.width = points.size();

  cloud.fields.resize(5);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[0].count = 1;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[1].count = 1;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[2].count = 1;
  cloud.fields[3].name = "intensity";
  cloud.fields[3].offset = 12;
  cloud.fields[3].datatype = sensor_msgs::PointField::UINT16;
  cloud.fields[3].count = 1;
  cloud.fields[4].name = "time_offset";
  cloud.fields[4].offset = 14;
  cloud.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[4].count = 1;

  cloud.is_bigendian = kIS_BIG_ENDIAN;

  cloud.point_step = static_cast<uint32_t>(18);

  cloud.row_step = cloud.width * cloud.point_step;

  cloud.data.clear();
  cloud.data.reserve(cloud.row_step * cloud.height);
  for (const auto &point : points) {
    const uint8_t *xdata = reinterpret_cast<const uint8_t *>(&point.x);
    cloud.data.push_back(xdata[0]);
    cloud.data.push_back(xdata[1]);
    cloud.data.push_back(xdata[2]);
    cloud.data.push_back(xdata[3]);
    const uint8_t *ydata = reinterpret_cast<const uint8_t *>(&point.y);
    cloud.data.push_back(ydata[0]);
    cloud.data.push_back(ydata[1]);
    cloud.data.push_back(ydata[2]);
    cloud.data.push_back(ydata[3]);
    const uint8_t *zdata = reinterpret_cast<const uint8_t *>(&point.z);
    cloud.data.push_back(zdata[0]);
    cloud.data.push_back(zdata[1]);
    cloud.data.push_back(zdata[2]);
    cloud.data.push_back(zdata[3]);
    const uint8_t *intensityData =
        reinterpret_cast<const uint8_t *>(&point.intensity);
    cloud.data.push_back(intensityData[0]);
    cloud.data.push_back(intensityData[1]);
    const uint8_t *timeOffsetData =
        reinterpret_cast<const uint8_t *>(&point.timeOffset);
    cloud.data.push_back(timeOffsetData[0]);
    cloud.data.push_back(timeOffsetData[1]);
    cloud.data.push_back(timeOffsetData[2]);
    cloud.data.push_back(timeOffsetData[3]);
  }
  cloud.is_dense = false;
}
} // namespace velodyne_decoder
