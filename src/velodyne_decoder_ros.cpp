#include "velodyne_decoder/velodyne_decoder_ros.hpp"

#include "velodyne_decoder/vlp16.hpp"

namespace velodyne_decoder {

std::vector<PointXYZICT> decodeVelodynePackets2(
    const std::vector<velodyne_msgs::VelodynePacket> &packets) {
  std::vector<PointXYZICT> cloud;
  std::ranges::for_each(packets, [&cloud](const auto &packet) {
    float azimuth = -1.0f;
    vlp16::appendToCloud(packet.data.elems, cloud, azimuth);
  });
  return cloud;
}

void toMsg(const std::vector<PointXYZICT> &cloud,
           sensor_msgs::PointCloud2 &msg) {
  msg.height = 1;

  msg.width = cloud.size();

  msg.fields.resize(6);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;

  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;

  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;

  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 12;
  msg.fields[3].datatype = sensor_msgs::PointField::UINT8;
  msg.fields[3].count = 1;

  msg.fields[4].name = "channel";
  msg.fields[4].offset = 13;
  msg.fields[4].datatype = sensor_msgs::PointField::UINT8;
  msg.fields[4].count = 1;

  msg.fields[5].name = "timeOffset";
  msg.fields[5].offset = 14;
  msg.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[5].count = 1;

  msg.is_bigendian = (std::endian::native == std::endian::big);

  msg.point_step = static_cast<uint32_t>(18);

  msg.row_step = msg.width * msg.point_step;

  msg.data.clear();
  msg.data.reserve(msg.row_step * msg.height);

  for (const auto &point : cloud) {
    const uint8_t *xdata = reinterpret_cast<const uint8_t *>(&point.x);
    msg.data.push_back(xdata[0]);
    msg.data.push_back(xdata[1]);
    msg.data.push_back(xdata[2]);
    msg.data.push_back(xdata[3]);

    const uint8_t *ydata = reinterpret_cast<const uint8_t *>(&point.y);
    msg.data.push_back(ydata[0]);
    msg.data.push_back(ydata[1]);
    msg.data.push_back(ydata[2]);
    msg.data.push_back(ydata[3]);

    const uint8_t *zdata = reinterpret_cast<const uint8_t *>(&point.z);
    msg.data.push_back(zdata[0]);
    msg.data.push_back(zdata[1]);
    msg.data.push_back(zdata[2]);
    msg.data.push_back(zdata[3]);

    const uint8_t *intensityData =
        reinterpret_cast<const uint8_t *>(&point.intensity);
    msg.data.push_back(intensityData[0]);

    const uint8_t *channelData =
        reinterpret_cast<const uint8_t *>(&point.channel);
    msg.data.push_back(channelData[0]);

    const uint8_t *timeOffsetData =
        reinterpret_cast<const uint8_t *>(&point.timeOffset);
    msg.data.push_back(timeOffsetData[0]);
    msg.data.push_back(timeOffsetData[1]);
    msg.data.push_back(timeOffsetData[2]);
    msg.data.push_back(timeOffsetData[3]);
  }
  msg.is_dense = false;
}

} // namespace velodyne_decoder
