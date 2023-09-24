#include "velodyne_decoder/velodyne_decoder_ros.hpp"

#include "velodyne_decoder/vlp16.hpp"

#include <ranges>
#include <span>

namespace velodyne_decoder {

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

  msg.point_step = static_cast<uint32_t>(
      sizeof(PointXYZICT::x) + sizeof(PointXYZICT::y) + sizeof(PointXYZICT::z) +
      sizeof(PointXYZICT::intensity) + sizeof(PointXYZICT::channel) +
      sizeof(PointXYZICT::timeOffset));

  msg.row_step = msg.width * msg.point_step;

  msg.data.clear();
  msg.data.reserve(msg.row_step * msg.height);
  for (const auto &point : cloud) {
    std::ranges::copy(
        std::span{reinterpret_cast<const uint8_t *>(&point.x), sizeof(point.x)},
        std::back_inserter(msg.data));
    std::ranges::copy(
        std::span{reinterpret_cast<const uint8_t *>(&point.y), sizeof(point.y)},
        std::back_inserter(msg.data));
    std::ranges::copy(
        std::span{reinterpret_cast<const uint8_t *>(&point.z), sizeof(point.z)},
        std::back_inserter(msg.data));
    std::ranges::copy(
        std::span{reinterpret_cast<const uint8_t *>(&point.intensity),
                  sizeof(point.intensity)},
        std::back_inserter(msg.data));
    std::ranges::copy(
        std::span{reinterpret_cast<const uint8_t *>(&point.channel),
                  sizeof(point.channel)},
        std::back_inserter(msg.data));
    std::ranges::copy(
        std::span{reinterpret_cast<const uint8_t *>(&point.timeOffset),
                  sizeof(point.timeOffset)},
        std::back_inserter(msg.data));
  }
  msg.is_dense = false;
}

} // namespace velodyne_decoder
