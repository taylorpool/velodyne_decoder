#pragma once

#include "velodyne_decoder/velodyne_decoder.hpp"

#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_msgs/VelodyneScan.h>

#include <sensor_msgs/PointCloud2.h>

namespace velodyne_decoder {

void toMsg(const std::vector<PointXYZICT> &cloud,
           sensor_msgs::PointCloud2 &msg);

} // namespace velodyne_decoder
