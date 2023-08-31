#pragma once

#include "velodyne_decoder/velodyne_decoder.hpp"

#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_msgs/VelodyneScan.h>

#include <sensor_msgs/PointCloud2.h>

namespace velodyne_decoder {

void decode(const velodyne_msgs::VelodyneScanConstPtr &msg,
            sensor_msgs::PointCloud2 &cloud);

} // namespace velodyne_decoder
