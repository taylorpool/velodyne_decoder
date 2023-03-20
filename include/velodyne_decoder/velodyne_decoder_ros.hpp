#pragma once

#include "velodyne_decoder/velodyne_decoder.hpp"

#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_msgs/VelodyneScan.h>

#include <sensor_msgs/PointCloud2.h>

#include <array>

namespace velodyne_decoder {

constexpr bool kIS_BIG_ENDIAN =
    (static_cast<uint16_t>(256) & (static_cast<uint8_t>(1) << 8)) ==
    static_cast<uint16_t>(256);

template <typename T, uint64_t N>
constexpr std::array<T, N> toStdArray(const boost::array<T, N> &boostArray) {
  std::array<T, N> stdArray;
  for (uint64_t index = 0; index < N; ++index) {
    stdArray[index] = boostArray[index];
  }
  return stdArray;
}

RectangularPoint::Vector decodeVelodynePackets(
    const std::vector<velodyne_msgs::VelodynePacket> &packets);

void decode(const velodyne_msgs::VelodyneScanConstPtr &msg,
            sensor_msgs::PointCloud2 &cloud);

} // namespace velodyne_decoder
