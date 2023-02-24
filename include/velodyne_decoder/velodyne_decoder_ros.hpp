#pragma once

#include "velodyne_decoder/velodyne_decoder.hpp"

#include <velodyne_msgs/VelodynePacket.h>

#include <array>

namespace velodyne_decoder {

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
} // namespace velodyne_decoder
