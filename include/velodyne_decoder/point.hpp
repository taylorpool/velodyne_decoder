#pragma once

#include <cstdint>

namespace velodyne_decoder {

struct PointXYZICT {
  float x;
  float y;
  float z;
  uint8_t intensity;
  uint8_t channel;
  float timeOffset;
};

} // namespace velodyne_decoder
