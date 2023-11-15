#pragma once

#include <cstdint>

#include <Eigen/Dense>

namespace velodyne_decoder {

struct PointXYZICT {
  float x;
  float y;
  float z;
  float _;
  uint8_t intensity;
  uint8_t channel;
  float timeOffset;

  [[nodiscard]] Eigen::Map<Eigen::Vector3f> getVec3Map() {
    return Eigen::Map<Eigen::Vector3f>(&x);
  }

  [[nodiscard]] Eigen::Map<const Eigen::Vector3f> getVec3Map() const {
    return Eigen::Map<const Eigen::Vector3f>(&x);
  }

  [[nodiscard]] Eigen::Map<Eigen::Vector4f> getVec4Map() {
    return Eigen::Map<Eigen::Vector4f>(&x);
  }

  [[nodiscard]] Eigen::Map<const Eigen::Vector4f> getVec4Map() const {
    return Eigen::Map<const Eigen::Vector4f>(&x);
  }
};

} // namespace velodyne_decoder
