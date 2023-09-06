#pragma once

#include "velodyne_decoder/velodyne_decoder.hpp"

#include <array>
#include <cmath>
#include <vector>

namespace velodyne_decoder::vlp32c {
const std::size_t kNUM_BLOCKS = 12;
const std::size_t kSEQUENCES_PER_BLOCK = 1;
const std::size_t kNUM_CHANNELS = 32;
const std::size_t kBLOCK_SIZE =
    kFLAG_SIZE + kAZIMUTH_SIZE +
    kSEQUENCES_PER_BLOCK * kNUM_CHANNELS * (kRANGE_SIZE + kINTENSITY_SIZE);

const float kCHANNEL_TIME = 2.304e-6f;
const float kRECHARGE_TIME = 18.432e-6f;
const float kSEQUENCE_TIME = kCHANNEL_TIME * kNUM_CHANNELS + kRECHARGE_TIME;
const float kBLOCK_TIME = kSEQUENCE_TIME * kSEQUENCES_PER_BLOCK;

const std::array<float, kNUM_CHANNELS> channelToVerticalAngle{
    -25.0f * kDEG_TO_RAD,   -1.0f * kDEG_TO_RAD,   -1.667f * kDEG_TO_RAD,
    -15.639f * kDEG_TO_RAD, -11.31f * kDEG_TO_RAD, 0.0f * kDEG_TO_RAD,
    -0.667f * kDEG_TO_RAD,  -8.843f * kDEG_TO_RAD, -7.254f * kDEG_TO_RAD,
    0.333f * kDEG_TO_RAD,   -0.333f * kDEG_TO_RAD, -6.148f * kDEG_TO_RAD,
    -5.333f * kDEG_TO_RAD,  1.333f * kDEG_TO_RAD,  0.667f * kDEG_TO_RAD,
    -4.0f * kDEG_TO_RAD,    -4.667f * kDEG_TO_RAD, 1.667f * kDEG_TO_RAD,
    1.0f * kDEG_TO_RAD,     -3.667f * kDEG_TO_RAD, -3.333f * kDEG_TO_RAD,
    3.333f * kDEG_TO_RAD,   2.333f * kDEG_TO_RAD,  -2.667f * kDEG_TO_RAD,
    -3.0f * kDEG_TO_RAD,    7.0f * kDEG_TO_RAD,    4.667f * kDEG_TO_RAD,
    -2.333f * kDEG_TO_RAD,  -2.0f * kDEG_TO_RAD,   15.0f * kDEG_TO_RAD,
    10.333f * kDEG_TO_RAD,  -1.333f * kDEG_TO_RAD};

const std::array<float, kNUM_CHANNELS> channelToSinVerticalAngle{
    std::sin(channelToVerticalAngle[0]),  std::sin(channelToVerticalAngle[1]),
    std::sin(channelToVerticalAngle[2]),  std::sin(channelToVerticalAngle[3]),
    std::sin(channelToVerticalAngle[4]),  std::sin(channelToVerticalAngle[5]),
    std::sin(channelToVerticalAngle[6]),  std::sin(channelToVerticalAngle[7]),
    std::sin(channelToVerticalAngle[8]),  std::sin(channelToVerticalAngle[9]),
    std::sin(channelToVerticalAngle[10]), std::sin(channelToVerticalAngle[11]),
    std::sin(channelToVerticalAngle[12]), std::sin(channelToVerticalAngle[13]),
    std::sin(channelToVerticalAngle[14]), std::sin(channelToVerticalAngle[15]),
    std::sin(channelToVerticalAngle[16]), std::sin(channelToVerticalAngle[17]),
    std::sin(channelToVerticalAngle[18]), std::sin(channelToVerticalAngle[19]),
    std::sin(channelToVerticalAngle[20]), std::sin(channelToVerticalAngle[21]),
    std::sin(channelToVerticalAngle[22]), std::sin(channelToVerticalAngle[23]),
    std::sin(channelToVerticalAngle[24]), std::sin(channelToVerticalAngle[25]),
    std::sin(channelToVerticalAngle[26]), std::sin(channelToVerticalAngle[27]),
    std::sin(channelToVerticalAngle[28]), std::sin(channelToVerticalAngle[29]),
    std::sin(channelToVerticalAngle[30]), std::sin(channelToVerticalAngle[31]),
};

const std::array<float, kNUM_CHANNELS> channelToCosVerticalAngle{
    std::cos(channelToVerticalAngle[0]),  std::cos(channelToVerticalAngle[1]),
    std::cos(channelToVerticalAngle[2]),  std::cos(channelToVerticalAngle[3]),
    std::cos(channelToVerticalAngle[4]),  std::cos(channelToVerticalAngle[5]),
    std::cos(channelToVerticalAngle[6]),  std::cos(channelToVerticalAngle[7]),
    std::cos(channelToVerticalAngle[8]),  std::cos(channelToVerticalAngle[9]),
    std::cos(channelToVerticalAngle[10]), std::cos(channelToVerticalAngle[11]),
    std::cos(channelToVerticalAngle[12]), std::cos(channelToVerticalAngle[13]),
    std::cos(channelToVerticalAngle[14]), std::cos(channelToVerticalAngle[15]),
    std::cos(channelToVerticalAngle[16]), std::cos(channelToVerticalAngle[17]),
    std::cos(channelToVerticalAngle[18]), std::cos(channelToVerticalAngle[19]),
    std::cos(channelToVerticalAngle[20]), std::cos(channelToVerticalAngle[21]),
    std::cos(channelToVerticalAngle[22]), std::cos(channelToVerticalAngle[23]),
    std::cos(channelToVerticalAngle[24]), std::cos(channelToVerticalAngle[25]),
    std::cos(channelToVerticalAngle[26]), std::cos(channelToVerticalAngle[27]),
    std::cos(channelToVerticalAngle[28]), std::cos(channelToVerticalAngle[29]),
    std::cos(channelToVerticalAngle[30]), std::cos(channelToVerticalAngle[31]),
};

struct VelodyneDecoder {
  float m_azimuth;
  VelodyneDecoder();
  void appendToCloud(const VelodynePacket &packet,
                     std::vector<PointXYZICT> &cloud);
};
} // namespace velodyne_decoder::vlp32c
