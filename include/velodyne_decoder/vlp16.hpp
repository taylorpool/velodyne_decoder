#pragma once

#include "velodyne_decoder/velodyne_decoder.hpp"

#include <array>
#include <cmath>
#include <vector>

namespace velodyne_decoder::vlp16 {

const size_t kNUM_BLOCKS = 12;
const size_t kNUM_CHANNELS = 16;
const size_t kSEQUENCES_PER_BLOCK = 2;
const size_t kBLOCK_SIZE =
    kFLAG_SIZE + kAZIMUTH_SIZE +
    kNUM_CHANNELS * kSEQUENCES_PER_BLOCK * (kRANGE_SIZE + kINTENSITY_SIZE);

const float kCHANNEL_TIME = 2.304e-6f;
const float kRECHARGE_TIME = 18.43e-6f;
const float kSEQUENCE_TIME = kCHANNEL_TIME * kNUM_CHANNELS + kRECHARGE_TIME;
const float kBLOCK_TIME = kSEQUENCE_TIME * kSEQUENCES_PER_BLOCK;

const std::array<float, kNUM_CHANNELS> channelToVerticalAngle{
    -15.0f * kDEG_TO_RAD, 1.0f * kDEG_TO_RAD,   -13.0f * kDEG_TO_RAD,
    3.0f * kDEG_TO_RAD,   -11.0f * kDEG_TO_RAD, 5.0f * kDEG_TO_RAD,
    -9.0f * kDEG_TO_RAD,  7.0f * kDEG_TO_RAD,   -7.0f * kDEG_TO_RAD,
    9.0f * kDEG_TO_RAD,   -5.0f * kDEG_TO_RAD,  11.0f * kDEG_TO_RAD,
    -3.0f * kDEG_TO_RAD,  13.0f * kDEG_TO_RAD,  -1.0f * kDEG_TO_RAD,
    15.0f * kDEG_TO_RAD};

const std::array<float, kNUM_CHANNELS> channelToSinVerticalAngle{
    std::sin(channelToVerticalAngle[0]),  std::sin(channelToVerticalAngle[1]),
    std::sin(channelToVerticalAngle[2]),  std::sin(channelToVerticalAngle[3]),
    std::sin(channelToVerticalAngle[4]),  std::sin(channelToVerticalAngle[5]),
    std::sin(channelToVerticalAngle[6]),  std::sin(channelToVerticalAngle[7]),
    std::sin(channelToVerticalAngle[8]),  std::sin(channelToVerticalAngle[9]),
    std::sin(channelToVerticalAngle[10]), std::sin(channelToVerticalAngle[11]),
    std::sin(channelToVerticalAngle[12]), std::sin(channelToVerticalAngle[13]),
    std::sin(channelToVerticalAngle[14]), std::sin(channelToVerticalAngle[15])};

const std::array<float, kNUM_CHANNELS> channelToCosVerticalAngle{
    std::cos(channelToVerticalAngle[0]),  std::cos(channelToVerticalAngle[1]),
    std::cos(channelToVerticalAngle[2]),  std::cos(channelToVerticalAngle[3]),
    std::cos(channelToVerticalAngle[4]),  std::cos(channelToVerticalAngle[5]),
    std::cos(channelToVerticalAngle[6]),  std::cos(channelToVerticalAngle[7]),
    std::cos(channelToVerticalAngle[8]),  std::cos(channelToVerticalAngle[9]),
    std::cos(channelToVerticalAngle[10]), std::cos(channelToVerticalAngle[11]),
    std::cos(channelToVerticalAngle[12]), std::cos(channelToVerticalAngle[13]),
    std::cos(channelToVerticalAngle[14]), std::cos(channelToVerticalAngle[15])};

const std::array<float, kNUM_CHANNELS> channelToVerticalCorrection{
    11.2f * kMILLI_TO_UNIT, -0.7f * kMILLI_TO_UNIT, 9.7f * kMILLI_TO_UNIT,
    -2.2f * kMILLI_TO_UNIT, 8.1f * kMILLI_TO_UNIT,  -3.7f * kMILLI_TO_UNIT,
    6.6f * kMILLI_TO_UNIT,  -5.1f * kMILLI_TO_UNIT, 5.1f * kMILLI_TO_UNIT,
    -6.6f * kMILLI_TO_UNIT, 3.7f * kMILLI_TO_UNIT,  -8.1f * kMILLI_TO_UNIT,
    2.2f * kMILLI_TO_UNIT,  -9.7f * kMILLI_TO_UNIT, 0.7f * kMILLI_TO_UNIT,
    -11.2f * kMILLI_TO_UNIT};

struct VelodyneDecoder {
  float m_azimuth;
  VelodyneDecoder() : m_azimuth(-1.0) {}
  void appendToCloud(const VelodynePacket &packet,
                     std::vector<PointXYZICT> &cloud);
};

} // namespace velodyne_decoder::vlp16