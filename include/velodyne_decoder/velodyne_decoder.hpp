#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace velodyne_decoder {

struct SphericalPoint {
  float azimuth;
  float elevation;
  float range;
  uint8_t intensity;
  uint8_t laserId;
  int timeNs;
  using Vector = std::vector<SphericalPoint>;
};

struct RectangularPoint {
  float timeOffset;
  float x;
  float y;
  float z;
  uint8_t intensity;
  using Vector = std::vector<RectangularPoint>;
};

RectangularPoint toRectangularPoint(const SphericalPoint &sphericalPoint);

constexpr int kNUM_BLOCKS = 12;
constexpr int kLIDAR_SCAN_LINES = 32;
constexpr int kPOINTS_PER_PACKET = kNUM_BLOCKS * kLIDAR_SCAN_LINES;
constexpr float kDEG_TO_RAD = 0.01 * M_PI / 180.0;
constexpr float kTWO_PI = 2.0f * M_PI;
constexpr float kAZIMUTH_FIXED_OFFSET[kLIDAR_SCAN_LINES] = {
    0.02443461, -0.07330383, 0.02443461, -0.02443461, 0.02443461, -0.02443461,
    0.07330383, -0.02443461, 0.02443461, -0.07330383, 0.02443461, -0.02443461,
    0.07330383, -0.02443461, 0.07330383, -0.02443461, 0.02443461, -0.07330383,
    0.02443461, -0.07330383, 0.07330383, -0.02443461, 0.02443461, -0.02443461,
    0.02443461, -0.02443461, 0.02443461, -0.07330383, 0.07330383, -0.02443461,
    0.02443461, -0.02443461};
constexpr float kAZIMUTH_TIME_OFFSET[kLIDAR_SCAN_LINES] = {
    0.00000000, 0.00000000, 0.00014476, 0.00014476, 0.00028953, 0.00028953,
    0.00043429, 0.00043429, 0.00057906, 0.00057906, 0.00072382, 0.00072382,
    0.00086859, 0.00086859, 0.00101335, 0.00101335, 0.00115812, 0.00115812,
    0.00130288, 0.00130288, 0.00144765, 0.00144765, 0.00159241, 0.00159241,
    0.00173718, 0.00173718, 0.00188194, 0.00188194, 0.00202670, 0.00202670,
    0.00217147, 0.00217147};
constexpr float kELEVATION_ANGLE[kLIDAR_SCAN_LINES] = {
    -0.43633231, -0.01745329, -0.02909464, -0.27295204, -0.19739674,
    0.00000000,  -0.01164135, -0.15433947, -0.12660618, 0.00581195,
    -0.00581195, -0.10730284, -0.09307841, 0.02326524,  0.01164135,
    -0.06981317, -0.08145452, 0.02909464,  0.01745329,  -0.06400122,
    -0.05817182, 0.05817182,  0.04071853,  -0.04654793, -0.05235988,
    0.12217305,  0.08145452,  -0.04071853, -0.03490659, 0.26179939,
    0.18034487,  -0.02326524};
const static float kSIN_ELEVATION_ANGLE[kLIDAR_SCAN_LINES] = {
    std::sin(kELEVATION_ANGLE[0]),  std::sin(kELEVATION_ANGLE[1]),
    std::sin(kELEVATION_ANGLE[2]),  std::sin(kELEVATION_ANGLE[3]),
    std::sin(kELEVATION_ANGLE[4]),  std::sin(kELEVATION_ANGLE[5]),
    std::sin(kELEVATION_ANGLE[6]),  std::sin(kELEVATION_ANGLE[7]),
    std::sin(kELEVATION_ANGLE[8]),  std::sin(kELEVATION_ANGLE[9]),
    std::sin(kELEVATION_ANGLE[10]), std::sin(kELEVATION_ANGLE[11]),
    std::sin(kELEVATION_ANGLE[12]), std::sin(kELEVATION_ANGLE[13]),
    std::sin(kELEVATION_ANGLE[14]), std::sin(kELEVATION_ANGLE[15]),
    std::sin(kELEVATION_ANGLE[16]), std::sin(kELEVATION_ANGLE[17]),
    std::sin(kELEVATION_ANGLE[18]), std::sin(kELEVATION_ANGLE[19]),
    std::sin(kELEVATION_ANGLE[20]), std::sin(kELEVATION_ANGLE[21]),
    std::sin(kELEVATION_ANGLE[22]), std::sin(kELEVATION_ANGLE[23]),
    std::sin(kELEVATION_ANGLE[24]), std::sin(kELEVATION_ANGLE[25]),
    std::sin(kELEVATION_ANGLE[26]), std::sin(kELEVATION_ANGLE[27]),
    std::sin(kELEVATION_ANGLE[28]), std::sin(kELEVATION_ANGLE[29]),
    std::sin(kELEVATION_ANGLE[30]), std::sin(kELEVATION_ANGLE[31])};
const static float kCOS_ELEVATION_ANGLE[kLIDAR_SCAN_LINES] = {
    std::cos(kELEVATION_ANGLE[0]),  std::cos(kELEVATION_ANGLE[1]),
    std::cos(kELEVATION_ANGLE[2]),  std::cos(kELEVATION_ANGLE[3]),
    std::cos(kELEVATION_ANGLE[4]),  std::cos(kELEVATION_ANGLE[5]),
    std::cos(kELEVATION_ANGLE[6]),  std::cos(kELEVATION_ANGLE[7]),
    std::cos(kELEVATION_ANGLE[8]),  std::cos(kELEVATION_ANGLE[9]),
    std::cos(kELEVATION_ANGLE[10]), std::cos(kELEVATION_ANGLE[11]),
    std::cos(kELEVATION_ANGLE[12]), std::cos(kELEVATION_ANGLE[13]),
    std::cos(kELEVATION_ANGLE[14]), std::cos(kELEVATION_ANGLE[15]),
    std::cos(kELEVATION_ANGLE[16]), std::cos(kELEVATION_ANGLE[17]),
    std::cos(kELEVATION_ANGLE[18]), std::cos(kELEVATION_ANGLE[19]),
    std::cos(kELEVATION_ANGLE[20]), std::cos(kELEVATION_ANGLE[21]),
    std::cos(kELEVATION_ANGLE[22]), std::cos(kELEVATION_ANGLE[23]),
    std::cos(kELEVATION_ANGLE[24]), std::cos(kELEVATION_ANGLE[25]),
    std::cos(kELEVATION_ANGLE[26]), std::cos(kELEVATION_ANGLE[27]),
    std::cos(kELEVATION_ANGLE[28]), std::cos(kELEVATION_ANGLE[29]),
    std::cos(kELEVATION_ANGLE[30]), std::cos(kELEVATION_ANGLE[31])};
constexpr uint8_t kLASER_ORDERING[kLIDAR_SCAN_LINES] = {
    0,  3, 4, 7,  8, 11, 12, 16, 15, 19, 20, 24, 23, 27, 28, 2,
    31, 1, 6, 10, 5, 9,  14, 18, 13, 17, 22, 21, 26, 25, 30, 29};
constexpr float kRANGE_TO_M = 4.0 / 1000.0;
constexpr int kFLAG_SIZE = 2;
constexpr int kAZIMUTH_SIZE = 2;
constexpr int kRANGE_SIZE = 2;
constexpr int kINTENSITY_SIZE = 1;
constexpr int kBLOCK_TIME_NS = 55296;
constexpr int kFIRING_TIME_NS = 2304;
constexpr int kPACKET_TIME_NS = kBLOCK_TIME_NS * kNUM_BLOCKS;
constexpr float kLIDAR_MESSAGE_TIME =
    static_cast<float>(kNUM_BLOCKS * kBLOCK_TIME_NS * 151) * 1e-9;
constexpr float kLIDAR_TIME_EPSILON = 0.05 * kLIDAR_MESSAGE_TIME;
constexpr float kLIDAR_ANGULAR_RESOLUTION = 0.003467542;

using VelodynePacket = uint8_t[1206];
SphericalPoint::Vector decodeVelodynePacket(const VelodynePacket &packet);

}; // namespace velodyne_decoder