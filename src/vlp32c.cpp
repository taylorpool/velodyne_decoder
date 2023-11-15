#include "velodyne_decoder/vlp32c.hpp"

#include <algorithm>
#include <numbers>
#include <ratio>

namespace velodyne_decoder::vlp32c {

VelodyneDecoder::VelodyneDecoder() : m_azimuth(-1.0) {
  constexpr std::array<float, kNUM_CHANNELS> channelToVerticalAngle{
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

  std::ranges::transform(channelToVerticalAngle,
                         m_channelToSinVerticalAngle.begin(),
                         [](float x) { return std::sin(x); });
  std::ranges::transform(channelToVerticalAngle,
                         m_channelToCosVerticalAngle.begin(),
                         [](float x) { return std::cos(x); });
}

} // namespace velodyne_decoder::vlp32c
