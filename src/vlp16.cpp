#include "velodyne_decoder/vlp16.hpp"

namespace velodyne_decoder::vlp16 {
VelodyneDecoder::VelodyneDecoder() noexcept : m_azimuth(-1.0) {
  constexpr std::array<float, kNUM_CHANNELS> channelToVerticalAngle{
      -15.0f * kDEG_TO_RAD, 1.0f * kDEG_TO_RAD,   -13.0f * kDEG_TO_RAD,
      3.0f * kDEG_TO_RAD,   -11.0f * kDEG_TO_RAD, 5.0f * kDEG_TO_RAD,
      -9.0f * kDEG_TO_RAD,  7.0f * kDEG_TO_RAD,   -7.0f * kDEG_TO_RAD,
      9.0f * kDEG_TO_RAD,   -5.0f * kDEG_TO_RAD,  11.0f * kDEG_TO_RAD,
      -3.0f * kDEG_TO_RAD,  13.0f * kDEG_TO_RAD,  -1.0f * kDEG_TO_RAD,
      15.0f * kDEG_TO_RAD};

  std::ranges::transform(channelToVerticalAngle,
                         m_channelToSinVerticalAngle.begin(),
                         [](float x) { return std::sin(x); });
  std::ranges::transform(channelToVerticalAngle,
                         m_channelToCosVerticalAngle.begin(),
                         [](float x) { return std::cos(x); });
}
} // namespace velodyne_decoder::vlp16
