#include "velodyne_decoder/velodyne_decoder_ros.hpp"

#include <ranges>
#include <span>

namespace velodyne_decoder {

void toMsg(const std::vector<PointXYZICT> &cloud,
           sensor_msgs::PointCloud2 &msg) {
  msg.height = 1;

  msg.width = cloud.size();

  msg.fields.resize(6);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;

  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;

  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;

  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 12;
  msg.fields[3].datatype = sensor_msgs::PointField::UINT8;
  msg.fields[3].count = 1;

  msg.fields[4].name = "channel";
  msg.fields[4].offset = 13;
  msg.fields[4].datatype = sensor_msgs::PointField::UINT8;
  msg.fields[4].count = 1;

  msg.fields[5].name = "timeOffset";
  msg.fields[5].offset = 14;
  msg.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[5].count = 1;

  msg.is_bigendian = (std::endian::native == std::endian::big);

  msg.point_step = static_cast<uint32_t>(
      sizeof(PointXYZICT::x) + sizeof(PointXYZICT::y) + sizeof(PointXYZICT::z) +
      sizeof(PointXYZICT::intensity) + sizeof(PointXYZICT::channel) +
      sizeof(PointXYZICT::timeOffset));

  msg.row_step = msg.width * msg.point_step;

  msg.data.clear();
  msg.data.reserve(msg.row_step * msg.height);
  for (const auto &point : cloud) {
    std::ranges::copy(
        std::span{reinterpret_cast<const uint8_t *>(&point.x), sizeof(point.x)},
        std::back_inserter(msg.data));
    std::ranges::copy(
        std::span{reinterpret_cast<const uint8_t *>(&point.y), sizeof(point.y)},
        std::back_inserter(msg.data));
    std::ranges::copy(
        std::span{reinterpret_cast<const uint8_t *>(&point.z), sizeof(point.z)},
        std::back_inserter(msg.data));
    std::ranges::copy(
        std::span{reinterpret_cast<const uint8_t *>(&point.intensity),
                  sizeof(point.intensity)},
        std::back_inserter(msg.data));
    std::ranges::copy(
        std::span{reinterpret_cast<const uint8_t *>(&point.channel),
                  sizeof(point.channel)},
        std::back_inserter(msg.data));
    std::ranges::copy(
        std::span{reinterpret_cast<const uint8_t *>(&point.timeOffset),
                  sizeof(point.timeOffset)},
        std::back_inserter(msg.data));
  }
  msg.is_dense = false;
}

void toMsg(const LidarScanStamped& scan,
    sensor_msgs::PointCloud2& msg)
{
    msg.header.stamp.fromSec(scan.stamp);
    toMsg(scan.cloud, msg);
}

VLP16Decoder::VLP16Decoder(): m_azimuth(-1.0)
{
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

LidarScanStamped VLP16Decoder::decode(const velodyne_msgs::VelodyneScan::ConstPtr& scan)
{
    LidarScanStamped result;
    result.stamp = scan->header.stamp.toSec();
    for(const auto& packet : scan->packets)
    {
        const double packetTimeOffset = packet.stamp.toSec() - result.stamp;

      constexpr std::array<float, kNUM_CHANNELS> channelToVerticalCorrection{
          11.2f * kMILLI_TO_UNIT, -0.7f * kMILLI_TO_UNIT,
          9.7f * kMILLI_TO_UNIT,  -2.2f * kMILLI_TO_UNIT,
          8.1f * kMILLI_TO_UNIT,  -3.7f * kMILLI_TO_UNIT,
          6.6f * kMILLI_TO_UNIT,  -5.1f * kMILLI_TO_UNIT,
          5.1f * kMILLI_TO_UNIT,  -6.6f * kMILLI_TO_UNIT,
          3.7f * kMILLI_TO_UNIT,  -8.1f * kMILLI_TO_UNIT,
          2.2f * kMILLI_TO_UNIT,  -9.7f * kMILLI_TO_UNIT,
          0.7f * kMILLI_TO_UNIT,  -11.2f * kMILLI_TO_UNIT};
      constexpr size_t kNUM_BLOCKS = 12;
      constexpr size_t kSEQUENCES_PER_BLOCK = 2;
      constexpr size_t kFLAG_SIZE = 2;
      constexpr size_t kAZIMUTH_SIZE = 2;
      constexpr size_t kRANGE_SIZE = 2;
      constexpr size_t kINTENSITY_SIZE = 1;
      constexpr size_t kBLOCK_SIZE = kFLAG_SIZE + kAZIMUTH_SIZE +
                                     kNUM_CHANNELS * kSEQUENCES_PER_BLOCK *
                                         (kRANGE_SIZE + kINTENSITY_SIZE);

      constexpr float kCHANNEL_TIME = 2.304e-6f;
      constexpr float kRECHARGE_TIME = 18.43e-6f;
      constexpr float kSEQUENCE_TIME =
          kCHANNEL_TIME * kNUM_CHANNELS + kRECHARGE_TIME;
      constexpr float kBLOCK_TIME = kSEQUENCE_TIME * kSEQUENCES_PER_BLOCK;

      size_t index = 0;
      for (size_t block = 0; block < kNUM_BLOCKS; ++block) {
        index += kFLAG_SIZE;
        constexpr float kCENTI_TO_UNIT = 0.01;
        const float blockAzimuth =
            static_cast<float>(getBytes<kAZIMUTH_SIZE>(&packet.data[index])) *
                kCENTI_TO_UNIT * kDEG_TO_RAD +
            std::numbers::pi_v<float> / 2.0f;

        float azimuthRate = 0.0;
        if (m_azimuth >= 0.0) {
          azimuthRate = (blockAzimuth - m_azimuth);
          if (azimuthRate < 0) {
            constexpr float kTWO_PI = std::numbers::pi_v<float> * 2;
            azimuthRate += kTWO_PI;
          }
          azimuthRate /= kSEQUENCE_TIME;
        }

        index += kAZIMUTH_SIZE;
        for (size_t sequence = 0; sequence < kSEQUENCES_PER_BLOCK; ++sequence) {
          for (size_t channel = 0; channel < kNUM_CHANNELS; ++channel) {
            const float range =
                static_cast<float>(static_cast<uint16_t>(2) *
                                   getBytes<kRANGE_SIZE>(&packet.data[index])) *
                kMILLI_TO_UNIT;
            index += kRANGE_SIZE;
            const uint8_t intensity = getBytes<kINTENSITY_SIZE>(&packet.data[index]);
            index += kINTENSITY_SIZE;
            const float timeOffset = packetTimeOffset +
                kBLOCK_TIME * static_cast<float>(block) +
                kSEQUENCE_TIME * static_cast<float>(sequence) +
                kCHANNEL_TIME * static_cast<float>(channel);
            const float preciseAzimuth =
                blockAzimuth +
                azimuthRate * (kCHANNEL_TIME * static_cast<float>(channel) +
                               kSEQUENCE_TIME * sequence);
            std::cout << timeOffset << std::endl;

            const float xyRange = range * m_channelToCosVerticalAngle[channel];
            result.cloud.push_back({.x = xyRange * std::sin(preciseAzimuth),
                             .y = xyRange * std::cos(preciseAzimuth),
                             .z = range * m_channelToSinVerticalAngle[channel] +
                                  channelToVerticalCorrection[channel],
                             .intensity = intensity,
                             .channel = static_cast<uint8_t>(channel),
                             .timeOffset = timeOffset});
          }
        }
      }
    }
    return result;
}

VLP32CDecoder::VLP32CDecoder() : m_azimuth(-1.0)
{
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

LidarScanStamped VLP32CDecoder::decode(const velodyne_msgs::VelodyneScan::ConstPtr& msg)
{
    LidarScanStamped result;
    result.stamp = msg->header.stamp.toSec();
    std::cout << "new scan\n";

    for(const auto& packet : msg->packets)
    {
        const double packetTimeOffset = packet.stamp.toSec() - result.stamp;
        std::cout << packetTimeOffset<< std::endl;
      constexpr float kTWO_PI = std::numbers::pi_v<float> * 2.0f;
      constexpr float kCENTI_TO_UNIT = 0.01f;
      constexpr float kMILLI_TO_UNIT = 0.001f;
      constexpr std::size_t kFLAG_SIZE = 2;
      constexpr std::size_t kAZIMUTH_SIZE = 2;
      constexpr std::size_t kRANGE_SIZE = 2;
      constexpr std::size_t kINTENSITY_SIZE = 1;
      constexpr size_t kNUM_BLOCKS = 12;
      constexpr size_t kSEQUENCES_PER_BLOCK = 1;
      constexpr size_t kBLOCK_SIZE = kFLAG_SIZE + kAZIMUTH_SIZE +
                                     kSEQUENCES_PER_BLOCK * kNUM_CHANNELS *
                                         (kRANGE_SIZE + kINTENSITY_SIZE);
      constexpr float kCHANNEL_TIME = 2.304e-6f;
      constexpr float kRECHARGE_TIME = 18.432e-6f;
      constexpr size_t kNUM_FIRINGS = 16;
      constexpr float kSEQUENCE_TIME =
          kCHANNEL_TIME * kNUM_FIRINGS + kRECHARGE_TIME;
      constexpr float kBLOCK_TIME = kSEQUENCE_TIME * kSEQUENCES_PER_BLOCK;

      size_t index = 0;
      for (size_t block = 0; block < kNUM_BLOCKS; ++block) {
        index += kFLAG_SIZE;
        const float blockAzimuth =
            static_cast<float>(getBytes<kAZIMUTH_SIZE>(&packet.data[index])) *
            kCENTI_TO_UNIT * kDEG_TO_RAD +
            std::numbers::pi_v<float> / 2.0f;

        float azimuthRate = 0.0;
        if (m_azimuth >= 0.0) {
          azimuthRate = (blockAzimuth - m_azimuth);
          if (azimuthRate < 0) {
            azimuthRate += kTWO_PI;
          }
          azimuthRate /= kSEQUENCE_TIME;
        }

        index += kAZIMUTH_SIZE;
        for (size_t sequence = 0; sequence < kSEQUENCES_PER_BLOCK; ++sequence) {
          for (size_t channel = 0; channel < kNUM_CHANNELS; ++channel) {
            const float range =
                static_cast<float>(static_cast<uint16_t>(2) *
                                   getBytes<kRANGE_SIZE>(&packet.data[index])) *
                kMILLI_TO_UNIT;
            index += kRANGE_SIZE;
            const uint8_t intensity = getBytes<kINTENSITY_SIZE>(&packet.data[index]);
            index += kINTENSITY_SIZE;
            const float timeOffset = packetTimeOffset +
                kBLOCK_TIME * static_cast<float>(block) +
                kSEQUENCE_TIME * static_cast<float>(sequence) +
                kCHANNEL_TIME * static_cast<float>(channel);
            // std::cout << timeOffset << "\n";
            float preciseAzimuth =
                blockAzimuth +
                azimuthRate * (kCHANNEL_TIME * static_cast<float>(channel) +
                               kSEQUENCE_TIME * sequence);

            const float xyRange = range * m_channelToCosVerticalAngle[channel];
            result.cloud.push_back({.x = xyRange * std::sin(preciseAzimuth),
                             .y = xyRange * std::cos(preciseAzimuth),
                             .z = range * m_channelToSinVerticalAngle[channel],
                             .intensity = intensity,
                             .channel = static_cast<uint8_t>(channel),
                             .timeOffset = timeOffset});
          }
        }
      }
    }
    return result;
}

RacerDecoder::RacerDecoder()
{
    for(std::size_t i = 0; i < LIDAR_SCAN_LINES; ++i)
    {
        m_cosElevationAngles(i) = std::cos(ELEVATION_ANGLE[i]);
        m_sinElevationAngles(i) = std::sin(ELEVATION_ANGLE[i]);
    }
}

std::vector<PointXYZICT> RacerDecoder::decode(
    const velodyne_msgs::VelodyneScan::ConstPtr& scan)
{
    std::vector<PointXYZICT> cloud;

    size_t packetIdx, blockIdx, channelIdx;
    size_t azimuthIdx, rangeIdx, intensityIdx;
    float blockAzimuth;
    unsigned int blockTime;
    float pointTime;
    size_t numPackets = scan->packets.size();
    size_t fullCloudSize = 0;
    size_t filteredCloudSize = 0;

    Eigen::Array<float, LIDAR_SCAN_LINES, 1>    azimuths;
    Eigen::Array<float, LIDAR_SCAN_LINES, 1>    elevations;
    Eigen::Array<uint16_t, LIDAR_SCAN_LINES, 1> ranges;
    Eigen::Array<float, LIDAR_SCAN_LINES, 1>    meterRanges;
    Eigen::Array<float, LIDAR_SCAN_LINES, 1>    planarRanges;
    Eigen::Array<uint8_t, LIDAR_SCAN_LINES, 1>  intensities;
    Eigen::Array<float, LIDAR_SCAN_LINES, 1>    tmp;
    Eigen::Array<bool, LIDAR_SCAN_LINES, 1>     fullCloudMask;
    Eigen::Array<bool, LIDAR_SCAN_LINES, 1>     filteredCloudMask;
    Eigen::Array<bool, LIDAR_SCAN_LINES, 1>     noReturnMask;
    Eigen::Matrix<float, 3, LIDAR_SCAN_LINES>   points;

    for(packetIdx = 0; packetIdx < numPackets; packetIdx++) {
        const auto& dataRef = scan->packets[packetIdx].data;
        for(blockIdx = 0; blockIdx < NUM_BLOCKS; blockIdx++) {
            azimuthIdx = (100 * blockIdx) + 2;
            blockAzimuth = dataRef[azimuthIdx] | (dataRef[azimuthIdx + 1] << 8);
            blockAzimuth *= CDEG_TO_RAD;
            for(channelIdx = 0; channelIdx < LIDAR_SCAN_LINES; channelIdx++) {
                rangeIdx = (100 * blockIdx) + 4 + (3 * channelIdx);
                ranges(channelIdx) = dataRef[rangeIdx] | (dataRef[rangeIdx + 1] << 8);
                azimuths(channelIdx) = blockAzimuth + AZIMUTH_FIXED_OFFSET[channelIdx] + AZIMUTH_TIME_OFFSET[channelIdx];
                intensityIdx = (100 * blockIdx) + 6 + (3 * channelIdx);
                intensities(channelIdx) = dataRef[intensityIdx];
            }

            // noReturnMask = (ranges == 0);   // Find non-returns
            meterRanges = RANGE_TO_M * ranges.cast<float>();
            // meterRanges += noReturnMask.cast<float>() * m_maxRange;    // Set non-returns to max range

            azimuths += M_PI_2;
            tmp = meterRanges * m_cosElevationAngles;
            points.row(0) = tmp * Eigen::sin(azimuths);             // x
            points.row(1) = tmp * Eigen::cos(azimuths);             // y
            points.row(2) = meterRanges * m_sinElevationAngles;     // z

            planarRanges = Eigen::sqrt(Eigen::square(points.row(0).array()) + Eigen::square(points.row(1).array()));

            // Exclude points inside of a bounding box around the vehicle
        //     fullCloudMask = !(
        //         (points.row(0).array() < m_blindFront) &&
        //         (points.row(0).array() > m_blindBack) &&
        //         (points.row(1).array() < m_blindLeft) &&
        //         (points.row(1).array() > m_blindRight) &&
        // (m_cutoffSlope*planarRanges.transpose()+m_cutoffIntercept < points.row(2).array())
        // );
            // Filter out points that are too close or far, or don't meet the intensity threshold
            // fullCloudMask = fullCloudMask && (meterRanges >= m_minRange);
            // fullCloudMask = fullCloudMask && (meterRanges <= m_maxRange);
            // fullCloudMask = fullCloudMask && (intensities >= m_minIntensity);

            // filteredCloudMask = fullCloudMask;

            // Include zero-range points in full cloud
            // fullCloudMask = fullCloudMask || noReturnMask;
            // Exclude points behind the odometry frame (x<0)
            // fullCloudMask = fullCloudMask && (points.row(0).array().transpose() >= 0.0);

            // Exclude zero-range points from filtered cloud
            // filteredCloudMask = filteredCloudMask && (!noReturnMask);

            blockTime = ((packetIdx * NUM_BLOCKS + blockIdx) * BLOCK_TIME_NS);

            for(size_t i = 0; i < LIDAR_SCAN_LINES; i++) {
                // Compute time in seconds since the start of the packet
                pointTime = (float)(blockTime + ((i/2) * FIRING_TIME_NS)) * 1.0e-9;
                velodyne_decoder::PointXYZICT point;
                point.x= points.col(i).matrix().x();
                point.y = points.col(i).matrix().y();
                point.z = points.col(i).matrix().z();
                point.intensity = intensities(i);
                point.timeOffset = pointTime;
                std::cout << point.z << ", " << point.timeOffset << std::endl;
                point.channel = LASER_ORDERING[i];
                // (*filteredCloud)[filteredCloudSize].getVector3fMap() = points.col(i).matrix();
                // (*filteredCloud)[filteredCloudSize].intensity = intensities(i);
                // (*filteredCloud)[filteredCloudSize].time = pointTime;
                // (*filteredCloud)[filteredCloudSize].laserId = LASER_ORDERING[i];
                cloud.push_back(point);
            }
        }
    }
    return cloud;
}

} // namespace velodyne_decoder
