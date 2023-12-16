#pragma once

#include "velodyne_decoder/velodyne_decoder.hpp"

#include <velodyne_msgs/VelodynePacket.h>
#include <velodyne_msgs/VelodyneScan.h>

#include <sensor_msgs/PointCloud2.h>

namespace velodyne_decoder {

void toMsg(const std::vector<PointXYZICT> &cloud,
           sensor_msgs::PointCloud2 &msg);


void toMsg(const LidarScanStamped& scan,
            sensor_msgs::PointCloud2& msg);


class VLP16Decoder{
    private:
    constexpr static size_t kNUM_CHANNELS = 16;
    constexpr static float kMILLI_TO_UNIT = 0.001;
    std::array<float, kNUM_CHANNELS> m_channelToSinVerticalAngle;
    std::array<float, kNUM_CHANNELS> m_channelToCosVerticalAngle;

    float m_azimuth;

    public:

    VLP16Decoder();

    LidarScanStamped decode(const velodyne_msgs::VelodyneScan::ConstPtr& scan);
};

class VLP32CDecoder{
    private:
        constexpr static size_t kNUM_CHANNELS = 32;
        std::array<float, kNUM_CHANNELS> m_channelToSinVerticalAngle;
        std::array<float, kNUM_CHANNELS> m_channelToCosVerticalAngle;
        float m_azimuth;

    public:
        VLP32CDecoder();

        LidarScanStamped decode(const velodyne_msgs::VelodyneScan::ConstPtr& scan);
};


struct RacerDecoder
{
    static constexpr double IMU_TIME_LENIENCY = 0.1;

    static constexpr size_t LIDAR_SCAN_LINES = 32;
    static constexpr double LIDAR_ANGULAR_RESOLUTION = 0.003467542; // radians
    static constexpr size_t NUM_BLOCKS = 12;    // Number of blocks in a Velodyne packet

    // Lidar ID numbers
    static constexpr uint8_t PRIMARY_LIDAR_ID = 0;
    static constexpr uint8_t SECONDARY_LIDAR_ID = 1;
    static constexpr uint8_t AUXILIARY_LIDAR_ID = 2;
    
    // Elevation angle [rad]
    static constexpr float ELEVATION_ANGLE[LIDAR_SCAN_LINES] = {
        -0.43633231, -0.01745329, -0.02909464, -0.27295204, -0.19739674,  0.00000000, -0.01164135, -0.15433947,
        -0.12660618,  0.00581195, -0.00581195, -0.10730284, -0.09307841,  0.02326524,  0.01164135, -0.06981317,
        -0.08145452,  0.02909464,  0.01745329, -0.06400122, -0.05817182,  0.05817182,  0.04071853, -0.04654793,
        -0.05235988,  0.12217305,  0.08145452, -0.04071853, -0.03490659,  0.26179939,  0.18034487, -0.02326524
    };
    // static constexpr float COS_ELEVATION_ANGLE[LIDAR_SCAN_LINES] = {
    //     std::cos(ELEVATION_ANGLE[0]), std::cos(ELEVATION_ANGLE[1]), std::cos(ELEVATION_ANGLE[2]), std::cos(ELEVATION_ANGLE[3]),
    //     std::cos(ELEVATION_ANGLE[4]), std::cos(ELEVATION_ANGLE[5]), std::cos(ELEVATION_ANGLE[6]), std::cos(ELEVATION_ANGLE[7]), 
    //     std::cos(ELEVATION_ANGLE[8]), std::cos(ELEVATION_ANGLE[9]), std::cos(ELEVATION_ANGLE[10]), std::cos(ELEVATION_ANGLE[11]), 
    //     std::cos(ELEVATION_ANGLE[12]), std::cos(ELEVATION_ANGLE[13]), std::cos(ELEVATION_ANGLE[14]), std::cos(ELEVATION_ANGLE[15]), 
    //     std::cos(ELEVATION_ANGLE[16]), std::cos(ELEVATION_ANGLE[17]), std::cos(ELEVATION_ANGLE[18]), std::cos(ELEVATION_ANGLE[19]), 
    //     std::cos(ELEVATION_ANGLE[20]), std::cos(ELEVATION_ANGLE[21]), std::cos(ELEVATION_ANGLE[22]), std::cos(ELEVATION_ANGLE[23]), 
    //     std::cos(ELEVATION_ANGLE[24]), std::cos(ELEVATION_ANGLE[25]), std::cos(ELEVATION_ANGLE[26]), std::cos(ELEVATION_ANGLE[27]), 
    //     std::cos(ELEVATION_ANGLE[28]), std::cos(ELEVATION_ANGLE[29]), std::cos(ELEVATION_ANGLE[30]), std::cos(ELEVATION_ANGLE[31])
    // };
    // static constexpr float SIN_ELEVATION_ANGLE[LIDAR_SCAN_LINES] = {
    //     std::sin(ELEVATION_ANGLE[0]), std::sin(ELEVATION_ANGLE[1]), std::sin(ELEVATION_ANGLE[2]), std::sin(ELEVATION_ANGLE[3]),
    //     std::sin(ELEVATION_ANGLE[4]), std::sin(ELEVATION_ANGLE[5]), std::sin(ELEVATION_ANGLE[6]), std::sin(ELEVATION_ANGLE[7]), 
    //     std::sin(ELEVATION_ANGLE[8]), std::sin(ELEVATION_ANGLE[9]), std::sin(ELEVATION_ANGLE[10]), std::sin(ELEVATION_ANGLE[11]), 
    //     std::sin(ELEVATION_ANGLE[12]), std::sin(ELEVATION_ANGLE[13]), std::sin(ELEVATION_ANGLE[14]), std::sin(ELEVATION_ANGLE[15]), 
    //     std::sin(ELEVATION_ANGLE[16]), std::sin(ELEVATION_ANGLE[17]), std::sin(ELEVATION_ANGLE[18]), std::sin(ELEVATION_ANGLE[19]), 
    //     std::sin(ELEVATION_ANGLE[20]), std::sin(ELEVATION_ANGLE[21]), std::sin(ELEVATION_ANGLE[22]), std::sin(ELEVATION_ANGLE[23]), 
    //     std::sin(ELEVATION_ANGLE[24]), std::sin(ELEVATION_ANGLE[25]), std::sin(ELEVATION_ANGLE[26]), std::sin(ELEVATION_ANGLE[27]), 
    //     std::sin(ELEVATION_ANGLE[28]), std::sin(ELEVATION_ANGLE[29]), std::sin(ELEVATION_ANGLE[30]), std::sin(ELEVATION_ANGLE[31]) 
    // };
    // Physical offset of each laser from the reported azimuth [rad]
    static constexpr float AZIMUTH_FIXED_OFFSET[LIDAR_SCAN_LINES] {
        0.02443461, -0.07330383,  0.02443461, -0.02443461,  0.02443461, -0.02443461,  0.07330383, -0.02443461,
        0.02443461, -0.07330383,  0.02443461, -0.02443461,  0.07330383, -0.02443461,  0.07330383, -0.02443461,
        0.02443461, -0.07330383,  0.02443461, -0.07330383,  0.07330383, -0.02443461,  0.02443461, -0.02443461,
        0.02443461, -0.02443461,  0.02443461, -0.07330383,  0.07330383, -0.02443461,  0.02443461, -0.02443461
    };
    // Additional azimuth offset by which each laser will have rotated by the time it fires [rad]
    static constexpr float AZIMUTH_TIME_OFFSET[LIDAR_SCAN_LINES] {
        0.00000000,  0.00000000,  0.00014476,  0.00014476,  0.00028953,  0.00028953,  0.00043429,  0.00043429,
        0.00057906,  0.00057906,  0.00072382,  0.00072382,  0.00086859,  0.00086859,  0.00101335,  0.00101335,
        0.00115812,  0.00115812,  0.00130288,  0.00130288,  0.00144765,  0.00144765,  0.00159241,  0.00159241,
        0.00173718,  0.00173718,  0.00188194,  0.00188194,  0.00202670,  0.00202670,  0.00217147,  0.00217147
    };
    
    static constexpr uint8_t LASER_ORDERING[LIDAR_SCAN_LINES] {
        0, 3, 4, 7, 8, 11, 12, 16, 15, 19, 20, 24, 23, 27, 28, 2, 31, 1, 6, 10, 5, 9, 14, 18, 13, 17, 22, 21, 26, 25, 30, 29
    };

    static constexpr unsigned int BLOCK_TIME_NS = 55296;   // Time in ns for one block (measurement + recharge)
    static constexpr unsigned int FIRING_TIME_NS = 2304;
    static constexpr double LIDAR_MESSAGE_TIME = (double)(NUM_BLOCKS * BLOCK_TIME_NS * 151) * 1e-9;
    static constexpr double LIDAR_TIME_EPSILON = 0.05 * LIDAR_MESSAGE_TIME;
    static constexpr float CDEG_TO_RAD = 0.01*M_PI/180.0;  // Constant for converting hundredths of a degree to radians
    static constexpr float RANGE_TO_M = 4.0/1000.0;        // Convert range from 4mm units to meters

    Eigen::Array<float, LIDAR_SCAN_LINES, 1> m_cosElevationAngles;
    Eigen::Array<float, LIDAR_SCAN_LINES, 1> m_sinElevationAngles;

    RacerDecoder();

    std::vector<PointXYZICT> decode(
    const velodyne_msgs::VelodyneScan::ConstPtr& scan);

};

} // namespace velodyne_decoder
