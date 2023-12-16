#include "velodyne_decoder/vlp16.hpp"

#include "velodyne_decoder/velodyne_decoder_ros.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_msgs/VelodyneScan.h>

int main(int argc, char *argv[]) {
  const std::string nodeName = "vlp16_node";
  ros::init(argc, argv, nodeName);

  ros::NodeHandle publicNode;
  ros::NodeHandle privateNode("~");

  std::string packetTopic;
  if (!privateNode.getParam("packet_topic", packetTopic)) {
    std::cerr << "Could not get param " << nodeName << "/packet_topic"
              << std::endl;
    ros::requestShutdown();
  }

  const std::string cloudTopic = packetTopic + "/point_cloud";

  auto cloudPublisher =
      privateNode.advertise<sensor_msgs::PointCloud2>(cloudTopic, 2);

  velodyne_decoder::vlp16::VelodyneDecoder decoder;
  velodyne_decoder::RacerDecoder racerDecoder;
  velodyne_decoder::VLP16Decoder newDecoder;

  auto velodyneSubscriber = publicNode.subscribe<velodyne_msgs::VelodyneScan>(
      packetTopic, 1,
      [&cloudPublisher,
       &decoder, &newDecoder](const velodyne_msgs::VelodyneScan::ConstPtr &msg) {
        sensor_msgs::PointCloud2 cloud;
        // const auto packets = std::ranges::views::transform(
        //     msg->packets, [](const auto &packet) { return packet.data.elems; });
        const velodyne_decoder::LidarScanStamped scan =
            newDecoder.decode(msg);
            // decoder.decode(packets);
        velodyne_decoder::toMsg(scan, cloud);
        cloud.header = msg->header;
        cloudPublisher.publish(cloud);
      });

  ros::spin();
  return 0;
}
