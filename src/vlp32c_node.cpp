#include "velodyne_decoder/vlp32c.hpp"

#include "velodyne_decoder/velodyne_decoder_ros.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_msgs/VelodyneScan.h>

int main(int argc, char *argv[]) {
  const std::string nodeName = "vlp32c_node";
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

  velodyne_decoder::vlp32c::VelodyneDecoder decoder;

  auto velodyneSubscriber = publicNode.subscribe<velodyne_msgs::VelodyneScan>(
      packetTopic, 1,
      [&cloudPublisher,
       &decoder](const velodyne_msgs::VelodyneScan::ConstPtr &msg) {
        sensor_msgs::PointCloud2 cloud;
        std::vector<velodyne_decoder::PointXYZICT> points;
        std::ranges::for_each(
            msg->packets, [&decoder, &points](const auto &packet) {
              decoder.appendToCloud(packet.data.elems, points);
            });
        velodyne_decoder::toMsg(points, cloud);
        cloud.header = msg->header;
        cloudPublisher.publish(cloud);
      });

  ros::spin();
  return 0;
}