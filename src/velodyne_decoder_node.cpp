#include "velodyne_decoder/velodyne_decoder_ros.hpp"

#include "velodyne_decoder/vlp16.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_msgs/VelodyneScan.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "velodyne_decoder_node");

  ros::NodeHandle publicNode;
  ros::NodeHandle privateNode("~");

  std::string velodyneTopic;
  if (!privateNode.getParam("velodyne_topic", velodyneTopic)) {
    std::cerr << "Could not get param velodyne_topic" << std::endl;
    ros::requestShutdown();
  }

  const std::string cloudTopic = velodyneTopic + "/point_cloud";

  auto cloudPublisher =
      privateNode.advertise<sensor_msgs::PointCloud2>(cloudTopic, 2);

  velodyne_decoder::vlp16::VelodyneDecoder decoder;

  auto velodyneSubscriber = publicNode.subscribe<velodyne_msgs::VelodyneScan>(
      velodyneTopic, 1,
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