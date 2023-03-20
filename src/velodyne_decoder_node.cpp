#include "velodyne_decoder/velodyne_decoder_ros.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_msgs/VelodyneScan.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "velodyne_decoder_node");

  ros::NodeHandle publicNode;
  ros::NodeHandle privateNode("~");

  std::string velodyneTopic;
  if (!privateNode.getParam("velodyne_topic", velodyneTopic)) {
    ros::requestShutdown();
  }

  std::string cloudTopic;
  if (!privateNode.getParam("cloud_topic", cloudTopic)) {
    ros::requestShutdown();
  }
  auto cloudPublisher =
      privateNode.advertise<sensor_msgs::PointCloud2>(cloudTopic, 2);

  auto velodyneSubscriber = publicNode.subscribe<velodyne_msgs::VelodyneScan>(
      velodyneTopic, 1,
      [&cloudPublisher](const velodyne_msgs::VelodyneScan::ConstPtr &msg) {
        sensor_msgs::PointCloud2 cloud;
        velodyne_decoder::decode(msg, cloud);
        cloudPublisher.publish(cloud);
      });

  ros::spin();
  return 0;
}