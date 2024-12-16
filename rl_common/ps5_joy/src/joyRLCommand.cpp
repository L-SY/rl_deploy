//
// Created by lsy on 24-9-12.
//

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "ps5_joy/ps5Joy.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "joy_rl");
  ros::NodeHandle nh;

  joy::PS5Joy ps5_joy(nh);

  std::string topic_name;
  int frequency;
  nh.param("rl_topic", topic_name, std::string("/rl/command"));
  nh.param("frequency", frequency, 100);
  ros::Publisher rl_pub = nh.advertise<std_msgs::Float64MultiArray>(topic_name, 10);

  ros::Rate rate(frequency);

  while (ros::ok()) {
    std_msgs::Float64MultiArray rl_msg;
    rl_msg.data.resize(6);
    rl_msg.data[0] = ps5_joy.getAxisValue(joy::PS5ButtonMap::R3Horizontal);
    rl_msg.data[1] = ps5_joy.getAxisValue(joy::PS5ButtonMap::L3Vertical);
    rl_msg.data[3] = ps5_joy.getAxisValue(joy::PS5ButtonMap::R3Horizontal);
    rl_msg.data[4] = ps5_joy.getAxisValue(joy::PS5ButtonMap::L3Vertical);

    rl_msg.data[2] = 0;
    rl_msg.data[5] = 0;
    rl_pub.publish(rl_msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
