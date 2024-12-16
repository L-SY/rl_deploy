//
// Created by lsy on 24-9-12.
//

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ps5_joy/ps5Joy.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "joy_twist");
  ros::NodeHandle nh;

  joy::PS5Joy ps5_joy(nh);

  std::string topic_name;
  int frequency;
  nh.param("twist_topic", topic_name, std::string("/cmd_vel"));
  nh.param("frequency", frequency, 100);
  ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>(topic_name, 10);

  ros::Rate rate(frequency);

  while (ros::ok()) {
    geometry_msgs::Twist twist_msg;

    twist_msg.linear.x = 5*ps5_joy.getAxisValue(joy::PS5ButtonMap::L3Vertical);  // 左摇杆前后控制x速度
    twist_msg.angular.z = 3.14*ps5_joy.getAxisValue(joy::PS5ButtonMap::L3Horizontal);  // 左摇杆左右控制yaw速度
    twist_msg.linear.z = 0.4*ps5_joy.getAxisValue(joy::PS5ButtonMap::R3Vertical);  // 右摇杆前后控制z速度
    ROS_INFO_STREAM(twist_msg.linear.x);
    twist_pub.publish(twist_msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
