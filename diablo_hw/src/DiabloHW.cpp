//
// Created by lsy on 24-9-23.
//

#include "diablo_hw/DiabloHW.h"

namespace diablo {
bool DiabloHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& /*robot_hw_nh*/) {
  if (!loadUrdf(root_nh)) {
    ROS_ERROR("Error occurred while setting up urdf");
    return false;
  }

  registerInterface(&jointStateInterface_);
  registerInterface(&effortJointInterface_);
  registerInterface(&imuSensorInterface_);

  return true;
}

bool DiabloHW::loadUrdf(ros::NodeHandle& rootNh) {
  std::string urdfString;
  if (urdfModel_ == nullptr) {
    urdfModel_ = std::make_shared<urdf::Model>();
  }
  // get the urdf param on param server
  rootNh.getParam("diablo_robot_description", urdfString);
  return !urdfString.empty() && urdfModel_->initString(urdfString);
}

}  // namespace diablo