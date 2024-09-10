//
// Created by lsy on 24-5-23.
//

#include <string>
#include <pluginlib/class_list_macros.hpp>

#include "wheeled_bipedal_rl_controller/WheeledBipedalRLController.h"

namespace rl_controller
{
bool WheeledBipedalRLController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                              ros::NodeHandle& controller_nh)
{
  // Hardware interface
  robotStateHandle_ = robot_hw->get<hardware_interface::RobotStateInterface>()->getHandle("robot_state");
  auto* effortJointInterface = robot_hw->get<hardware_interface::EffortJointInterface>();
  jointHandles_.push_back(effortJointInterface->getHandle("joint1"));
  jointHandles_.push_back(effortJointInterface->getHandle("joint2"));
  jointHandles_.push_back(effortJointInterface->getHandle("joint3"));
  jointHandles_.push_back(effortJointInterface->getHandle("joint4"));
  jointHandles_.push_back(effortJointInterface->getHandle("joint5"));
  jointHandles_.push_back(effortJointInterface->getHandle("joint6"));
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");
  cmdSub_ = controller_nh.subscribe("command", 1, &WheeledBipedalRLController::commandCB, this);
  return true;
}

void WheeledBipedalRLController::starting(const ros::Time& /*unused*/)
{
//  state_ = NORMAL;
}

void WheeledBipedalRLController::update(const ros::Time& time, const ros::Duration& period)
{
  //  if (state_ !=  geometry_msgs::Twist.mode)
  //  {
  //    state_ =  geometry_msgs::Twist.mode;
  //    stateChanged_ = true;
  //  }
//  switch (state_)
//  {
//    case rl_controller::NORMAL:
//      normal(time, period);
//      break;
//  }
}

void WheeledBipedalRLController::normal(const ros::Time& time, const ros::Duration& period)
{
  ROS_INFO_STREAM("normal Mode");
}

void WheeledBipedalRLController::commandCB(const  geometry_msgs::Twist& msg)
{
  cmdRtBuffer_.writeFromNonRT(msg);
}

}  // namespace rl_controller

PLUGINLIB_EXPORT_CLASS(rl_controller::WheeledBipedalRLController, controller_interface::ControllerBase)