//
// Created by lsy on 24-5-23.
//

#pragma once

#include <effort_controllers/joint_effort_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/imu_sensor_interface.h>

//robot_common
#include "robot_common/interface/hardware_interface/robot_state_interface.h"
#include "robot_common/utilities/ori_tool.h"
#include "robot_common/utilities/tf_rt_broadcaster.h"

#include "geometry_msgs/Twist.h"
namespace rl_controller
{
class WheeledBipedalRLController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,hardware_interface::ImuSensorInterface>
{
  enum ControllerState {
    NORMAL
  };
public:
  WheeledBipedalRLController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  void normal(const ros::Time& time, const ros::Duration& period);
  void commandCB(const geometry_msgs::Twist& msg);

  int controllerState_ = NORMAL;
  ros::Time startTime_;
  bool stateChanged_ = false;
  ros::Subscriber cmdSub_;

  // Interface
  hardware_interface::RobotStateHandle robotStateHandle_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;
  std::vector<hardware_interface::JointHandle> jointHandles_;
  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmdRtBuffer_{};
};

}  // namespace rl_controller