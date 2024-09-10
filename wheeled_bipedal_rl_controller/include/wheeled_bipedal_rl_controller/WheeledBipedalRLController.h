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

// robot_common
#include "robot_common/interface/hardware_interface/robot_state_interface.h"
#include "robot_common/utilities/ori_tool.h"
#include "robot_common/utilities/tf_rt_broadcaster.h"

#include "geometry_msgs/Twist.h"

// Gazebo
#include <gazebo_msgs/SetModelState.h>
#include "std_srvs/Empty.h"

// rl
#include <torch/script.h>
#include "rl_sdk/rl_sdk.hpp"
#include "observation_buffer.hpp"

namespace rl_controller
{
class WheeledBipedalRLController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,hardware_interface::ImuSensorInterface>
{
  enum ControllerState {
    NORMAL,
    RL
  };
public:
  WheeledBipedalRLController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void setCommand();
  void setRLState();

private:
  void normal(const ros::Time& time, const ros::Duration& period);
  void rl(const ros::Time& time, const ros::Duration& period);
  void commandCB(const geometry_msgs::Twist& msg);

  int controllerState_ = NORMAL;
  ros::Time startTime_;
  bool stateChanged_ = false;
  ros::Subscriber cmdSub_;

  // Interface
  hardware_interface::RobotStateHandle robotStateHandle_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;
  std::vector<hardware_interface::JointHandle> jointHandles_;

  // RL
  rl_sdk rlActing_;

  // history buffer
  std::shared_ptr<torch::Tensor> history_obs_ptr_;
  std::shared_ptr<ObservationBuffer> history_obs_buf_ptr_;

  //  Gazebo Service
  std::string gazebo_model_name_;
  ros::ServiceClient gazebo_set_model_state_client_;
  ros::ServiceClient gazebo_pause_physics_client_;
  ros::ServiceClient gazebo_unpause_physics_client_;

  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmdRtBuffer_{};
};

}  // namespace rl_controller