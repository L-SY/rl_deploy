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

  // read params from yaml
  controller_nh.param<std::string>("robot_name", rlActing_.robot_name, "");
  rlActing_.ReadYaml(rlActing_.robot_name);

  // history
  history_obs_ptr_ = std::make_shared<torch::Tensor>(torch::zeros({rlActing_.params.num_observations}));
  if(rlActing_.params.use_history)
  {
    history_obs_buf_ptr_ = std::make_shared<ObservationBuffer>(1, rlActing_.params.num_observations, 6);
  }

  // init
  torch::autograd::GradMode::set_enabled(false);
  rlActing_.InitObservations();
  rlActing_.InitOutputs();
  rlActing_.InitControl();

  // model
  std::string model_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + rlActing_.robot_name + "/" + rlActing_.params.model_name;
  rlActing_.model = torch::jit::load(model_path);

  // gazebo_service
  controller_nh.param<std::string>("gazebo_model_name", gazebo_model_name_, "");
  gazebo_set_model_state_client_ = controller_nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_pause_physics_client_ = controller_nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  gazebo_unpause_physics_client_ = controller_nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

  // Hardware interface
  robotStateHandle_ = robot_hw->get<hardware_interface::RobotStateInterface>()->getHandle("robot_state");
  auto* effortJointInterface = robot_hw->get<hardware_interface::EffortJointInterface>();
  jointHandles_.push_back(effortJointInterface->getHandle("left_hip_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("left_knee_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("left_wheel_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("right_hip_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("right_knee_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("right_wheel_joint"));
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");
  cmdSub_ = controller_nh.subscribe("command", 1, &WheeledBipedalRLController::commandCB, this);
  return true;
}

void WheeledBipedalRLController::starting(const ros::Time& /*unused*/)
{
  controllerState_ = NORMAL;
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

void WheeledBipedalRLController::rl(const ros::Time& time, const ros::Duration& period)
{
//TODO ：add set control.x, control.y, control.yaw.
  setRLState();
  rlActing_.SetObservation();

  torch::Tensor clamped_actions = rlActing_.Forward(history_obs_ptr_, history_obs_buf_ptr_);

  rlActing_.obs.actions = clamped_actions;

  torch::Tensor origin_output_torques = rlActing_.ComputeTorques(rlActing_.obs.actions);

  // rlActing_.TorqueProtect(origin_output_torques);

  rlActing_.output_torques = torch::clamp(origin_output_torques, -(rlActing_.params.torque_limits), rlActing_.params.torque_limits);
  rlActing_.output_dof_pos = rlActing_.ComputePosition(rlActing_.obs.actions);
  setCommand();
}

void WheeledBipedalRLController::commandCB(const  geometry_msgs::Twist& msg)
{
  cmdRtBuffer_.writeFromNonRT(msg);
}

void WheeledBipedalRLController::setRLState()
{
  auto quaternion = imuSensorHandle_.getOrientation();
  auto gyroscope = imuSensorHandle_.getAngularVelocity();
  if(rlActing_.params.framework == "isaacgym")
  {
    rlActing_.robot_state.imu.quaternion[3] = quaternion[3];
    rlActing_.robot_state.imu.quaternion[0] = quaternion[0];
    rlActing_.robot_state.imu.quaternion[1] = quaternion[1];
    rlActing_.robot_state.imu.quaternion[2] = quaternion[2];
  }
  else if(rlActing_.params.framework == "isaacsim")
  {
    rlActing_.robot_state.imu.quaternion[0] = quaternion[3];
    rlActing_.robot_state.imu.quaternion[1] = quaternion[0];
    rlActing_.robot_state.imu.quaternion[2] = quaternion[1];
    rlActing_.robot_state.imu.quaternion[3] = quaternion[2];
  }

  rlActing_.robot_state.imu.gyroscope[0] = gyroscope[0];
  rlActing_.robot_state.imu.gyroscope[1] = gyroscope[1];
  rlActing_.robot_state.imu.gyroscope[2] = gyroscope[2];

  // The order is determined by the Hardware interface in init.
  for(int i = 0; i < rlActing_.params.num_of_dofs; ++i)
  {
    rlActing_.robot_state.motor_state.q[i] = jointHandles_[i].getPosition();
    rlActing_.robot_state.motor_state.dq[i] = jointHandles_[i].getVelocity();
    rlActing_.robot_state.motor_state.tauEst[i] = jointHandles_[i].getEffort();
  }
}

}  // namespace rl_controller
PLUGINLIB_EXPORT_CLASS(rl_controller::WheeledBipedalRLController, controller_interface::ControllerBase)