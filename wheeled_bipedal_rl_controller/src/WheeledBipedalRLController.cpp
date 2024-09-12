//
// Created by lsy on 24-5-23.
//

#include <string>
#include <pluginlib/class_list_macros.h>
#include "wheeled_bipedal_rl_controller/WheeledBipedalRLController.h"
#include <torch/script.h>

namespace rl_controller
{
bool WheeledBipedalRLController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                              ros::NodeHandle& controller_nh)
{
  // read params from yaml
  controller_nh.param<std::string>("robot_name", rlActing_.robot_name, "");
  std::string rl_path;
  controller_nh.param<std::string>("rl_path", rl_path, "");
  std::string config_path = std::string(rl_path + "/config.yaml");
  rlActing_.ReadYaml(config_path);

  // model
  std::string model_path = std::string(rl_path + "/" + rlActing_.params.model_name);
  rlActing_.model = torch::jit::load(model_path);
//  rlActing_.model.dump(true,false,false);

  // init
  torch::autograd::GradMode::set_enabled(false);
  rlActing_.InitObservations();
  rlActing_.InitOutputs();
  rlActing_.InitControl();
  //  for (size_t i = 0; i < jointHandles_.size(); ++i) {
  //    auto pid =
  //    Pids_.push_back()
  //  }

  // history
  history_obs_ptr_ = std::make_shared<torch::Tensor>(torch::zeros({rlActing_.params.num_observations}));

  if(rlActing_.params.use_history)
  {
    history_obs_buf_ptr_ = std::make_shared<ObservationBuffer>(1, rlActing_.params.num_observations, 3);
  }
  // Hardware interface
  auto* effortJointInterface = robot_hw->get<hardware_interface::EffortJointInterface>();
  jointHandles_.push_back(effortJointInterface->getHandle("left_hip_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("left_knee_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("left_wheel_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("right_hip_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("right_knee_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("right_wheel_joint"));
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");
  robotStateHandle_ = robot_hw->get<hardware_interface::RobotStateInterface>()->getHandle("robot_state");
  // gazebo_service
  controller_nh.param<std::string>("gazebo_model_name", gazebo_model_name_, "");
  gazebo_set_model_state_client_ = controller_nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_pause_physics_client_ = controller_nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  gazebo_unpause_physics_client_ = controller_nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

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
  std::string path = "/home/lsy/rl_ws/src/assets/diablo/models/policy_27.pt";

  torch::jit::script::Module model;
  try {
    model = torch::jit::load(path);
  } catch (const c10::Error& e) {
    std::cerr << "Error loading the model.\n";
  }

  std::cout << "Model loaded successfully.\n";

  torch::Tensor input_tensor = torch::rand({1, 27});
  std::cout << "input_tensor" << input_tensor.sizes() << std::endl;

  torch::Tensor output_tensor;
  try {
    output_tensor = model.forward({input_tensor}).toTensor();
  } catch (const c10::Error& e) {
    std::cerr << "Error during forward pass.\n";
  }

  std::cout << "Output tensor: " << output_tensor << std::endl;
  rl(time,period);
}

void WheeledBipedalRLController::normal(const ros::Time& time, const ros::Duration& period)
{
  ROS_INFO_STREAM("normal Mode");
}

void WheeledBipedalRLController::rl(const ros::Time& time, const ros::Duration& period)
{
//TODO ：add set control.x, control.y, control.yaw.
  rlActing_.control.vel_x = 0.;
  rlActing_.control.vel_yaw = 0.;
  rlActing_.control.pos_z = 0.20;
  setRLState();
  rlActing_.SetObservation();

  torch::Tensor clamped_actions = rlActing_.Forward();

  rlActing_.obs.actions = clamped_actions;

  torch::Tensor origin_output_torques = rlActing_.ComputeTorques(rlActing_.obs.actions);

  // rlActing_.TorqueProtect(origin_output_torques);

  rlActing_.output_torques = torch::clamp(origin_output_torques, -(rlActing_.params.torque_limits), rlActing_.params.torque_limits);
  rlActing_.output_dof_pos = rlActing_.ComputePosition(rlActing_.obs.actions);
  rlActing_.output_dof_vel = rlActing_.ComputeVelocity(rlActing_.obs.actions);
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

void WheeledBipedalRLController::setCommand()
{
  for (size_t i = 0; i < jointHandles_.size(); ++i) {
    jointHandles_[i].setCommand(rlActing_.output_dof_pos[0][i].item<double>());
  }
}
}  // namespace rl_controller

PLUGINLIB_EXPORT_CLASS(rl_controller::WheeledBipedalRLController, controller_interface::ControllerBase)