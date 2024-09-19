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

  // init
  initStateMsg();

  // Low-level-controller
  Pids_.resize(jointHandles_.size());
  for (size_t i = 0; i < jointHandles_.size(); ++i)
  {
    ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + jointHandles_[i].getName());
    Pids_[i].reset();
    if (!Pids_[i].init(joint_nh))
    {
      ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
      return false;
    }
  }

  // rl_interface
  robotStatePub_ = controller_nh.advertise<rl_msgs::RobotState>("/rl/robot_state", 10);
  rlCommandSub_ = controller_nh.subscribe("/rl/command", 1, &WheeledBipedalRLController::rlCommandCB, this);

  controller_nh.param("default_length", default_length_, 0.18);

  // prostrate
  ros::NodeHandle prostrate_nh(controller_nh, "prostrate");
  prostrate_nh.param("hip", prostrateHip_, 0.);
  prostrate_nh.param("knee", prostrateKnee_, 0.);

  // vmc
  geometry_msgs::Twist initTwist;
  initTwist.linear.x = 0.05;
  initTwist.linear.y = 0.0;
  initTwist.linear.z = default_length_;
  initTwist.angular.x = 0.0;
  initTwist.angular.y = 0.0;
  initTwist.angular.z = 0.0;
  cmdRtBuffer_.initRT(initTwist);
  return true;
}

void WheeledBipedalRLController::starting(const ros::Time& /*unused*/)
{
  ROS_INFO_STREAM("WheeledBipedalRLController Starting!");
  controllerState_ = NORMAL;
}

void WheeledBipedalRLController::update(const ros::Time& time, const ros::Duration& period)
{
//  rl(time,period);
  prostrate(time,period);
  pubRLState();
}

void WheeledBipedalRLController::prostrate (const ros::Time& time, const ros::Duration& period)
{
  auto rt_buffer = cmdRtBuffer_.readFromRT();

  jointHandles_[0].setCommand(Pids_[0].computeCommand(prostrateHip_ - jointHandles_[0].getPosition(),period));
  jointHandles_[1].setCommand(Pids_[1].computeCommand(prostrateKnee_ - jointHandles_[1].getPosition(),period));
  jointHandles_[3].setCommand(Pids_[3].computeCommand(prostrateHip_ - jointHandles_[3].getPosition(),period));
  jointHandles_[4].setCommand(Pids_[4].computeCommand(prostrateKnee_ - jointHandles_[4].getPosition(),period));
  // wheel
  double Vx = rt_buffer->linear.x;
  double Vyaw = rt_buffer->angular.z;
  double Vleft = Vx / 2 - Vyaw;
  double Vright = Vx / 2 + Vyaw;
  jointHandles_[2].setCommand(Pids_[2].computeCommand(Vleft-jointHandles_[2].getVelocity(),period));
  jointHandles_[5].setCommand(Pids_[5].computeCommand(Vright-jointHandles_[5].getVelocity(),period));

  ROS_INFO_STREAM("prostrate  Mode");
}

void WheeledBipedalRLController::rl(const ros::Time& time, const ros::Duration& period)
{
//TODO ：add set control.x, control.y, control.yaw.
  setCommand(period);
}

void WheeledBipedalRLController::commandCB(const geometry_msgs::Twist& msg)
{
  cmdRtBuffer_.writeFromNonRT(msg);
}

void WheeledBipedalRLController::rlCommandCB(const std_msgs::Float64MultiArray&  msg)
{
  rlCmdRtBuffer_.writeFromNonRT(msg);
}

void WheeledBipedalRLController::pubRLState()
{
  robotStateMsg_.imu_states.header.frame_id = imuSensorHandle_.getName();
  robotStateMsg_.imu_states.orientation.x = imuSensorHandle_.getOrientation()[0];
  robotStateMsg_.imu_states.orientation.y = imuSensorHandle_.getOrientation()[1];
  robotStateMsg_.imu_states.orientation.z = imuSensorHandle_.getOrientation()[2];
  robotStateMsg_.imu_states.orientation.w = imuSensorHandle_.getOrientation()[3];
  robotStateMsg_.imu_states.angular_velocity.x = imuSensorHandle_.getAngularVelocity()[0];
  robotStateMsg_.imu_states.angular_velocity.y = imuSensorHandle_.getAngularVelocity()[1];
  robotStateMsg_.imu_states.angular_velocity.z = imuSensorHandle_.getAngularVelocity()[2];
  robotStateMsg_.imu_states.linear_acceleration.x = imuSensorHandle_.getLinearAcceleration()[0];
  robotStateMsg_.imu_states.linear_acceleration.y = imuSensorHandle_.getLinearAcceleration()[1];
  robotStateMsg_.imu_states.linear_acceleration.z = imuSensorHandle_.getLinearAcceleration()[2];

  for(size_t i = 0; i < jointHandles_.size(); ++i)
  {
    robotStateMsg_.joint_states.name[i] = jointHandles_[i].getName();
    robotStateMsg_.joint_states.position[i] = jointHandles_[i].getPosition();
    robotStateMsg_.joint_states.velocity[i] = jointHandles_[i].getVelocity();
    robotStateMsg_.joint_states.effort[i] = jointHandles_[i].getEffort();
  }

  auto cmdBuff = cmdRtBuffer_.readFromRT();
  double linear_x = cmdBuff->linear.x;
  double angular_z = cmdBuff->angular.z;
  double linear_z = cmdBuff->linear.z;

  if (linear_x != 0.0 || angular_z != 0.0 || linear_z != 0.0)
  {
    robotStateMsg_.commands[0] = linear_x;
    robotStateMsg_.commands[1] = angular_z;
    robotStateMsg_.commands[2] = default_length_ + linear_z;
  }
  else
  {
    robotStateMsg_.commands[0] = 0;
    robotStateMsg_.commands[1] = 0;
    robotStateMsg_.commands[2] = default_length_;
  }
  robotStatePub_.publish(robotStateMsg_);
}

void WheeledBipedalRLController::setCommand(const ros::Duration& period)
{
  auto rt_buffer = rlCmdRtBuffer_.readFromRT();
  const auto& data = rt_buffer->data;

  if (data.empty())
  {
    for (auto& jointHandle : jointHandles_) {
      jointHandle.setCommand(0.0);
    }
  }
  else
  {
    jointHandles_[0].setCommand(Pids_[0].computeCommand(data[0]-jointHandles_[0].getPosition(),period));
    jointHandles_[1].setCommand(Pids_[1].computeCommand(data[1]-jointHandles_[1].getPosition(),period));
    jointHandles_[3].setCommand(Pids_[3].computeCommand(data[3]-jointHandles_[3].getPosition(),period));
    jointHandles_[4].setCommand(Pids_[4].computeCommand(data[4]-jointHandles_[4].getPosition(),period));

//    TODO: 20 should be add in rl_interface
    jointHandles_[2].setCommand(Pids_[2].computeCommand(data[2]*20-jointHandles_[2].getVelocity(),period));
    jointHandles_[5].setCommand(Pids_[5].computeCommand(data[5]*20-jointHandles_[5].getVelocity(),period));
//    jointHandles_[2].setCommand(Pids_[2].computeCommand(data[2]-jointHandles_[2].getVelocity(),period));
//    jointHandles_[5].setCommand(Pids_[5].computeCommand(data[5]-jointHandles_[5].getVelocity(),period));
  }
}

void WheeledBipedalRLController::initStateMsg()
{
  robotStateMsg_.imu_states.header.frame_id = "";
  robotStateMsg_.imu_states.orientation.x = 0.0;
  robotStateMsg_.imu_states.orientation.y = 0.0;
  robotStateMsg_.imu_states.orientation.z = 0.0;
  robotStateMsg_.imu_states.orientation.w = 0.0;
  robotStateMsg_.imu_states.angular_velocity.x = 0.0;
  robotStateMsg_.imu_states.angular_velocity.y = 0.0;
  robotStateMsg_.imu_states.angular_velocity.z = 0.0;
  robotStateMsg_.imu_states.linear_acceleration.x = 0.0;
  robotStateMsg_.imu_states.linear_acceleration.y = 0.0;
  robotStateMsg_.imu_states.linear_acceleration.z = 0.0;

  size_t num_joints = jointHandles_.size();
  robotStateMsg_.joint_states.name.clear();
  robotStateMsg_.joint_states.position.clear();
  robotStateMsg_.joint_states.velocity.clear();
  robotStateMsg_.joint_states.effort.clear();
  robotStateMsg_.joint_states.name.resize(num_joints);
  robotStateMsg_.joint_states.position.resize(num_joints);
  robotStateMsg_.joint_states.velocity.resize(num_joints);
  robotStateMsg_.joint_states.effort.resize(num_joints);

  robotStateMsg_.commands.clear();
  robotStateMsg_.commands.resize(3);
  robotStateMsg_.commands[0] = 0.0;
  robotStateMsg_.commands[2] = default_length_; // init_pos_z
}

}  // namespace rl_controller

PLUGINLIB_EXPORT_CLASS(rl_controller::WheeledBipedalRLController, controller_interface::ControllerBase)