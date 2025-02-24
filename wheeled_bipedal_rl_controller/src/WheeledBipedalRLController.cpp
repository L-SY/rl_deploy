//
// Created by lsy on 24-5-23.
//

#include <string>
#include <pluginlib/class_list_macros.h>
#include "wheeled_bipedal_rl_controller/WheeledBipedalRLController.h"

namespace rl_controller
{
bool WheeledBipedalRLController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                              ros::NodeHandle& controller_nh)
{
  controller_nh.param("simulation", simulation_, false);
  // Hardware interface
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");
  robotStateHandle_ = robot_hw->get<hardware_interface::RobotStateInterface>()->getHandle("robot_state");

  // gazebo_service
  controller_nh.param<std::string>("gazebo_model_name", gazebo_model_name_, "");
  gazebo_set_model_state_client_ = controller_nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_pause_physics_client_ = controller_nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  gazebo_unpause_physics_client_ = controller_nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

  cmdSub_ = controller_nh.subscribe("command", 1, &WheeledBipedalRLController::commandCB, this);

  // rl_interface
  robotStatePub_ = controller_nh.advertise<rl_msgs::RobotState>("/rl/robot_state", 1);
  if (simulation_)
    simRobotStatePub_ = controller_nh.advertise<rl_msgs::RobotState>("/rl/simulation/robot_state", 1);
  rlCommandSub_ = controller_nh.subscribe("/rl/command", 1, &WheeledBipedalRLController::rlCommandCB, this);

  controller_nh.param("default_length", default_length_, 0.18);

  // prostrate
  ros::NodeHandle prostrate_nh(controller_nh, "prostrate");
  prostrate_nh.param("hip", prostrateHip_, 0.);
  prostrate_nh.param("knee", prostrateKnee_, 0.);

  // gravity_ff
  controller_nh.param("add_gravity_ff", addGravityFF_, false);

  // vmc
  ros::NodeHandle vmc_nh(controller_nh, "vmc");
  vmc_nh.param("use_vmc", useVMC_, false);
  if (useVMC_ || addGravityFF_)
  {
    std::string left_type, right_type;
    double left_l1, left_l2, right_l1, right_l2;
    double left_centre_offset, right_centre_offset;
    vmc_nh.param("hip_bias", hipBias_, 0.);
    vmc_nh.param("knee_bias", kneeBias_, 0.);
    vmc_nh.param("gravity_feedforward", gravityFeedforward_, 50.0);

    vmc_nh.param("left_vmc/l1", left_l1, 0.14);
    vmc_nh.param("left_vmc/l2", left_l2, 0.14);
    vmc_nh.param("right_vmc/l1", right_l1, 0.14);
    vmc_nh.param("right_vmc/l2", right_l2, 0.14);
    vmc_nh.param("left_vmc/type", left_type, std::string("serial"));
    vmc_nh.param("right_vmc/type", right_type, std::string("serial"));
    vmc_nh.param("left_vmc/centre_offset", left_centre_offset, 0.0);
    vmc_nh.param("right_vmc/centre_offset", right_centre_offset, 0.0);

    ros::NodeHandle vmc_left_nh(controller_nh, std::string("vmc_left"));
    ros::NodeHandle vmc_right_nh(controller_nh, std::string("vmc_left"));
    leftSerialVMCPtr_ = std::make_shared<vmc::SerialVMC>(left_l1,left_l2, vmc_left_nh);
    rightSerialVMCPtr_ = std::make_shared<vmc::SerialVMC>(right_l1,right_l2, vmc_right_nh);
    if (useVMC_)
    {
      VMCPids_.resize(4);
      std::vector<std::string> VMCNames = {"left_r", "left_theta", "right_r", "right_theta"};
      for (size_t i = 0; i < 4; ++i)
      {
        ros::NodeHandle vmc_pid_nh(vmc_nh, std::string("gains/") + VMCNames[i]);
        VMCPids_[i].reset();
        if (!VMCPids_[i].init(vmc_pid_nh))
        {
          ROS_WARN_STREAM("Failed to initialize VMC PID gains from ROS parameter server.");
          return false;
        }
      }
    }
  }

  auto* effortJointInterface = robot_hw->get<hardware_interface::EffortJointInterface>();
  jointHandles_.push_back(effortJointInterface->getHandle("left_fake_hip_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("left_hip_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("left_wheel_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("right_fake_hip_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("right_hip_joint"));
  jointHandles_.push_back(effortJointInterface->getHandle("right_wheel_joint"));

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
  ROS_INFO_STREAM("Load lower-level controller right");

  // init
  initStateMsg();
//  double lp_cutoff_frequency;
//  controller_nh.param("lp_cutoff_frequency", lp_cutoff_frequency, 50.);
//  for (int i = 0; i < 6; ++i) {
//    actionLPFs_.emplace_back(lp_cutoff_frequency);
//  }
  actions_.resize(6);
  lastAction_.resize(6);
//  double inertialAlpha;
//  controller_nh.param("inertial_alpha", inertialAlpha, 0.5);
//  for (int i = 0; i < 6; ++i) {
//    actionIFs_.emplace_back(inertialAlpha);
//  }
//  int windowSize;
//  controller_nh.param("window_size", windowSize, 2);
//  for (int i = 0; i < 6; ++i) {
//    actionMFs_.emplace_back(windowSize);
//    actionMFs_[i].reset();
//  }
  geometry_msgs::Twist initTwist;
  initTwist.linear.x = 0.0;
  initTwist.linear.y = 0.0;
  initTwist.linear.z = 0.0;
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
  if (useVMC_ || addGravityFF_)
  {
      // change for diablo urdf
      leftSerialVMCPtr_->update(
          M_PI + jointHandles_[0].getPosition() - hipBias_ ,jointHandles_[0].getVelocity(), jointHandles_[0].getEffort(),
          jointHandles_[1].getPosition() - M_PI - kneeBias_, jointHandles_[1].getVelocity(), jointHandles_[1].getEffort());
      rightSerialVMCPtr_->update(
          M_PI + jointHandles_[3].getPosition() - hipBias_,jointHandles_[3].getVelocity(), jointHandles_[3].getEffort(),
          jointHandles_[4].getPosition() - M_PI - kneeBias_, jointHandles_[4].getVelocity(), jointHandles_[4].getEffort());
  }
  rl(time,period);
//  prostrate(time,period);
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

//  ROS_INFO_STREAM("prostrate  Mode");
}

void WheeledBipedalRLController::rl(const ros::Time& time, const ros::Duration& period)
{
//TODO ：add set control.x, control.y, control.yaw.
  setCommand(time, period);
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
  robotStateMsg_.stamp = ros::Time::now();
  robotStateMsg_.imu_states.header.frame_id = imuSensorHandle_.getName();
  robotStateMsg_.imu_states.orientation.x = imuSensorHandle_.getOrientation()[0];
  robotStateMsg_.imu_states.orientation.y = imuSensorHandle_.getOrientation()[1];
  robotStateMsg_.imu_states.orientation.z = imuSensorHandle_.getOrientation()[2];
  robotStateMsg_.imu_states.orientation.w = imuSensorHandle_.getOrientation()[3];

//  For test imu
  geometry_msgs::Quaternion base;
  double roll, yaw;
  base.x = imuSensorHandle_.getOrientation()[0];
  base.y = imuSensorHandle_.getOrientation()[1];
  base.z = imuSensorHandle_.getOrientation()[2];
  base.w = imuSensorHandle_.getOrientation()[3];
  robot_common::quatToRPY(base,roll,basePitch_,yaw);
//  ROS_INFO_STREAM("Roll==" << roll);
//  ROS_INFO_STREAM("Pitch==" << basePitch_);
//  ROS_INFO_STREAM("Yaw==" << yaw);
  robotStateMsg_.rpy[0] = roll;
  robotStateMsg_.rpy[1] = basePitch_;
  robotStateMsg_.rpy[2] = yaw;

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

  if (useVMC_)
  {
    robotStateMsg_.left.l = leftSerialVMCPtr_->r_;
    robotStateMsg_.left.l_dot = leftSerialVMCPtr_->dr_;
    robotStateMsg_.left.theta = leftSerialVMCPtr_->theta_;
    robotStateMsg_.left.theta_dot = leftSerialVMCPtr_->dtheta_;
    robotStateMsg_.right.l = rightSerialVMCPtr_->r_;
    robotStateMsg_.right.l_dot = rightSerialVMCPtr_->dr_;
    robotStateMsg_.right.theta = rightSerialVMCPtr_->theta_;
    robotStateMsg_.right.theta_dot = rightSerialVMCPtr_->dtheta_;
  }
  for (int i = 0; i < 6; ++i)
  {
    robotStateMsg_.actions[i] = actions_[i];
  }
  robotStatePub_.publish(robotStateMsg_);
  if (simulation_)
    simRobotStatePub_.publish(robotStateMsg_);
}

void WheeledBipedalRLController::setCommand(const ros::Time& time, const ros::Duration& period)
{
  auto rt_buffer = rlCmdRtBuffer_.readFromRT();
  auto& data = rt_buffer->data;
  if (data.empty())
  {
    jointHandles_[0].setCommand(Pids_[0].computeCommand(0-jointHandles_[0].getPosition(),period));
    jointHandles_[1].setCommand(Pids_[1].computeCommand(0-jointHandles_[1].getPosition(),period));
    jointHandles_[3].setCommand(Pids_[3].computeCommand(0-jointHandles_[3].getPosition(),period));
    jointHandles_[4].setCommand(Pids_[4].computeCommand(0-jointHandles_[4].getPosition(),period));
//    jointHandles_[0].setCommand(0);
//    jointHandles_[1].setCommand(0);
//    jointHandles_[3].setCommand(0);
//    jointHandles_[4].setCommand(0);

    jointHandles_[2].setCommand(0);
    jointHandles_[5].setCommand(0);
  }
  else
  {
      for (int i = 0; i < static_cast<int>(data.size()); ++i)
      {
//        actionMFs_[i].input(data[i]);
//        actions_[i] = actionMFs_[i].output();
          actions_[i] = data[i];
      }
    if (useVMC_)
    {
      //  data: [left_theta, left_l, left_wheel_vel, right_theta, right_l, right_wheel_vel]
      double leftFR = VMCPids_[0].computeCommand(actions_[1] - leftSerialVMCPtr_->r_,period);
      double leftFTheta = VMCPids_[1].computeCommand(actions_[0] - leftSerialVMCPtr_->theta_,period);
      double rightFR = VMCPids_[2].computeCommand(actions_[4] - rightSerialVMCPtr_->r_,period);
      double rightFTheta = VMCPids_[3].computeCommand(actions_[3] - rightSerialVMCPtr_->theta_,period);

      std::vector<double> leftJointCmd = leftSerialVMCPtr_->getDesJointEff(leftSerialVMCPtr_->phi1_,leftSerialVMCPtr_->phi2_,leftFR + gravityFeedforward_* cos(leftSerialVMCPtr_->theta_), leftFTheta - gravityFeedforward_* sin(leftSerialVMCPtr_->theta_));
      std::vector<double> rightJointCmd = rightSerialVMCPtr_->getDesJointEff(rightSerialVMCPtr_->phi1_,rightSerialVMCPtr_->phi2_,rightFR + gravityFeedforward_* cos(rightSerialVMCPtr_->theta_), rightFTheta - gravityFeedforward_* sin(rightSerialVMCPtr_->theta_));

      jointHandles_[0].setCommand(leftJointCmd[0]);
      jointHandles_[1].setCommand(leftJointCmd[1]);
      jointHandles_[3].setCommand(rightJointCmd[0]);
      jointHandles_[4].setCommand(rightJointCmd[1]);

      jointHandles_[2].setCommand(Pids_[2].computeCommand(actions_[2]-jointHandles_[2].getVelocity(),period));
      jointHandles_[5].setCommand(Pids_[5].computeCommand(actions_[5]-jointHandles_[5].getVelocity(),period));
    }
    else
    {
      std::vector<double> leftJointCmd = {0., 0.};
      std::vector<double> rightJointCmd = {0., 0.};
      if (addGravityFF_)
      {
        leftJointCmd = leftSerialVMCPtr_->getDesJointEff(leftSerialVMCPtr_->phi1_,leftSerialVMCPtr_->phi2_, gravityFeedforward_, 0.);
        rightJointCmd = rightSerialVMCPtr_->getDesJointEff(rightSerialVMCPtr_->phi1_,rightSerialVMCPtr_->phi2_,gravityFeedforward_, 0.);
      }
      double pitchReturn = basePitch_;
      double hipReturnTorque = pitchReturn * 0 ;
      double kneeReturnTorque = abs(pitchReturn) * 0 ;
      double wheelReturnTorque = pitchReturn * 0 ;
      jointHandles_[0].setCommand(Pids_[0].computeCommand(actions_[0]-jointHandles_[0].getPosition(),period) + leftJointCmd[0] + hipReturnTorque);
      jointHandles_[1].setCommand(Pids_[1].computeCommand(actions_[1]-jointHandles_[1].getPosition(),period) + leftJointCmd[1] + kneeReturnTorque);
      jointHandles_[3].setCommand(Pids_[3].computeCommand(actions_[3]-jointHandles_[3].getPosition(),period) + rightJointCmd[0] + hipReturnTorque);
      jointHandles_[4].setCommand(Pids_[4].computeCommand(actions_[4]-jointHandles_[4].getPosition(),period) + rightJointCmd[1] + kneeReturnTorque);

      jointHandles_[2].setCommand(Pids_[2].computeCommand(actions_[2]-jointHandles_[2].getVelocity(),period) + wheelReturnTorque);
      jointHandles_[5].setCommand(Pids_[5].computeCommand(actions_[5]-jointHandles_[5].getVelocity(),period) + wheelReturnTorque);
    }
  }
  lastAction_ = data;
}

void WheeledBipedalRLController::initStateMsg()
{
  robotStateMsg_.stamp = ros::Time::now();
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
  if (useVMC_)
  {
    robotStateMsg_.left.l = 0.05;
    robotStateMsg_.left.l_dot = 0.0;
    robotStateMsg_.left.theta = 0.0;
    robotStateMsg_.left.theta_dot = 0.0;
    robotStateMsg_.right.l = 0.05;
    robotStateMsg_.right.l_dot = 0.0;
    robotStateMsg_.right.theta = 0.0;
    robotStateMsg_.right.theta_dot = 0.0;
  }

  robotStateMsg_.rpy.resize(3);
  robotStateMsg_.rpy[0] = 0;
  robotStateMsg_.rpy[1] = 0;
  robotStateMsg_.rpy[2] = 0;
  robotStateMsg_.actions.resize(6);
  for (double & action : robotStateMsg_.actions)
  {
      action = 0.;
  }
}

}  // namespace rl_controller

PLUGINLIB_EXPORT_CLASS(rl_controller::WheeledBipedalRLController, controller_interface::ControllerBase)