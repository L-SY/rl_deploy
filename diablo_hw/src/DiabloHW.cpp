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
  diabloSDK_ = std::make_shared<SerialHandle>();
  diabloSDK_->serial_init("/dev/ttyUSB0");

  registerInterface(&jointStateInterface_);
  registerInterface(&effortJointInterface_);
  registerInterface(&imuSensorInterface_);
  registerInterface(&robotStateInterface_);

  setupJoints();
  setupImu();
  diabloSDK_->start_joint_sdk();
  ros::Duration(0.5).sleep();
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

void DiabloHW::read(const ros::Time& time, const ros::Duration& /*period*/) {
  std::lock_guard<std::mutex> lock(diabloSDK_->buffer_mutex);
  auto diabloInfo = diabloSDK_->rec_package;
  auto leftJoints = {diabloInfo->left_hip, diabloInfo->left_knee, diabloInfo->left_wheel};
  auto rightJoints = {diabloInfo->right_hip, diabloInfo->right_knee, diabloInfo->right_wheel};
  int i = 0;
  for (const auto& joint : leftJoints) {
    jointData_[i].pos_ = joint.pos / 5215.03f;
    jointData_[i].pos_ += leftJointOffset_[i];
    jointData_[i].vel_ = joint.vel / 655.34f;
    jointData_[i].tau_ = joint.torque / 655.34f;
    jointData_[i].pos_ *= jointDirection_[i];
    jointData_[i].vel_ *= jointDirection_[i];
    jointData_[i].tau_ *= jointDirection_[i];
    ++i;
  }
  int j = 0;
  for (const auto& joint : rightJoints) {
    jointData_[i+j].pos_ = joint.pos / 5215.03f;
    jointData_[i+j].pos_ += rightJointOffset_[j];
    jointData_[i+j].vel_ = joint.vel / 655.34f;
    jointData_[i+j].tau_ = joint.torque / 655.34f;
    jointData_[i+j].pos_ *= jointDirection_[j];
    jointData_[i+j].vel_ *= jointDirection_[j];
    jointData_[i+j].tau_ *= jointDirection_[j];
    ++j;
  }

  imuData_.ori_[0] = diabloInfo->orientation.x / 32767.f;
  imuData_.ori_[1] = diabloInfo->orientation.y / 32767.f;
  imuData_.ori_[2] = diabloInfo->orientation.z / 32767.f;
  imuData_.ori_[3] = diabloInfo->orientation.w / 32767.f;
  imuData_.angularVel_[0] = diabloInfo->gyro.x / 327.67f;
  imuData_.angularVel_[1] = diabloInfo->gyro.y / 327.67f;
  imuData_.angularVel_[2] = diabloInfo->gyro.z / 327.67f;
  imuData_.linearAcc_[0] = diabloInfo->accl.x / 1638.5f;
  imuData_.linearAcc_[1] = diabloInfo->accl.y / 1638.5f;
  imuData_.linearAcc_[2] = diabloInfo->accl.z / 1638.5f;
}

void DiabloHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  float motorCmd[6] = {0., 0., 0., 0., 0., 0.0};
  for (int i = 0; i < 6; ++i) {
    motorCmd[i] = static_cast<float>(jointData_[i].cmdTau_) * jointDirection_[i];
  }
  diabloSDK_->create_package(motorCmd,sendStruct_);
  diabloSDK_->send_commond(sendStruct_);
}

bool DiabloHW::setupJoints() {
  for (int i = 0; i < static_cast<int>(jointName.size()); ++i)
  {
    hardware_interface::JointStateHandle state_handle(jointName[i], &jointData_[i].pos_, &jointData_[i].vel_,
                                                      &jointData_[i].tau_);
    jointStateInterface_.registerHandle(state_handle);
    effortJointInterface_.registerHandle(hardware_interface::JointHandle(state_handle, &jointData_[i].cmdTau_));
  }
  return true;
}

bool DiabloHW::setupImu() {
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("base_imu", "base_imu", imuData_.ori_, imuData_.oriCov_,
                                                                         imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                         imuData_.linearAccCov_));
  imuData_.oriCov_[0] = 0.0012;
  imuData_.oriCov_[4] = 0.0012;
  imuData_.oriCov_[8] = 0.0012;

  imuData_.angularVelCov_[0] = 0.0004;
  imuData_.angularVelCov_[4] = 0.0004;
  imuData_.angularVelCov_[8] = 0.0004;

  return true;
}

}  // namespace diablo