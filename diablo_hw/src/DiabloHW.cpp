//
// Created by lsy on 24-9-23.
//
// ref: https://github.com/qiayuanl/legged_control

#include "diablo_hw/DiabloHW.h"

namespace diablo {
bool DiabloHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
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

  double lp_cutoff_frequency;
  robot_hw_nh.param("lp_cutoff_frequency", lp_cutoff_frequency, 50.);
  robot_hw_nh.param("use_filter", useFilter_, true);
  for (int i = 0; i < 6; ++i) {
    velLPFs_.emplace_back(lp_cutoff_frequency);
    posLPFs_.emplace_back(lp_cutoff_frequency);
    tauLPFs_.emplace_back(lp_cutoff_frequency);
  }

  double max_acc;
  robot_hw_nh.param("max_acc", max_acc, 50.);
  for (int i = 0; i < 6; ++i) {
    velVFs_.emplace_back(max_acc);
    posVFs_.emplace_back(max_acc);
    tauVFs_.emplace_back(max_acc);
  }

  diabloSDK_->start_joint_sdk();
  ros::Duration(0.5).sleep();
  return true;
}

void DiabloHW::updatePos(float new_position, int seq) {
  new_position += jointOffset_[seq];
  new_position *= jointDirection_[seq];
  float delta_position = new_position - jointData_[seq].pos_;

  if (delta_position < -POS_THRESHOLD) {
    revolutionCount[seq] += 1;
  }
  else if (delta_position > POS_THRESHOLD) {
    revolutionCount[seq] -= 1;
  }
  jointData_[seq].pos_ = revolutionCount[seq] * 2 * M_PI + new_position;
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

void DiabloHW::read(const ros::Time& time, const ros::Duration& period) {
//  std::lock_guard<std::mutex> lock(diabloSDK_->buffer_mutex);
  auto diabloInfo = diabloSDK_->rec_package;
  auto Joints = std::vector<motor_msgs_t>{diabloInfo->left_hip, diabloInfo->left_knee, diabloInfo->left_wheel,diabloInfo->right_hip, diabloInfo->right_knee, diabloInfo->right_wheel};

  if (!init_)
  {
    jointOffset_[2] = -Joints[2].pos / POS_SCALE * jointDirection_[2];
    jointOffset_[5] = -Joints[5].pos / POS_SCALE * jointDirection_[5];
    init_ = true;
  }
  int i = 0;
  for (const auto& joint : Joints) {
    if (useFilter_)
    {
      posLPFs_[i].input(joint.pos / POS_SCALE);
      velLPFs_[i].input(joint.vel / VEL_SCALE);
      tauLPFs_[i].input( joint.torque / TAU_SCALE);

      jointData_[i].pos_ = posLPFs_[i].output();
      jointData_[i].vel_ = velLPFs_[i].output();
      jointData_[i].tau_ = tauLPFs_[i].output();
    }
    else
    {
      updatePos(joint.pos / POS_SCALE, i);
      jointData_[i].vel_ = joint.vel / VEL_SCALE;
      jointData_[i].tau_ = joint.torque / TAU_SCALE;
    }
    jointData_[i].vel_ *= jointDirection_[i];
    jointData_[i].tau_ *= jointDirection_[i];
    ++i;
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