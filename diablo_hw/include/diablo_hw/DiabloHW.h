//
// Created by lsy on 24-9-23.
//

#pragma once

#include <memory>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// ROS control
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <robot_common/interface/hardware_interface/robot_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "diablo_hw/lib/serial_handle.hpp"

namespace diablo {
struct DiabloMotorData {
  double pos_, vel_, tau_;                 // state
  double cmdTau_;  // command
};

struct DiabloImuData {
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class DiabloHW : public hardware_interface::RobotHW {
public:
  DiabloHW() = default;
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  /** \brief Communicate with hardware. Get data, status of robot.
   *
   * Call @ref UNITREE_LEGGED_SDK::UDP::Recv() to get robot's state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref UNITREE_LEGGED_SDK::UDP::Recv(). Publish actuator
   * current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

private:
  /** \brief Load urdf of robot from param server.
   *
   * Load urdf of robot from param server.
   *
   * @param rootNh Root node-handle of a ROS node
   * @return True if successful.
   */
  bool loadUrdf(ros::NodeHandle& rootNh);

  bool setupJoints();

  bool setupImu();

  // Interface
  hardware_interface::JointStateInterface jointStateInterface_;  // NOLINT(misc-non-private-member-variables-in-classes)
  hardware_interface::ImuSensorInterface imuSensorInterface_;    // NOLINT(misc-non-private-member-variables-in-classes)
  hardware_interface::EffortJointInterface effortJointInterface_;
  hardware_interface::RobotStateInterface robotStateInterface_;

  // URDF model of the robot
  std::shared_ptr<urdf::Model> urdfModel_;  // NOLINT(misc-non-private-member-variables-in-classes)

  // Diablo SDK
  std::shared_ptr<SerialHandle> diabloSDK_;
  motor_torque_t sendStruct_;

  DiabloImuData imuData_{};
  DiabloMotorData jointData_[8]{};
  std::vector<double> leftJointOffset_ = { -1.12 + 2 * M_PI, -2.8 + 2 * M_PI, 0.};
  std::vector<double> rightJointOffset_ = { -1.12, -2.8, 0.};
  std::vector<std::string> jointName = {"left_fake_hip_joint", "left_hip_joint", "left_wheel_joint","right_fake_hip_joint", "right_hip_joint", "right_wheel_joint"};
};

}// namespace diablo
