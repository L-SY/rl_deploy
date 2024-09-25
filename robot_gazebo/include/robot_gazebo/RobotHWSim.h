#pragma once

#include <deque>
#include <unordered_map>

#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <robot_common/interface/hardware_interface/robot_state_interface.h>
#include <robot_common/interface/hardware_interface/HybridJointInterface.h>
#include <hardware_interface/joint_command_interface.h>

namespace Robot
{
struct HybridJointData
{
  hardware_interface::JointHandle joint_;
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};
};

struct HybridJointCommand
{
  ros::Time stamp_;
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};
};

struct ImuData
{
  gazebo::physics::LinkPtr linkPtr_;
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class RobotHWSim : public gazebo_ros_control::DefaultRobotHWSim
{
public:
  bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
               const urdf::Model* urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions) override;
  void readSim(ros::Time time, ros::Duration period) override;
  void writeSim(ros::Time time, ros::Duration period) override;

private:
  void parseImu(XmlRpc::XmlRpcValue& imuDatas, const gazebo::physics::ModelPtr& parentModel);

  hardware_interface::RobotStateInterface robotStateInterface_;
  hardware_interface::ImuSensorInterface imuSensorInterface_;
  std::list<ImuData> imuDatas_;

  std::vector<std::string> hybridJoints_;
  hardware_interface::HybridJointInterface hybridJointInterface_;
  std::list<HybridJointData> hybridJointDatas_;
  std::unordered_map<std::string, std::deque<HybridJointCommand>> cmdBuffer_;

  double delay_{};
};

}  // namespace Robot
