#include "robot_gazebo/RobotHWSim.h"
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>

namespace Robot
{
bool RobotHWSim::initSim(const std::string& robot_namespace, ros::NodeHandle model_nh,
                         gazebo::physics::ModelPtr parent_model, const urdf::Model* urdf_model,
                         std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  bool ret = DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);
  // HybridJoint interface
  registerInterface(&hybridJointInterface_);
  if (!model_nh.getParam("gazebo/hybrid_joints", hybridJoints_))
  {
    ROS_INFO_STREAM("Parameter 'gazebo/hybrid_joints' not found, using default values.");
    for (const auto& name : hybridJoints_)
    {
      hybridJointDatas_.push_back(HybridJointData{ .joint_ = ej_interface_.getHandle(name) });
      HybridJointData& back = hybridJointDatas_.back();
      hybridJointInterface_.registerHandle(hardware_interface::HybridJointHandle(
          back.joint_, &back.posDes_, &back.velDes_, &back.kp_, &back.kd_, &back.ff_));
      cmdBuffer_.insert(std::make_pair(name.c_str(), std::deque<HybridJointCommand>()));
    }
  }

  registerInterface(&robotStateInterface_);
  registerInterface(&imuSensorInterface_);
  XmlRpc::XmlRpcValue xmlRpcValue;
  if (!model_nh.getParam("gazebo/imus", xmlRpcValue))
  {
    ROS_WARN("No imu specified");
  }
  else
  {
    parseImu(xmlRpcValue, parent_model);
  }
  if (!model_nh.getParam("gazebo/delay", delay_))
  {
    delay_ = 0.;
  }

  // Initialize joint positions and read from ROS parameters
  XmlRpc::XmlRpcValue joint_init_positions;
  if (model_nh.getParam("gazebo/joint_init_positions", joint_init_positions))
  {
    for (auto& joint_init_position : joint_init_positions)
    {
      std::string joint_name = joint_init_position.first;
      auto joint_position = static_cast<double>(joint_init_position.second);

      // Find the joint name in joint_names_ and set its initial position
      auto it_joint = std::find(joint_names_.begin(), joint_names_.end(), joint_name);
      if (it_joint != joint_names_.end())
      {
        size_t index = std::distance(joint_names_.begin(), it_joint);

        // Update joint_position_ for the found index
        joint_position_[index] = joint_position;

        // Set the joint position in Gazebo
        sim_joints_[index]->SetPosition(0, joint_position, true);
        ROS_INFO_STREAM("Set initial position of " << joint_name << " to " << joint_position);
      }
      else
      {
        ROS_WARN_STREAM("Joint " << joint_name << " not found in joint_names_");
      }
    }
  }
  else
  {
    ROS_WARN("No joint initial positions specified");
  }

  double lp_cutoff_frequency;
  model_nh.param("lp_cutoff_frequency", lp_cutoff_frequency, 100.);
  model_nh.param("use_filter", useFilter_, true);
  for (unsigned int i = 0; i < n_dof_; ++i) {
    velLPFs_.emplace_back(lp_cutoff_frequency);
  }


  return ret;
}

void RobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  //  gazebo_ros_control::DefaultRobotHWSim::readSim(time, period); The
  //  DefaultRobotHWSim Provide a bias joint velocity
  for (unsigned int j = 0; j < n_dof_; j++)
  {
    double position = sim_joints_[j]->Position(0);
//    joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);

    if (time == ros::Time(period.toSec()))
    {
      joint_velocity_[j] = 0;
    }
    else
    {
      if (useFilter_)
      {
        velLPFs_[j].input((position - joint_position_[j]) / period.toSec(), time);
        joint_velocity_[j] = velLPFs_[j].output();
      }
      else
        joint_velocity_[j] = (position - joint_position_[j]) / period.toSec();
    }

    if (joint_types_[j] == urdf::Joint::PRISMATIC)
    {
      joint_position_[j] = position;
    }
    else
    {
      joint_position_[j] += angles::shortest_angular_distance(joint_position_[j], position);
    }
    joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
  }

  for (auto& imu : imuDatas_)
  {
    // TODO(qiayuan) Add noise
    ignition::math::Pose3d pose = imu.linkPtr_->WorldPose();
    imu.ori_[0] = pose.Rot().X();
    imu.ori_[1] = pose.Rot().Y();
    imu.ori_[2] = pose.Rot().Z();
    imu.ori_[3] = pose.Rot().W();
    ignition::math::Vector3d rate = imu.linkPtr_->RelativeAngularVel();
    imu.angularVel_[0] = rate.X();
    imu.angularVel_[1] = rate.Y();
    imu.angularVel_[2] = rate.Z();

    ignition::math::Vector3d gravity = { 0., 0., -9.81 };
    ignition::math::Vector3d accel = imu.linkPtr_->RelativeLinearAccel() - pose.Rot().RotateVectorReverse(gravity);
    imu.linearAcc_[0] = accel.X();
    imu.linearAcc_[1] = accel.Y();
    imu.linearAcc_[2] = accel.Z();
  }

  // Set cmd to zero to avoid crazy soft limit oscillation when not controller
  // loaded
  for (auto& cmd : joint_effort_command_)
  {
    cmd = 0;
  }
  for (auto& cmd : joint_velocity_command_)
  {
    cmd = 0;
  }
  if (!hybridJoints_.empty())
  {
    for (auto& joint : hybridJointDatas_)
    {
      joint.posDes_ = joint.joint_.getPosition();
      joint.velDes_ = joint.joint_.getVelocity();
      joint.kp_ = 0.;
      joint.kd_ = 0.;
      joint.ff_ = 0.;
    }
  }
}

void RobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
  if (!hybridJoints_.empty())
  {
    for (auto joint : hybridJointDatas_)
    {
      auto& buffer = cmdBuffer_.find(joint.joint_.getName())->second;
      if (time == ros::Time(period.toSec()))
      {  // Simulation reset
        buffer.clear();
      }

      while (!buffer.empty() && buffer.back().stamp_ + ros::Duration(delay_) < time)
      {
        buffer.pop_back();
      }
      buffer.push_front(HybridJointCommand{ .stamp_ = time,
                                            .posDes_ = joint.posDes_,
                                            .velDes_ = joint.velDes_,
                                            .kp_ = joint.kp_,
                                            .kd_ = joint.kd_,
                                            .ff_ = joint.ff_ });

      const auto& cmd = buffer.back();
      joint.joint_.setCommand(cmd.kp_ * (cmd.posDes_ - joint.joint_.getPosition()) +
                              cmd.kd_ * (cmd.velDes_ - joint.joint_.getVelocity()) + cmd.ff_);
    }
  }
  DefaultRobotHWSim::writeSim(time, period);
}

void RobotHWSim::parseImu(XmlRpc::XmlRpcValue& imuDatas, const gazebo::physics::ModelPtr& parentModel)
{
  ROS_ASSERT(imuDatas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (auto it = imuDatas.begin(); it != imuDatas.end(); ++it)
  {
    if (!it->second.hasMember("frame_id"))
    {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated frame id.");
      continue;
    }
    else if (!it->second.hasMember("orientation_covariance_diagonal"))
    {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated orientation covariance diagonal.");
      continue;
    }
    else if (!it->second.hasMember("angular_velocity_covariance"))
    {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated angular velocity covariance.");
      continue;
    }
    else if (!it->second.hasMember("linear_acceleration_covariance"))
    {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated linear acceleration covariance.");
      continue;
    }
    XmlRpc::XmlRpcValue oriCov = imuDatas[it->first]["orientation_covariance_diagonal"];
    ROS_ASSERT(oriCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(oriCov.size() == 3);
    for (int i = 0; i < oriCov.size(); ++i)
    {
      ROS_ASSERT(oriCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }
    XmlRpc::XmlRpcValue angularCov = imuDatas[it->first]["angular_velocity_covariance"];
    ROS_ASSERT(angularCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(angularCov.size() == 3);
    for (int i = 0; i < angularCov.size(); ++i)
    {
      ROS_ASSERT(angularCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }
    XmlRpc::XmlRpcValue linearCov = imuDatas[it->first]["linear_acceleration_covariance"];
    ROS_ASSERT(linearCov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(linearCov.size() == 3);
    for (int i = 0; i < linearCov.size(); ++i)
    {
      ROS_ASSERT(linearCov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    }

    std::string frameId = imuDatas[it->first]["frame_id"];
    gazebo::physics::LinkPtr linkPtr = parentModel->GetLink(frameId);
    ROS_ASSERT(linkPtr != nullptr);
    imuDatas_.push_back((
        ImuData{ .linkPtr_ = linkPtr,
                 .ori_ = { 0., 0., 0., 0. },
                 .oriCov_ = { static_cast<double>(oriCov[0]), 0., 0., 0., static_cast<double>(oriCov[1]), 0., 0., 0.,
                              static_cast<double>(oriCov[2]) },
                 .angularVel_ = { 0., 0., 0. },
                 .angularVelCov_ = { static_cast<double>(angularCov[0]), 0., 0., 0., static_cast<double>(angularCov[1]),
                                     0., 0., 0., static_cast<double>(angularCov[2]) },
                 .linearAcc_ = { 0., 0., 0. },
                 .linearAccCov_ = { static_cast<double>(linearCov[0]), 0., 0., 0., static_cast<double>(linearCov[1]),
                                    0., 0., 0., static_cast<double>(linearCov[2]) } }));
    ImuData& imuData = imuDatas_.back();
    imuSensorInterface_.registerHandle(
        hardware_interface::ImuSensorHandle(it->first, frameId, imuData.ori_, imuData.oriCov_, imuData.angularVel_,
                                            imuData.angularVelCov_, imuData.linearAcc_, imuData.linearAccCov_));
  }
}

}  // namespace Robot

PLUGINLIB_EXPORT_CLASS(Robot::RobotHWSim, gazebo_ros_control::RobotHWSim)
GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::GazeboRosControlPlugin)  // Default plugin
