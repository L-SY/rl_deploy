#include "robot_common/utilities/can_devices.h"

namespace can_interface
{
bool CanDevices::initMotor(XmlRpc::XmlRpcValue& act_coeffs, XmlRpc::XmlRpcValue& act_datas, ros::NodeHandle& robot_hw_nh){
  bool success = true;
  success &= parseActCoeffs(act_coeffs);
  success &= parseActData(act_datas, robot_hw_nh);

  return success;
}

bool CanDevices::initIMU(XmlRpc::XmlRpcValue& imu_datas, ros::NodeHandle& robot_hw_nh)
{
  return parseImuData(imu_datas, robot_hw_nh);
}

bool CanDevices::parseActCoeffs(XmlRpc::XmlRpcValue& act_coeffs)
{
  ROS_ASSERT(act_coeffs.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  try
  {
    for (auto it = act_coeffs.begin(); it != act_coeffs.end(); ++it)
    {
      can_interface::ActCoeff act_coeff{};

      act_coeff.act2pos = xmlRpcGetDouble(act_coeffs[it->first], "act2pos", 0.0001192087);
      act_coeff.act2vel = xmlRpcGetDouble(act_coeffs[it->first], "act2vel", 0.001022904);
      act_coeff.act2effort = xmlRpcGetDouble(act_coeffs[it->first], "act2effort", 0.00341797);

      act_coeff.pos2act = xmlRpcGetDouble(act_coeffs[it->first], "pos2act", 2621.4);
      act_coeff.vel2act = xmlRpcGetDouble(act_coeffs[it->first], "vel2act", 977.61267);
      act_coeff.effort2act = xmlRpcGetDouble(act_coeffs[it->first], "effort2act", 292.571322);

      act_coeff.pos_offset = xmlRpcGetDouble(act_coeffs[it->first], "pos_offset", -3.1415926);
      act_coeff.vel_offset = xmlRpcGetDouble(act_coeffs[it->first], "vel_offset", -20.943951);
      act_coeff.effort_offset = xmlRpcGetDouble(act_coeffs[it->first], "effort_offset", -7.0);
      act_coeff.kp2act = xmlRpcGetDouble(act_coeffs[it->first], "kp2act", 8.19);
      act_coeff.kd2act = xmlRpcGetDouble(act_coeffs[it->first], "kd2act", 819);

      act_coeff.max_out = xmlRpcGetDouble(act_coeffs[it->first], "max_out", 10000);
      std::string type = it->first;

      if (type2act_coeffs_.find(type) == type2act_coeffs_.end())
        type2act_coeffs_.insert(std::make_pair(type, act_coeff));
      else
        ROS_ERROR_STREAM("Repeat actuator coefficient of type: " << type);
    }
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
                     << "configuration: " << e.getMessage() << ".\n"
                     << "Please check the configuration, particularly parameter types.");
    return false;
  }
  return true;
}
bool CanDevices::parseActData(XmlRpc::XmlRpcValue& act_datas, ros::NodeHandle& robot_hw_nh)
{
  ROS_ASSERT(act_datas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  try
  {
    for (auto it = act_datas.begin(); it != act_datas.end(); ++it)
    {
      if (!it->second.hasMember("bus"))
      {
        ROS_ERROR_STREAM("Actuator " << it->first << " has no associated bus.");
        continue;
      }
      else if (!it->second.hasMember("type"))
      {
        ROS_ERROR_STREAM("Actuator " << it->first << " has no associated type.");
        continue;
      }
      else if (!it->second.hasMember("id"))
      {
        ROS_ERROR_STREAM("Actuator " << it->first << " has no associated ID.");
        continue;
      }
      // the act_offset must be 0.x format.
      double act_offset = it->second.hasMember("act_offset") ? ((double)act_datas[it->first]["act_offset"]) : 0.;
      std::string control_mod_str = it->second.hasMember("control_mode") ? ((std::string)act_datas[it->first]["control_mode"]) : "EFFORT";
      ControlMode control_mode = string2mode(control_mod_str);
      std::string bus = act_datas[it->first]["bus"], type = act_datas[it->first]["type"];
      int id = static_cast<int>(act_datas[it->first]["id"]);
      // check define of act_coeffs
      if (type2act_coeffs_.find(type) == type2act_coeffs_.end())
      {
        ROS_ERROR_STREAM("Type " << type << " has no associated coefficient.");
        return false;
      }
      // for bus interface
      if (bus_id2act_data_.find(bus) == bus_id2act_data_.end())
        bus_id2act_data_.insert(std::make_pair(bus, std::unordered_map<int, can_interface::ActData>()));

      if (!(bus_id2act_data_[bus].find(id) == bus_id2act_data_[bus].end()))
      {
        ROS_ERROR_STREAM("Repeat actuator on bus " << bus << " and ID " << id);
        return false;
      }
      else
      {
        ros::NodeHandle nh = ros::NodeHandle(robot_hw_nh, "actuators/" + it->first);
        bus_id2act_data_[bus].insert(std::make_pair(id, can_interface::ActData{ .name = it->first,
                                                                                .type = type,
                                                                                .stamp = ros::Time::now(),
                                                                                .seq = 0,
                                                                                .mode = control_mode,
                                                                                .halted = false,
                                                                                .q_raw = 0,
                                                                                .qd_raw = 0,
                                                                                .temp = 0,
                                                                                .q_circle = 0,
                                                                                .q_last = 0,
                                                                                .frequency = 0,
                                                                                .pos = 0,
                                                                                .vel = 0,
                                                                                .effort = 0,
                                                                                .cmd_pos = 0,
                                                                                .cmd_vel = 0,
                                                                                .cmd_effort = 0,
                                                                                .exe_effort = 0,
                                                                                .act_offset = act_offset,
                                                                                .lp_filter = new LowPassFilter(nh) }));
      }
    }
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
                     << "configuration: " << e.getMessage() << ".\n"
                     << "Please check the configuration, particularly parameter types.");
    return false;
  }
  return true;
}

bool CanDevices::initCanBus(ros::NodeHandle& robot_hw_nh)
{
  // CAN Bus
  XmlRpc::XmlRpcValue xml_rpc_value;
  int thread_priority_ = 95;
  if (!robot_hw_nh.getParam("bus", xml_rpc_value))
    ROS_WARN("No bus specified");
  else if (xml_rpc_value.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ASSERT(xml_rpc_value[0].getType() == XmlRpc::XmlRpcValue::TypeString);
    for (int i = 0; i < xml_rpc_value.size(); ++i)
    {
      std::string bus_name = xml_rpc_value[i];
      if (bus_name.find("can") != std::string::npos)
        can_buses_.push_back(
            new can_interface::CanBus(bus_name,
                                      can_interface::CanDataPtr{ .type2act_coeffs_ = &type2act_coeffs_,
                                                                 .id2act_data_ = &bus_id2act_data_[bus_name],
                                                                 .id2imu_data_ = &bus_id2imu_data_[bus_name]},
                                      thread_priority_));
      else
        ROS_ERROR_STREAM("Unknown bus: " << bus_name);
    }
  }
  return true;
}

bool CanDevices::parseImuData(XmlRpc::XmlRpcValue& imu_datas, ros::NodeHandle& robot_hw_nh)
{
  ROS_ASSERT(imu_datas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  try
  {
    for (auto it = imu_datas.begin(); it != imu_datas.end(); ++it)
    {
      std::string name = it->first;
      if (!it->second.hasMember("frame_id"))
      {
        ROS_ERROR_STREAM("Imu " << name << " has no associated frame id.");
        continue;
      }
      else if (!it->second.hasMember("bus"))
      {
        ROS_ERROR_STREAM("Imu " << name << " has no associated bus.");
        continue;
      }
      else if (!it->second.hasMember("id"))
      {
        ROS_ERROR_STREAM("Imu " << name << " has no associated ID.");
        continue;
      }
      else if (!it->second.hasMember("orientation_covariance_diagonal"))
      {
        ROS_ERROR_STREAM("Imu " << name << " has no associated orientation covariance diagonal.");
        continue;
      }
      else if (!it->second.hasMember("angular_velocity_covariance"))
      {
        ROS_ERROR_STREAM("Imu " << name << " has no associated angular velocity covariance.");
        continue;
      }
      else if (!it->second.hasMember("linear_acceleration_covariance"))
      {
        ROS_ERROR_STREAM("Imu " << name << " has no associated linear acceleration covariance.");
        continue;
      }
      else if (!it->second.hasMember("angular_vel_offset"))
      {
        ROS_ERROR_STREAM("Imu " << name << " has no associated angular_vel_offset type");
        continue;
      }
      else if (!it->second.hasMember("angular_vel_coeff"))
      {
        ROS_ERROR_STREAM("Imu " << name << " has no associated angular velocity coefficient.");
        continue;
      }
      else if (!it->second.hasMember("accel_coeff"))
      {
        ROS_ERROR_STREAM("Imu " << name << " has no associated linear acceleration coefficient.");
        continue;
      }
      else if (!it->second.hasMember("temp_coeff"))
      {
        ROS_ERROR_STREAM("Imu " << name << " has no associated temperate coefficient.");
        continue;
      }
      else if (!it->second.hasMember("filter"))
      {
        ROS_ERROR_STREAM("Imu " << name << " has no associated filter type");
        continue;
      }
      else if (!it->second.hasMember("angular_vel_offset"))
      {
        ROS_ERROR_STREAM("Imu " << name << " has no associated angular_vel_offset type");
        continue;
      }
      XmlRpc::XmlRpcValue angular_vel_offsets = imu_datas[name]["angular_vel_offset"];
      ROS_ASSERT(angular_vel_offsets.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(angular_vel_offsets.size() == 3);
      for (int i = 0; i < angular_vel_offsets.size(); ++i)
        ROS_ASSERT(angular_vel_offsets[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      XmlRpc::XmlRpcValue ori_cov = imu_datas[name]["orientation_covariance_diagonal"];
      ROS_ASSERT(ori_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(ori_cov.size() == 3);
      for (int i = 0; i < ori_cov.size(); ++i)
        ROS_ASSERT(ori_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      XmlRpc::XmlRpcValue angular_cov = imu_datas[name]["angular_velocity_covariance"];
      ROS_ASSERT(angular_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(angular_cov.size() == 3);
      for (int i = 0; i < angular_cov.size(); ++i)
        ROS_ASSERT(angular_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      XmlRpc::XmlRpcValue linear_cov = imu_datas[name]["linear_acceleration_covariance"];
      ROS_ASSERT(linear_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(linear_cov.size() == 3);
      for (int i = 0; i < linear_cov.size(); ++i)
        ROS_ASSERT(linear_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      std::string filter_type = imu_datas[name]["filter"];
      // TODO(Zhenyu Ye): Add more types of filter.
      robot_common::ImuFilterBase* imu_filter;
      if (filter_type.find("complementary") != std::string::npos)
        imu_filter = new robot_common::ImuComplementaryFilter;
      else
      {
        ROS_ERROR_STREAM("Imu " << name << " doesn't has filter type " << filter_type);
        return false;
      }
      imu_filter->init(it->second, name);

      std::string frame_id = imu_datas[name]["frame_id"], bus = imu_datas[name]["bus"];
      int id = static_cast<int>(imu_datas[name]["id"]);

      // for bus interface
      if (bus_id2imu_data_.find(bus) == bus_id2imu_data_.end())
        bus_id2imu_data_.insert(std::make_pair(bus, std::unordered_map<int, ImuData>()));

      if (!(bus_id2imu_data_[bus].find(id) == bus_id2imu_data_[bus].end()))
      {
        ROS_ERROR_STREAM("Repeat Imu on bus " << bus << " and ID " << id);
        return false;
      }
      else
        bus_id2imu_data_[bus].insert(std::make_pair(
            id, ImuData{ .time_stamp = {},
                         .imu_name = name,
                         .ori = {},
                         .angular_vel = {},
                         .linear_acc = {},
                         .angular_vel_offset = { static_cast<double>(angular_vel_offsets[0]),
                                                 static_cast<double>(angular_vel_offsets[1]),
                                                 static_cast<double>(angular_vel_offsets[2]) },
                         .ori_cov = { static_cast<double>(ori_cov[0]), 0., 0., 0., static_cast<double>(ori_cov[1]), 0.,
                                      0., 0., static_cast<double>(ori_cov[2]) },
                         .angular_vel_cov = { static_cast<double>(angular_cov[0]), 0., 0., 0.,
                                              static_cast<double>(angular_cov[1]), 0., 0., 0.,
                                              static_cast<double>(angular_cov[2]) },
                         .linear_acc_cov = { static_cast<double>(linear_cov[0]), 0., 0., 0.,
                                             static_cast<double>(linear_cov[1]), 0., 0., 0.,
                                             static_cast<double>(linear_cov[2]) },
                         .temperature = 0.0,
                         .angular_vel_coeff = xmlRpcGetDouble(imu_datas[name], "angular_vel_coeff", 0.),
                         .accel_coeff = xmlRpcGetDouble(imu_datas[name], "accel_coeff", 0.),
                         .temp_coeff = xmlRpcGetDouble(imu_datas[name], "temp_coeff", 0.),
                         .temp_offset = xmlRpcGetDouble(imu_datas[name], "temp_offset", 0.),
                         .accel_updated = false,
                         .gyro_updated = false,
                         .camera_trigger = false,
                         .enabled_trigger = false,
                         .imu_filter = imu_filter }));
    }
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    ROS_FATAL_STREAM("Exception raised by XmlRpc while reading the "
                     << "configuration: " << e.getMessage() << ".\n"
                     << "Please check the configuration, particularly parameter types.");
    return false;
  }
  return true;
}

void CanDevices::startMotor()
{
  for (auto can_bus : can_buses_)
  {
    can_bus->start();
  }
}

void CanDevices::closeMotor()
{
  for (auto can_bus : can_buses_)
  {
    can_bus->close();
  }
}

void CanDevices::testMotor()
{
  for (auto can_bus : can_buses_)
  {
    can_bus->test();
  }
}

ControlMode CanDevices::string2mode(std::string mode)
{
  if (mode == "MIT")
    return MIT;
  else if (mode == "POS")
    return POS;
  else if (mode == "VEL")
    return VEL;
  else if (mode == "EFFORT")
    return EFFORT;
  else
    throw std::invalid_argument("Invalid mode string: " + mode);
}
}  // namespace can_interface
