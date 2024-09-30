#include "rl_sdk/rl_sdk.hpp"
#include "iostream"
#include "observation_buffer.hpp"
/* You may need to override this Forward() function
torch::Tensor RL_XXX::Forward()
{
    torch::autograd::GradMode::set_enabled(false);
    torch::Tensor clamped_obs = ComputeObservation();
    torch::Tensor actions = model.forward({clamped_obs}).toTensor();
    torch::Tensor clamped_actions = torch::clamp(actions,
params.clip_actions_lower, params.clip_actions_upper); return
clamped_actions;
}
*/

torch::Tensor rl_sdk::ComputeObservation()
{
  std::vector<torch::Tensor> obs_list;

  for (const std::string& observation : params.observations)
  {
    if (observation == "lin_vel")
    {
      obs_list.push_back(obs.lin_vel * params.lin_vel_scale);
    }
    else if (observation == "ang_vel")
    {
      obs_list.push_back(obs.ang_vel * params.ang_vel_scale);  // TODO is QuatRotateInverse necessery?
      //            obs_list.push_back(QuatRotateInverse(obs.base_quat,
      //            obs.ang_vel, params.framework) * params.ang_vel_scale);
    }
    else if (observation == "gravity_vec")
    {
      obs_list.push_back(QuatRotateInverse(obs.base_quat, obs.gravity_vec, params.framework));
    }
    else if (observation == "commands")
    {
      obs_list.push_back(obs.commands * params.commands_scale);
    }

    if (params.use_vmc)
    {
      if (observation == "vmc")
      {
        //      theta, theta_dot, l, l_dot
        obs.vmc[0][0] *= params.theta_scale;
        obs.vmc[0][1] *= params.theta_scale;
        obs.vmc[0][2] *= params.theta_dot_scale;
        obs.vmc[0][3] *= params.theta_dot_scale;
        obs.vmc[0][4] *= params.l_scale;
        obs.vmc[0][5] *= params.l_scale;
        obs.vmc[0][6] *= params.l_dot_scale;
        obs.vmc[0][7] *= params.l_dot_scale;
        obs_list.push_back(obs.vmc);
      }
      else if (observation == "wheel")
      {
        torch::Tensor wheel_tensors;
        wheel_tensors = torch::zeros({ 1, 4 });
        wheel_tensors[0][0] = obs.dof_pos[0][2] * params.dof_pos_scale;
        wheel_tensors[0][1] = obs.dof_pos[0][5] * params.dof_pos_scale;
        wheel_tensors[0][2] = obs.dof_vel[0][2] * params.dof_vel_scale;
        wheel_tensors[0][3] = obs.dof_vel[0][5] * params.dof_vel_scale;
        obs_list.push_back(wheel_tensors);
      }
    }
    else
    {
      if (observation == "dof_pos")
      {
//      Let wheel pos to zero
        obs.dof_pos[0][2] = 0.;
        obs.dof_pos[0][5] = 0.;
        obs_list.push_back((obs.dof_pos - params.default_dof_pos) * params.dof_pos_scale);
      }
      else if (observation == "dof_vel")
      {
        obs_list.push_back(obs.dof_vel * params.dof_vel_scale);
      }
    }

    if (observation == "actions")
    {
      obs_list.push_back(obs.actions);
    }
  }

  // Debug
  //  std::cout << "obs_list size: " << obs_list.size() << std::endl;
  //  for (size_t i = 0; i < obs_list.size(); ++i) {
  //    std::cout << "Tensor " << i << " shape: " << obs_list[i].sizes()
  //              << std::endl;
  //  }

  torch::Tensor obs = torch::cat(obs_list, 1);
  torch::Tensor clamped_obs = torch::clamp(obs, -params.clip_obs, params.clip_obs);
  return clamped_obs;
}

void rl_sdk::InitObservations()
{
  obs.lin_vel = torch::tensor({ { 0.0, 0.0, 0.0 } });
  obs.ang_vel = torch::tensor({ { 0.0, 0.0, 0.0 } });
  // No need change to -9.81
  obs.gravity_vec = torch::tensor({ { 0.0, 0.0, -1.0 } });
  obs.commands = torch::tensor({ { 0.0, 0.0, 0.0 } });
  obs.base_quat = torch::tensor({ { 0.0, 0.0, 0.0, 1.0 } });
  obs.vmc = torch::zeros({ 1, params.num_of_vmc });
  obs.dof_pos = params.default_dof_pos;
  obs.dof_vel = torch::zeros({ 1, params.num_of_dofs });
  obs.actions = torch::zeros({ 1, params.num_of_dofs });
}

void rl_sdk::InitOutputs()
{
  output_command = torch::zeros({ 1, params.num_of_dofs });
  output_dof_pos = params.default_dof_pos;
}

void rl_sdk::InitControl()
{
  control.vel_x = 0.0;
  control.vel_yaw = 0.0;
  if (params.use_vmc)
    control.pos_z = params.l_offset;
  else
    control.pos_z = 0;
}

torch::Tensor rl_sdk::ComputeCommand(torch::Tensor actions)
{
  if (params.use_vmc)
  {
    // TODO: should change the order to l-theta
    torch::Tensor scaled_action = torch::zeros({ 1, 6 });;

    scaled_action[0][0] = actions[0][0] * params.action_scale_theta;
    scaled_action[0][1] = actions[0][1] * params.action_scale_l;
    scaled_action[0][1] += params.l_offset;
    scaled_action[0][2] = actions[0][2] * params.action_scale_vel;
    scaled_action[0][3] = actions[0][3] * params.action_scale_theta;
    scaled_action[0][4] = actions[0][4] * params.action_scale_l;
    scaled_action[0][4] += params.l_offset;
    scaled_action[0][5] = actions[0][5] * params.action_scale_vel;

    output_command = scaled_action;
  }
  else
  {
    torch::Tensor actions_scaled = actions * params.action_scale_pos;
    double scale_factor = params.action_scale_vel / params.action_scale_pos;
    actions_scaled[0][2] *= scale_factor;
    actions_scaled[0][5] *= scale_factor;
    torch::Tensor command = actions_scaled + params.default_dof_pos;

    output_command = command;
  }
  return output_command;
}

torch::Tensor rl_sdk::QuatRotateInverse(torch::Tensor q, torch::Tensor v, const std::string& framework)
{
  torch::Tensor q_w;
  torch::Tensor q_vec;
  if (framework == "isaacsim")
  {
    q_w = q.index({ torch::indexing::Slice(), 0 });
    q_vec = q.index({ torch::indexing::Slice(), torch::indexing::Slice(1, 4) });
  }
  else if (framework == "isaacgym")
  {
    q_w = q.index({ torch::indexing::Slice(), 3 });
    q_vec = q.index({ torch::indexing::Slice(), torch::indexing::Slice(0, 3) });
  }
  c10::IntArrayRef shape = q.sizes();

  torch::Tensor a = v * (2.0 * torch::pow(q_w, 2) - 1.0).unsqueeze(-1);
  torch::Tensor b = torch::cross(q_vec, v, -1) * q_w.unsqueeze(-1) * 2.0;
  torch::Tensor c = q_vec * torch::bmm(q_vec.view({ shape[0], 1, 3 }), v.view({ shape[0], 3, 1 })).squeeze(-1) * 2.0;
  return a - b + c;
}

void rl_sdk::TorqueProtect(torch::Tensor origin_output_torques)
{
  std::vector<int> out_of_range_indices;
  std::vector<double> out_of_range_values;
  for (int i = 0; i < origin_output_torques.size(1); ++i)
  {
    double torque_value = origin_output_torques[0][i].item<double>();
    double limit_lower = -params.torque_limits[0][i].item<double>();
    double limit_upper = params.torque_limits[0][i].item<double>();

    if (torque_value < limit_lower || torque_value > limit_upper)
    {
      out_of_range_indices.push_back(i);
      out_of_range_values.push_back(torque_value);
    }
  }
  if (!out_of_range_indices.empty())
  {
    for (int i = 0; i < out_of_range_indices.size(); ++i)
    {
      int index = out_of_range_indices[i];
      double value = out_of_range_values[i];
      double limit_lower = -params.torque_limits[0][index].item<double>();
      double limit_upper = params.torque_limits[0][index].item<double>();

      std::cout << LOGGER::WARNING << "Torque(" << index + 1 << ")=" << value << " out of range(" << limit_lower << ", "
                << limit_upper << ")" << std::endl;
    }
    // Just a reminder, no protection
    // control.control_state = STATE_POS_GETDOWN;
    // std::cout << LOGGER::INFO << "Switching to STATE_POS_GETDOWN"<<
    // std::endl;
  }
}

template <typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node& node)
{
  std::vector<T> values;
  for (const auto& val : node)
  {
    values.push_back(val.as<T>());
  }
  return values;
}

template <typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node& node, const std::string& framework, const int& rows, const int& cols)
{
  std::vector<T> values;
  for (const auto& val : node)
  {
    values.push_back(val.as<T>());
  }

  if (framework == "isaacsim")
  {
    std::vector<T> transposed_values(cols * rows);
    for (int r = 0; r < rows; ++r)
    {
      for (int c = 0; c < cols; ++c)
      {
        transposed_values[c * rows + r] = values[r * cols + c];
      }
    }
    return transposed_values;
  }
  else if (framework == "isaacgym")
  {
    return values;
  }
  else
  {
    throw std::invalid_argument("Unsupported framework: " + framework);
  }
}

void rl_sdk::ReadYaml(const std::string config_path)
{
  YAML::Node config;
  try
  {
    config = YAML::LoadFile(config_path);
  }
  catch (YAML::BadFile& e)
  {
    std::cout << LOGGER::ERROR << "The file '" << config_path << "' does not exist" << std::endl;
    return;
  }

  params.model_name = config["model_name"].as<std::string>();
  params.framework = config["framework"].as<std::string>();
  int rows = config["rows"].as<int>();
  int cols = config["cols"].as<int>();
  params.use_history = config["use_history"].as<bool>();
  params.use_vmc = config["use_vmc"].as<bool>();
  params.dt = config["dt"].as<double>();
  params.decimation = config["decimation"].as<int>();
  params.num_observations = config["num_observations"].as<int>();
  params.observations = ReadVectorFromYaml<std::string>(config["observations"]);
  params.clip_obs = config["clip_obs"].as<double>();
  if (config["clip_actions_lower"].IsNull() && config["clip_actions_upper"].IsNull())
  {
    params.clip_actions_upper = torch::tensor({}).view({ 1, -1 });
    params.clip_actions_lower = torch::tensor({}).view({ 1, -1 });
  }
  else
  {
    params.clip_actions_upper =
        torch::tensor(ReadVectorFromYaml<double>(config["clip_actions_upper"], params.framework, rows, cols))
            .view({ 1, -1 });
    params.clip_actions_lower =
        torch::tensor(ReadVectorFromYaml<double>(config["clip_actions_lower"], params.framework, rows, cols))
            .view({ 1, -1 });
  }
  params.action_scale_pos = config["action_scale_pos"].as<double>();
  params.action_scale_vel = config["action_scale_vel"].as<double>();

  params.num_of_dofs = config["num_of_dofs"].as<int>();
  params.lin_vel_scale = config["lin_vel_scale"].as<double>();
  params.ang_vel_scale = config["ang_vel_scale"].as<double>();
  params.dof_pos_scale = config["dof_pos_scale"].as<double>();
  params.dof_vel_scale = config["dof_vel_scale"].as<double>();

  //  For vmc
  if (params.use_vmc)
  {
    params.num_of_vmc = config["num_of_vmc"].as<int>();
    params.l_scale = config["l_scale"].as<double>();
    params.l_dot_scale = config["l_dot_scale"].as<double>();
    params.theta_scale = config["theta_scale"].as<double>();
    params.theta_dot_scale = config["theta_dot_scale"].as<double>();
    params.l_offset = config["l_offset"].as<double>();
    params.action_scale_l = config["action_scale_l"].as<double>();
    params.action_scale_theta = config["action_scale_theta"].as<double>();
    params.action_scale_vel = config["action_scale_vel"].as<double>();
  }

//  TODO： read from yaml
  params.commands_scale = torch::tensor({ params.lin_vel_scale, params.ang_vel_scale, 5.0 });
  params.torque_limits =
      torch::tensor(ReadVectorFromYaml<double>(config["torque_limits"], params.framework, rows, cols)).view({ 1, -1 });
  params.default_dof_pos =
      torch::tensor(ReadVectorFromYaml<double>(config["default_dof_pos"], params.framework, rows, cols)).view({ 1, -1 });
}

torch::Tensor rl_sdk::Forward()
{
  torch::autograd::GradMode::set_enabled(false);
  torch::Tensor clamped_obs = ComputeObservation();

  torch::Tensor actions;
  actions = model.forward({ clamped_obs }).toTensor();

  return actions;
}

void rl_sdk::SetObservation()
{
  obs.ang_vel = torch::tensor(robot_state.imu.gyroscope).unsqueeze(0);
  obs.commands = torch::tensor({ { control.vel_x, control.vel_yaw, control.pos_z } });
  obs.base_quat = torch::tensor(robot_state.imu.quaternion).unsqueeze(0);
  obs.dof_pos = torch::tensor(robot_state.motor_state.q).narrow(0, 0, params.num_of_dofs).unsqueeze(0);
  obs.dof_vel = torch::tensor(robot_state.motor_state.dq).narrow(0, 0, params.num_of_dofs).unsqueeze(0);
  //  obs.actions = torch::tensor(robot_state.actions).narrow(0, 0, params.num_of_dofs).unsqueeze(0);

  //  [theta , theta_dot, l, l_dot]
  //  donot do like narrow(0, 0, params.num_of_vmc / 4) could make some err
  auto theta_tensor = torch::tensor(robot_state.vmc.theta).narrow(0, 0, 2).unsqueeze(0);
  auto dtheta_tensor = torch::tensor(robot_state.vmc.dtheta).narrow(0, 0, 2).unsqueeze(0);
  auto l_tensor = torch::tensor(robot_state.vmc.l).narrow(0, 0, 2).unsqueeze(0);
  auto dl_tensor = torch::tensor(robot_state.vmc.dl).narrow(0, 0, 2).unsqueeze(0);
  obs.vmc = torch::cat({theta_tensor, dtheta_tensor, l_tensor, dl_tensor}, 0).view({1, -1});
}