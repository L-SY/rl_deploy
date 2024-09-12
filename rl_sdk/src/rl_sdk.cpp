#include "rl_sdk/rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "iostream"
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

    for(const std::string& observation : params.observations)
    {
        if(observation == "lin_vel")
        {
            obs_list.push_back(obs.lin_vel * params.lin_vel_scale);
        }
        else if(observation == "ang_vel")
        {
             obs_list.push_back(obs.ang_vel * params.ang_vel_scale); // TODO is QuatRotateInverse necessery?
//            obs_list.push_back(QuatRotateInverse(obs.base_quat, obs.ang_vel, params.framework) * params.ang_vel_scale);
        }
        else if(observation == "gravity_vec")
        {
            obs_list.push_back(QuatRotateInverse(obs.base_quat, obs.gravity_vec, params.framework));
        }
        else if(observation == "commands")
        {
            obs_list.push_back(obs.commands * params.commands_scale);
        }
        else if(observation == "dof_pos")
        {
            obs_list.push_back((obs.dof_pos - params.default_dof_pos) * params.dof_pos_scale);
        }
        else if(observation == "dof_vel")
        {
            obs_list.push_back(obs.dof_vel * params.dof_vel_scale);
        }
        else if(observation == "actions")
        {
            obs_list.push_back(obs.actions);
        }
    }

    torch::Tensor obs = torch::cat(obs_list, 1);
    torch::Tensor clamped_obs = torch::clamp(obs, -params.clip_obs, params.clip_obs);
    return clamped_obs;
}

void rl_sdk::InitObservations()
{
    obs.lin_vel = torch::tensor({{0.0, 0.0, 0.0}});
    obs.ang_vel = torch::tensor({{0.0, 0.0, 0.0}});
    // No need change to -9.81
    obs.gravity_vec = torch::tensor({{0.0, 0.0, -1.0}});
    obs.commands = torch::tensor({{0.0, 0.0, 0.18}});
    obs.base_quat = torch::tensor({{0.0, 0.0, 0.0, 1.0}});
    obs.dof_pos = params.default_dof_pos;
    obs.dof_vel = torch::zeros({1, params.num_of_dofs});
    obs.actions = torch::zeros({1, params.num_of_dofs});
}

void rl_sdk::InitOutputs()
{
    output_command = torch::zeros({1, params.num_of_dofs});
    output_dof_pos = params.default_dof_pos;
}

void rl_sdk::InitControl()
{
    control.vel_x = 0.0;
    control.vel_yaw = 0.0;
    control.pos_z = 0.0;
}

torch::Tensor rl_sdk::ComputeCommand(torch::Tensor actions)
{
    torch::Tensor actions_scaled = actions * params.action_scale;
    torch::Tensor pos_torques = actions_scaled + params.default_dof_pos ;
    output_command = pos_torques;
    return output_command;
}

torch::Tensor rl_sdk::QuatRotateInverse(torch::Tensor q, torch::Tensor v, const std::string& framework)
{
    torch::Tensor q_w;
    torch::Tensor q_vec;
    if(framework == "isaacsim")
    {
        q_w = q.index({torch::indexing::Slice(), 0});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(1, 4)});
    }
    else if(framework == "isaacgym")
    {
        q_w = q.index({torch::indexing::Slice(), 3});
        q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(0, 3)});
    }
    c10::IntArrayRef shape = q.sizes();
    
    torch::Tensor a = v * (2.0 * torch::pow(q_w, 2) - 1.0).unsqueeze(-1);
    torch::Tensor b = torch::cross(q_vec, v, -1) * q_w.unsqueeze(-1) * 2.0;
    torch::Tensor c = q_vec * torch::bmm(q_vec.view({shape[0], 1, 3}), v.view({shape[0], 3, 1})).squeeze(-1) * 2.0;
    return a - b + c;
}

void rl_sdk::TorqueProtect(torch::Tensor origin_output_torques)
{
    std::vector<int> out_of_range_indices;
    std::vector<double> out_of_range_values;
    for(int i = 0; i < origin_output_torques.size(1); ++i)
    {
        double torque_value = origin_output_torques[0][i].item<double>();
        double limit_lower = -params.torque_limits[0][i].item<double>();
        double limit_upper = params.torque_limits[0][i].item<double>();

        if(torque_value < limit_lower || torque_value > limit_upper)
        {
            out_of_range_indices.push_back(i);
            out_of_range_values.push_back(torque_value);
        }
    }
    if(!out_of_range_indices.empty())
    {
        for(int i = 0; i < out_of_range_indices.size(); ++i)
        {
            int index = out_of_range_indices[i];
            double value = out_of_range_values[i];
            double limit_lower = -params.torque_limits[0][index].item<double>();
            double limit_upper = params.torque_limits[0][index].item<double>();

            std::cout << LOGGER::WARNING << "Torque(" << index+1 << ")=" << value << " out of range(" << limit_lower << ", " << limit_upper << ")" << std::endl;
        }
        // Just a reminder, no protection
        // control.control_state = STATE_POS_GETDOWN;
        // std::cout << LOGGER::INFO << "Switching to STATE_POS_GETDOWN"<< std::endl;
    }
}

#include <termios.h>
#include <sys/ioctl.h>
static bool kbhit()
{
    termios term;
    tcgetattr(0, &term);
    
    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);
    
    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);
    
    tcsetattr(0, TCSANOW, &term);
    
    return byteswaiting > 0;
}

template<typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node& node)
{
    std::vector<T> values;
    for(const auto& val : node)
    {
        values.push_back(val.as<T>());
    }
    return values;
}

template<typename T>
std::vector<T> ReadVectorFromYaml(const YAML::Node& node, const std::string& framework, const int& rows, const int& cols)
{
    std::vector<T> values;
    for(const auto& val : node)
    {
        values.push_back(val.as<T>());
    }

    if(framework == "isaacsim")
    {
        std::vector<T> transposed_values(cols * rows);
        for(int r = 0; r < rows; ++r)
        {
            for(int c = 0; c < cols; ++c)
            {
                transposed_values[c * rows + r] = values[r * cols + c];
            }
        }
        return transposed_values;
    }
    else if(framework == "isaacgym")
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
    } catch(YAML::BadFile &e)
    {
        std::cout << LOGGER::ERROR << "The file '" << config_path << "' does not exist" << std::endl;
        return;
    }

    params.model_name = config["model_name"].as<std::string>();
    params.framework = config["framework"].as<std::string>();
    int rows = config["rows"].as<int>();
    int cols = config["cols"].as<int>();
    params.use_history = config["use_history"].as<bool>();
    params.dt = config["dt"].as<double>();
    params.decimation = config["decimation"].as<int>();
    params.num_observations = config["num_observations"].as<int>();
    params.observations = ReadVectorFromYaml<std::string>(config["observations"]);
    params.clip_obs = config["clip_obs"].as<double>();
    if(config["clip_actions_lower"].IsNull() && config["clip_actions_upper"].IsNull())
    {
        params.clip_actions_upper = torch::tensor({}).view({1, -1});
        params.clip_actions_lower = torch::tensor({}).view({1, -1});
    }
    else
    {
        params.clip_actions_upper = torch::tensor(ReadVectorFromYaml<double>(config["clip_actions_upper"], params.framework, rows, cols)).view({1, -1});
        params.clip_actions_lower = torch::tensor(ReadVectorFromYaml<double>(config["clip_actions_lower"], params.framework, rows, cols)).view({1, -1});
    }
    params.action_scale = config["action_scale"].as<double>();
//    params.hip_scale_reduction = config["hip_scale_reduction"].as<double>();
//    params.hip_scale_reduction_indices = ReadVectorFromYaml<int>(config["hip_scale_reduction_indices"]);
    params.num_of_dofs = config["num_of_dofs"].as<int>();
    params.lin_vel_scale = config["lin_vel_scale"].as<double>();
    params.ang_vel_scale = config["ang_vel_scale"].as<double>();
    params.dof_pos_scale = config["dof_pos_scale"].as<double>();
    params.dof_vel_scale = config["dof_vel_scale"].as<double>();
//    params.commands_scale = torch::tensor(ReadVectorFromYaml<double>(config["commands_scale"])).view({1, -1});
//    params.commands_scale = torch::tensor({params.lin_vel_scale, params.lin_vel_scale, params.ang_vel_scale});
    params.commands_scale = torch::tensor({params.lin_vel_scale, params.ang_vel_scale, 5.0});
//    params.fixed_kp = torch::tensor(ReadVectorFromYaml<double>(config["fixed_kp"], params.framework, rows, cols)).view({1, -1});
//    params.fixed_kd = torch::tensor(ReadVectorFromYaml<double>(config["fixed_kd"], params.framework, rows, cols)).view({1, -1});
    params.torque_limits = torch::tensor(ReadVectorFromYaml<double>(config["torque_limits"], params.framework, rows, cols)).view({1, -1});
    params.default_dof_pos = torch::tensor(ReadVectorFromYaml<double>(config["default_dof_pos"], params.framework, rows, cols)).view({1, -1});
//    params.joint_controller_names = ReadVectorFromYaml<std::string>(config["joint_controller_names"], params.framework, rows, cols);
}

torch::Tensor rl_sdk::Forward()
{
  torch::autograd::GradMode::set_enabled(false);
  torch::Tensor clamped_obs = ComputeObservation();

  torch::Tensor actions;
  actions = model.forward({clamped_obs}).toTensor();

  if(params.clip_actions_upper.numel() != 0 && params.clip_actions_lower.numel() != 0)
  {
    return torch::clamp(actions, params.clip_actions_lower, params.clip_actions_upper);
  }
  else
  {
    return actions;
  }
}

void rl_sdk::SetObservation()
{
  obs.ang_vel = torch::tensor(robot_state.imu.gyroscope).unsqueeze(0);
  obs.commands = torch::tensor({{control.vel_x, control.vel_yaw, control.pos_z}});
  obs.base_quat = torch::tensor(robot_state.imu.quaternion).unsqueeze(0);
  obs.dof_pos = torch::tensor(robot_state.motor_state.q).narrow(0, 0, params.num_of_dofs).unsqueeze(0);
  obs.dof_vel = torch::tensor(robot_state.motor_state.dq).narrow(0, 0, params.num_of_dofs).unsqueeze(0);
}