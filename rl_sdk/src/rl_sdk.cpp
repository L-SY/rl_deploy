#include "rl_sdk/rl_sdk.hpp"
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

    for(const std::string& observation : params.observations)
    {
        if(observation == "lin_vel")
        {
            obs_list.push_back(obs.lin_vel * params.lin_vel_scale);
        }
        else if(observation == "ang_vel")
        {
            // obs_list.push_back(obs.ang_vel * params.ang_vel_scale); // TODO is QuatRotateInverse necessery?
            obs_list.push_back(QuatRotateInverse(obs.base_quat, obs.ang_vel, params.framework) * params.ang_vel_scale);
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
    obs.gravity_vec = torch::tensor({{0.0, 0.0, -1.0}});
    obs.commands = torch::tensor({{0.0, 0.0, 0.0}});
    obs.base_quat = torch::tensor({{0.0, 0.0, 0.0, 1.0}});
    obs.dof_pos = params.default_dof_pos;
    obs.dof_vel = torch::zeros({1, params.num_of_dofs});
    obs.actions = torch::zeros({1, params.num_of_dofs});
}

void rl_sdk::InitOutputs()
{
    output_torques = torch::zeros({1, params.num_of_dofs});
    output_dof_pos = params.default_dof_pos;
}

void rl_sdk::InitControl()
{
    control.control_state = STATE_WAITING;
    control.vel_x = 0.0;
    control.vel_yaw = 0.0;
    control.pos_z = 0.0;
}

torch::Tensor rl_sdk::ComputeTorques(torch::Tensor actions)
{
    torch::Tensor actions_scaled = actions * params.action_scale;
    torch::Tensor output_torques = params.rl_kp * (actions_scaled + params.default_dof_pos - obs.dof_pos) - params.rl_kd * obs.dof_vel;
    return output_torques;
}

torch::Tensor rl_sdk::ComputePosition(torch::Tensor actions)
{
    torch::Tensor actions_scaled = actions * params.action_scale;
    return actions_scaled + params.default_dof_pos;
}

torch::Tensor rl_sdk::ComputeVelocity(torch::Tensor actions)
{
//  TODO: is right?
  torch::Tensor actions_scaled = actions * params.action_scale;
  return actions_scaled * params.dof_vel_scale;
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

void rl_sdk::StateController(const RobotState<double> *state, RobotCommand<double> *command)
{
    static RobotState<double> start_state;
    static RobotState<double> now_state;
    static float getup_percent = 0.0;
    static float getdown_percent = 0.0;

    // waiting
    if(running_state == STATE_WAITING)
    {
        for(int i = 0; i < params.num_of_dofs; ++i)
        {
            command->motor_command.q[i] = state->motor_state.q[i];
        }
        if(control.control_state == STATE_POS_GETUP)
        {
            control.control_state = STATE_WAITING;
            getup_percent = 0.0;
            for(int i = 0; i < params.num_of_dofs; ++i)
            {
                now_state.motor_state.q[i] = state->motor_state.q[i];
                start_state.motor_state.q[i] = now_state.motor_state.q[i];
            }
            running_state = STATE_POS_GETUP;
            std::cout << std::endl << LOGGER::INFO << "Switching to STATE_POS_GETUP" << std::endl;
        }
    }
    // stand up (position control)
    else if(running_state == STATE_POS_GETUP)
    {
        if(getup_percent < 1.0)
        {
            getup_percent += 1 / 500.0;
            getup_percent = getup_percent > 1.0 ? 1.0 : getup_percent;
            for(int i = 0; i < params.num_of_dofs; ++i)
            {
                command->motor_command.q[i] = (1 - getup_percent) * now_state.motor_state.q[i] + getup_percent * params.default_dof_pos[0][i].item<double>();
                command->motor_command.dq[i] = 0;
                command->motor_command.kp[i] = params.fixed_kp[0][i].item<double>();
                command->motor_command.kd[i] = params.fixed_kd[0][i].item<double>();
                command->motor_command.tau[i] = 0;
            }
            std::cout << "\r" << std::flush << LOGGER::INFO << "Getting up " << std::fixed << std::setprecision(2) << getup_percent * 100.0 << std::flush;
        }
        if(control.control_state == STATE_RL_INIT)
        {
            control.control_state = STATE_WAITING;
            running_state = STATE_RL_INIT;
            std::cout << std::endl << LOGGER::INFO << "Switching to STATE_RL_INIT" << std::endl;
        }
        else if(control.control_state == STATE_POS_GETDOWN)
        {
            control.control_state = STATE_WAITING;
            getdown_percent = 0.0;
            for(int i = 0; i < params.num_of_dofs; ++i)
            {
                now_state.motor_state.q[i] = state->motor_state.q[i];
            }
            running_state = STATE_POS_GETDOWN;
            std::cout << std::endl << LOGGER::INFO << "Switching to STATE_POS_GETDOWN" << std::endl;
        }
    }
    // init obs and start rl loop
    else if(running_state == STATE_RL_INIT)
    {
        if(getup_percent == 1)
        {
            InitObservations();
            InitOutputs();
            InitControl();
            running_state = STATE_RL_RUNNING;
            std::cout << std::endl << LOGGER::INFO << "Switching to STATE_RL_RUNNING" << std::endl;
        }
    }
    // rl loop
    else if(running_state == STATE_RL_RUNNING)
    {
        std::cout << "\r" << std::flush << LOGGER::INFO << "rl_sdk Controller x:" << control.vel_x << " y:" << control.vel_yaw << " yaw:" << control.pos_z << std::flush;
        for(int i = 0; i < params.num_of_dofs; ++i)
        {
            command->motor_command.q[i] = output_dof_pos[0][i].item<double>();
            command->motor_command.dq[i] = 0;
            command->motor_command.kp[i] = params.rl_kp[0][i].item<double>();
            command->motor_command.kd[i] = params.rl_kd[0][i].item<double>();
            command->motor_command.tau[i] = 0;
        }
        if(control.control_state == STATE_POS_GETDOWN)
        {
            control.control_state = STATE_WAITING;
            getdown_percent = 0.0;
            for(int i = 0; i < params.num_of_dofs; ++i)
            {
                now_state.motor_state.q[i] = state->motor_state.q[i];
            }
            running_state = STATE_POS_GETDOWN;
            std::cout << std::endl << LOGGER::INFO << "Switching to STATE_POS_GETDOWN" << std::endl;
        }
        else if(control.control_state == STATE_POS_GETUP)
        {
            control.control_state = STATE_WAITING;
            getup_percent = 0.0;
            for(int i = 0; i < params.num_of_dofs; ++i)
            {
                now_state.motor_state.q[i] = state->motor_state.q[i];
            }
            running_state = STATE_POS_GETUP;
            std::cout << std::endl << LOGGER::INFO << "Switching to STATE_POS_GETUP" << std::endl;
        }
    }
    // get down (position control)
    else if(running_state == STATE_POS_GETDOWN)
    {
        if(getdown_percent < 1.0)
        {
            getdown_percent += 1 / 500.0;
            getdown_percent = getdown_percent > 1.0 ? 1.0 : getdown_percent;
            for(int i = 0; i < params.num_of_dofs; ++i)
            {
                command->motor_command.q[i] = (1 - getdown_percent) * now_state.motor_state.q[i] + getdown_percent * start_state.motor_state.q[i];
                command->motor_command.dq[i] = 0;
                command->motor_command.kp[i] = params.fixed_kp[0][i].item<double>();
                command->motor_command.kd[i] = params.fixed_kd[0][i].item<double>();
                command->motor_command.tau[i] = 0;
            }
            std::cout << "\r" << std::flush << LOGGER::INFO << "Getting down " << std::fixed << std::setprecision(2) << getdown_percent * 100.0 << std::flush;
        }
        if(getdown_percent == 1)
        {
            InitObservations();
            InitOutputs();
            InitControl();
            running_state = STATE_WAITING;
            std::cout << std::endl << LOGGER::INFO << "Switching to STATE_WAITING" << std::endl;
        }
    }
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

void rl_sdk::KeyboardInterface()
{
    if(kbhit())
    {
        int c = fgetc(stdin);
        switch(c)
        {
            case '0': control.control_state = STATE_POS_GETUP; break;
            case 'p': control.control_state = STATE_RL_INIT; break;
            case '1': control.control_state = STATE_POS_GETDOWN; break;
            case 'q': break;
            case 'w': control.vel_x += 0.1; break;
            case 's': control.vel_x -= 0.1; break;
            case 'a': control.vel_yaw += 0.1; break;
            case 'd': control.vel_yaw -= 0.1; break;
            case 'i': break;
            case 'k': break;
            case 'j': control.pos_z += 0.1; break;
            case 'l': control.pos_z -= 0.1; break;
            case ' ': control.vel_x = 0; control.vel_yaw = 0; control.pos_z = 0; break;
            case 'r': control.control_state = STATE_RESET_SIMULATION; break;
            case '\n': control.control_state = STATE_TOGGLE_SIMULATION; break;
            default: break;
        }
    }
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
    // params.commands_scale = torch::tensor(ReadVectorFromYaml<double>(config["commands_scale"])).view({1, -1});
    params.commands_scale = torch::tensor({params.lin_vel_scale, params.lin_vel_scale, params.ang_vel_scale});
    params.rl_kp = torch::tensor(ReadVectorFromYaml<double>(config["rl_kp"], params.framework, rows, cols)).view({1, -1});
    params.rl_kd = torch::tensor(ReadVectorFromYaml<double>(config["rl_kd"], params.framework, rows, cols)).view({1, -1});
//    params.fixed_kp = torch::tensor(ReadVectorFromYaml<double>(config["fixed_kp"], params.framework, rows, cols)).view({1, -1});
//    params.fixed_kd = torch::tensor(ReadVectorFromYaml<double>(config["fixed_kd"], params.framework, rows, cols)).view({1, -1});
    params.torque_limits = torch::tensor(ReadVectorFromYaml<double>(config["torque_limits"], params.framework, rows, cols)).view({1, -1});
    params.default_dof_pos = torch::tensor(ReadVectorFromYaml<double>(config["default_dof_pos"], params.framework, rows, cols)).view({1, -1});
//    params.joint_controller_names = ReadVectorFromYaml<std::string>(config["joint_controller_names"], params.framework, rows, cols);
}

void rl_sdk::CSVInit(std::string robot_name)
{
    csv_filename = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/models/" + robot_name + "/motor";

    // Uncomment these lines if need timestamp for file name
    // auto now = std::chrono::system_clock::now();
    // std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    // std::stringstream ss;
    // ss << std::put_time(std::localtime(&now_c), "%Y%m%d%H%M%S");
    // std::string timestamp = ss.str();
    // csv_filename += "_" + timestamp;

    csv_filename += ".csv";
    std::ofstream file(csv_filename.c_str());

    for(int i = 0; i < 12; ++i) {file << "tau_cal_" << i << ",";}
    for(int i = 0; i < 12; ++i) {file << "tau_est_" << i << ",";}
    for(int i = 0; i < 12; ++i) {file << "joint_pos_" << i << ",";}
    for(int i = 0; i < 12; ++i) {file << "joint_pos_target_" << i << ",";}
    for(int i = 0; i < 12; ++i) {file << "joint_vel_" << i << ",";}

    file << std::endl;

    file.close();
}

void rl_sdk::CSVLogger(torch::Tensor torque, torch::Tensor tau_est, torch::Tensor joint_pos, torch::Tensor joint_pos_target, torch::Tensor joint_vel)
{
    std::ofstream file(csv_filename.c_str(), std::ios_base::app);

    for(int i = 0; i < 12; ++i) {file << torque[0][i].item<double>() << ",";}
    for(int i = 0; i < 12; ++i) {file << tau_est[0][i].item<double>() << ",";}
    for(int i = 0; i < 12; ++i) {file << joint_pos[0][i].item<double>() << ",";}
    for(int i = 0; i < 12; ++i) {file << joint_pos_target[0][i].item<double>() << ",";}
    for(int i = 0; i < 12; ++i) {file << joint_vel[0][i].item<double>() << ",";}

    file << std::endl;

    file.close();
}

torch::Tensor rl_sdk::Forward(std::shared_ptr<torch::Tensor> history_obs, std::shared_ptr<ObservationBuffer> history_obs_buf)
{
  torch::autograd::GradMode::set_enabled(false);
  torch::Tensor clamped_obs = ComputeObservation();
  std::cout << "Clamped Observation Shape: " << clamped_obs.sizes() << std::endl;
  torch::Tensor actions;
  if(params.use_history)
  {
    history_obs_buf->insert(clamped_obs);
    *history_obs = history_obs_buf->get_obs_vec({0, 1, 2});
    actions = model.forward({*history_obs}).toTensor();
  }
  else
  {
    std::cout << "before forward"<< std::endl;
    actions = model.forward({clamped_obs}).toTensor();
    std::cout << "after forward"<< std::endl;
  }

  std::cout << "2222222222" << std::endl;
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