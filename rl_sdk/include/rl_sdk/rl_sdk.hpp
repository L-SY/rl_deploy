#pragma once

#include <torch/script.h>
#include <iostream>
#include <string>
#include <unistd.h>

#include <yaml-cpp/yaml.h>
#include "observation_buffer.hpp"

// ref: https://github.com/fan-ziqi/rl_sar

namespace LOGGER {
    const char* const INFO    = "\033[0;37m[INFO]\033[0m ";
    const char* const WARNING = "\033[0;33m[WARNING]\033[0m ";
    const char* const ERROR   = "\033[0;31m[ERROR]\033[0m ";
    const char* const DEBUG   = "\033[0;32m[DEBUG]\033[0m ";
}


template<typename T>
struct RobotState
{
    struct IMU
    {
        std::vector<T> quaternion = {1.0, 0.0, 0.0, 0.0}; // w, x, y, z
        std::vector<T> gyroscope = {0.0, 0.0, 0.0};
        std::vector<T> accelerometer = {0.0, 0.0, 0.0};
    } imu;

    struct MotorState
    {
        std::vector<T> q = std::vector<T>(32, 0.0);
        std::vector<T> dq = std::vector<T>(32, 0.0);
        std::vector<T> ddq = std::vector<T>(32, 0.0);
        std::vector<T> tauEst = std::vector<T>(32, 0.0);
        std::vector<T> cur = std::vector<T>(32, 0.0);
    } motor_state;

    struct VMC
    {
      std::vector<T> left = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // left_l, left_l_dot, left_theta, left_theta_dot
      std::vector<T> right = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // right_l, right_l_dot, right_theta, right_theta_dot
    } vmc;
};

struct Control
{
    double vel_x = 0.0;
    double vel_yaw = 0.0;
    double pos_z = 0.0;
};

struct ModelParams
{
    std::string model_name;
    std::string framework;
    bool use_history;
    double dt;
    int decimation;
    int num_observations;
    std::vector<std::string> observations;
    double damping;
    double stiffness;
    double action_scale;
    std::vector<int> hip_scale_reduction_indices;
    int num_of_dofs;
    double lin_vel_scale;
    double ang_vel_scale;
    double dof_pos_scale;
    double dof_vel_scale;
    double clip_obs;
    torch::Tensor clip_actions_upper;
    torch::Tensor clip_actions_lower;
    torch::Tensor torque_limits;
    torch::Tensor commands_scale;
    torch::Tensor default_dof_pos;

    //  For vmc
    bool use_vmc;
    int num_of_vmc;
    double l_scale;
    double l_dot_scale;
    double theta_scale;
    double theta_dot_scale;
    double l_offset;
    double action_scale_l;
    double action_scale_theta;
    double action_scale_vel;
};

struct Observations
{
    torch::Tensor lin_vel;           
    torch::Tensor ang_vel;      
    torch::Tensor gravity_vec;      
    torch::Tensor commands;        
    torch::Tensor base_quat;
    torch::Tensor vmc;
    torch::Tensor dof_pos;           
    torch::Tensor dof_vel;           
    torch::Tensor actions;
};

class rl_sdk
{
public:
    rl_sdk(){};
    ~rl_sdk(){};

    ModelParams params;
    Observations obs;

    RobotState<double> robot_state;

    // init
    void InitObservations();
    void InitOutputs();
    void InitControl();

    // rl functions
    torch::Tensor Forward();
    torch::Tensor ComputeObservation();
    void SetObservation();
    torch::Tensor ComputeCommand(torch::Tensor actions);
    torch::Tensor QuatRotateInverse(torch::Tensor q, torch::Tensor v, const std::string& framework);

    // control
    Control control;

    // yaml params
    void ReadYaml(const std::string config_path);

    // others
    std::string robot_name;

    // protect func
    void TorqueProtect(torch::Tensor origin_output_torques);

//protected:
    // rl module
    torch::jit::script::Module model;
    // output buffer
    torch::Tensor output_command;
    torch::Tensor output_dof_pos;
};