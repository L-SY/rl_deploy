//
// Created by lsy on 24-9-12.
//

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <rl_msgs/RobotState.h>
#include "rl_sdk/rl_sdk.hpp"

class RLInterface : public rl_sdk
{
public:
  RLInterface(ros::NodeHandle &nh) : nh_(nh)
  {
    // rl interface
    int frequency;
    nh_.param("frequency", frequency, 100);
    robotStateSub_ = nh_.subscribe("/rl/robot_state", 1, &RLInterface::robotStateCB, this);
    loop_rate_ = new ros::Rate(frequency);

    // read params from yaml
    nh_.param<std::string>("robot_name", robot_name, "");
    std::string rl_path;
    nh_.param<std::string>("rl_path", rl_path, "");
    std::string config_path = std::string(rl_path + "/config.yaml");
    ReadYaml(config_path);

    nh_.param<bool>("send_command", sendCommand_, false);
    if (sendCommand_)
      rlCommandPub_ = nh_.advertise<std_msgs::Float64MultiArray>("/rl/command", 1);
    else
      rlCommandPub_ = nh_.advertise<std_msgs::Float64MultiArray>("/rl/command_test", 1);

    // model
    std::string model_path = std::string(rl_path + "/" + params.model_name);
    model = torch::jit::load(model_path);
    //  model.dump(true,false,false);

    // init
    torch::autograd::GradMode::set_enabled(false);
    InitObservations();
    InitOutputs();
    InitControl();

    ROS_INFO_STREAM("rl_interface init success!");
  }

  ~RLInterface()
  {
    delete loop_rate_;
  }

  void update()
  {
      SetObservation();
      obs.actions = Forward();
      torch::Tensor origin_output_command = ComputeCommand(obs.actions);
      output_command = torch::clamp(origin_output_command, params.clip_actions_lower, params.clip_actions_upper);
//      output_command[0][0] = 0;
//      output_command[0][1] = 0.1;
//      output_command[0][2] = 0;
//      output_command[0][3] = 0;
//      output_command[0][4] = 0.1;
//      output_command[0][5] = 0;
      ROS_INFO_STREAM(output_command[0]);
//      ROS_INFO_STREAM(obs.vmc);
      std_msgs::Float64MultiArray commandMsg;
      for (int i = 0; i < params.num_of_dofs; ++i) {
        commandMsg.data.push_back(output_command[0][i].item<double>());
      }
      rlCommandPub_.publish(commandMsg);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher rlCommandPub_;
  ros::Subscriber robotStateSub_;
  ros::Rate *loop_rate_;

  rl_msgs::RobotState robotStateMsg_;

  void robotStateCB(const rl_msgs::RobotState&msg)
  {
    if(params.framework == "isaacgym")
    {
      robot_state.imu.quaternion[3] = msg.imu_states.orientation.w;
      robot_state.imu.quaternion[0] = msg.imu_states.orientation.x;
      robot_state.imu.quaternion[1] = msg.imu_states.orientation.y;
      robot_state.imu.quaternion[2] = msg.imu_states.orientation.z;
    }
    else if(params.framework == "isaacsim")
    {
      robot_state.imu.quaternion[0] = msg.imu_states.orientation.w;
      robot_state.imu.quaternion[1] = msg.imu_states.orientation.x;
      robot_state.imu.quaternion[2] = msg.imu_states.orientation.y;
      robot_state.imu.quaternion[3] = msg.imu_states.orientation.z;
    }

    robot_state.imu.gyroscope[0] = msg.imu_states.angular_velocity.x;
    robot_state.imu.gyroscope[1] = msg.imu_states.angular_velocity.y;
    robot_state.imu.gyroscope[2] = msg.imu_states.angular_velocity.z;

    for(int i = 0; i < params.num_of_dofs; ++i)
    {
      robot_state.motor_state.q[i] = msg.joint_states.position[i];
      robot_state.motor_state.dq[i] = msg.joint_states.velocity[i];
      robot_state.motor_state.tauEst[i] = msg.joint_states.effort[i];
    }

    if (params.use_vmc)
    {
      // left_theta, left_theta_dot , left_l, left_l_dot,
      robot_state.vmc.theta[0] = msg.left.theta;
      robot_state.vmc.theta[1] = msg.right.theta;
      robot_state.vmc.dtheta[0] = msg.left.theta_dot;
      robot_state.vmc.dtheta[1] = msg.right.theta_dot;
      robot_state.vmc.l[0] = msg.left.l;
      robot_state.vmc.l[1] = msg.right.l;
      robot_state.vmc.dl[0] = msg.left.l_dot;
      robot_state.vmc.dl[1] = msg.right.l_dot;
    }

    control.vel_x = msg.commands[0];
    control.vel_yaw = msg.commands[1];
    control.pos_z = msg.commands[2];

//    for(int i = 0; i < params.num_of_dofs; ++i)
//    {
//      robot_state.actions[i] = msg.actions[i];
//    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rl_interface");
  ros::NodeHandle nh("~");

  RLInterface rl_interface(nh);
  // the frequence should be gym 1/(dt*decimation)
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    rl_interface.update();
    loop_rate.sleep();
  }
  return 0;
}

