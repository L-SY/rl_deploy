//
// Created by lsy on 24-9-12.
//

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <rl_msgs/RobotState.h>
#include <rl_msgs/vmcAction.h>
#include <rl_msgs/vmcObs.h>
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
    rlCommandPub_ = nh_.advertise<std_msgs::Float64MultiArray>("/rl/command", 1);
    vmcObsPub_ = nh_.advertise<rl_msgs::vmcObs>("/rl/vmc/obs", 1);
    vmcRawObsPub_ = nh_.advertise<rl_msgs::vmcObs>("/rl/vmc/raw_obs", 1);
    vmcActionPub_ = nh_.advertise<rl_msgs::vmcAction>("/rl/vmc/action", 1);
    loop_rate_ = new ros::Rate(frequency);

    // read params from yaml
    nh_.param<std::string>("robot_name", robot_name, "");
    std::string rl_path;
    nh_.param<std::string>("rl_path", rl_path, "");
    std::string config_path = std::string(rl_path + "/config_vmc.yaml");
    ReadYaml(config_path);

    // model
    std::string model_path = std::string(rl_path + "/" + params.model_name);
    model = torch::jit::load(model_path);
    //  model.dump(true,false,false);

    // init
    torch::autograd::GradMode::set_enabled(false);
    InitObservations();
    InitOutputs();
    InitControl();

    vmcObsMsg_.base_ang_vel.resize(3);
    vmcObsMsg_.projected_gravity.resize(3);
    vmcObsMsg_.commands.resize(3);
    vmcObsMsg_.theta.resize(2);
    vmcObsMsg_.theta_dot.resize(2);
    vmcObsMsg_.l.resize(2);
    vmcObsMsg_.l_dot.resize(2);
    vmcObsMsg_.wheel_pos.resize(2);
    vmcObsMsg_.wheel_vel.resize(2);
    vmcObsMsg_.actions.resize(6);

    ROS_INFO_STREAM("rl_interface init success!");
  }

  ~RLInterface()
  {
    delete loop_rate_;
  }

  void update()
  {
      SetObservation();
      vmcObsMsg_.base_ang_vel[0] = obs.ang_vel[0][0].item<double>() ;
      vmcObsMsg_.base_ang_vel[1] = obs.ang_vel[0][1].item<double>() ;
      vmcObsMsg_.base_ang_vel[2] = obs.ang_vel[0][2].item<double>() ;

      vmcObsMsg_.projected_gravity[0] = obs.real_gravity_vec[0][0].item<double>();
      vmcObsMsg_.projected_gravity[1] = obs.real_gravity_vec[0][1].item<double>();
      vmcObsMsg_.projected_gravity[2] = obs.real_gravity_vec[0][2].item<double>();

      vmcObsMsg_.commands[0] = obs.commands[0][0].item<double>() ;
      vmcObsMsg_.commands[1] = obs.commands[0][1].item<double>() ;
      vmcObsMsg_.commands[2] = obs.commands[0][2].item<double>() ;

      vmcObsMsg_.theta[0] = obs.vmc[0][0].item<double>();
      vmcObsMsg_.theta[1] = obs.vmc[0][1].item<double>();
      vmcObsMsg_.theta_dot[0] = obs.vmc[0][2].item<double>();
      vmcObsMsg_.theta_dot[1] = obs.vmc[0][3].item<double>();
      vmcObsMsg_.l[0] = obs.vmc[0][4].item<double>();
      vmcObsMsg_.l[1] = obs.vmc[0][5].item<double>();
      vmcObsMsg_.l_dot[0] = obs.vmc[0][6].item<double>();
      vmcObsMsg_.l_dot[1] = obs.vmc[0][7].item<double>();

      vmcObsMsg_.wheel_pos[0] = obs.dof_pos[0][2].item<double>() ;
      vmcObsMsg_.wheel_pos[1] = obs.dof_pos[0][5].item<double>() ;
      vmcObsMsg_.wheel_vel[0] = obs.dof_vel[0][2].item<double>() ;
      vmcObsMsg_.wheel_vel[1] = obs.dof_vel[0][5].item<double>() ;

      vmcObsMsg_.actions[0] = obs.actions[0][0].item<double>() ;
      vmcObsMsg_.actions[1] = obs.actions[0][1].item<double>() ;
      vmcObsMsg_.actions[2] = obs.actions[0][2].item<double>() ;
      vmcObsMsg_.actions[3] = obs.actions[0][3].item<double>() ;
      vmcObsMsg_.actions[4] = obs.actions[0][4].item<double>() ;
      vmcObsMsg_.actions[5] = obs.actions[0][5].item<double>() ;
      vmcRawObsPub_.publish(vmcObsMsg_);

      vmcObsMsg_.base_ang_vel[0] *= params.ang_vel_scale;
      vmcObsMsg_.base_ang_vel[1] *= params.ang_vel_scale;
      vmcObsMsg_.base_ang_vel[2] *= params.ang_vel_scale;

      vmcObsMsg_.commands[0] *= params.commands_scale[0].item<double>();
      vmcObsMsg_.commands[1] *= params.commands_scale[1].item<double>();
      vmcObsMsg_.commands[2] *= params.commands_scale[2].item<double>();

      vmcObsMsg_.theta[0] *= params.theta_scale;
      vmcObsMsg_.theta[1] *= params.theta_scale;
      vmcObsMsg_.theta_dot[0] *= params.theta_dot_scale;
      vmcObsMsg_.theta_dot[1] *= params.theta_dot_scale;
      vmcObsMsg_.l[0] *= params.l_scale;
      vmcObsMsg_.l[1] *= params.l_scale;
      vmcObsMsg_.l_dot[0] *= params.l_dot_scale;
      vmcObsMsg_.l_dot[1] *= params.l_dot_scale;

      vmcObsMsg_.wheel_pos[0] *= params.dof_pos_scale;
      vmcObsMsg_.wheel_pos[1] *= params.dof_pos_scale;
      vmcObsMsg_.wheel_vel[0] *= params.dof_vel_scale;
      vmcObsMsg_.wheel_vel[1] *= params.dof_vel_scale;

      vmcObsMsg_.actions[0] *= params.action_scale_theta;
      vmcObsMsg_.actions[1] *= params.action_scale_l;
      vmcObsMsg_.actions[1] += params.l_offset;
      vmcObsMsg_.actions[2] *= params.action_scale_vel;
      vmcObsMsg_.actions[3] *= params.action_scale_theta;
      vmcObsMsg_.actions[4] *= params.action_scale_l;
      vmcObsMsg_.actions[4] += params.l_offset;
      vmcObsMsg_.actions[5] *= params.action_scale_vel;
      vmcObsPub_.publish(vmcObsMsg_);

      torch::Tensor actions = Forward();
      obs.actions = actions;
      torch::Tensor origin_output_command = ComputeCommand(actions);
      output_command = torch::clamp(origin_output_command, params.clip_actions_lower, params.clip_actions_upper);
//      output_command[0][0] = -1.3;
//      output_command[0][1] = 0.2;
//      output_command[0][2] = 0;
//      output_command[0][3] = -1.3;
//      output_command[0][4] = 0.2;
//      output_command[0][5] = 0;
//      ROS_INFO_STREAM(output_command[0]);
      vmcActionMsg_.theta_left = output_command[0][0].item<double>();
      vmcActionMsg_.l_left = output_command[0][1].item<double>();
      vmcActionMsg_.wheel_vel_left = output_command[0][2].item<double>();
      vmcActionMsg_.theta_right = output_command[0][3].item<double>();
      vmcActionMsg_.l_right = output_command[0][4].item<double>();
      vmcActionMsg_.wheel_vel_right = output_command[0][5].item<double>();
      vmcActionPub_.publish(vmcActionMsg_);

      std_msgs::Float64MultiArray commandMsg;
      for (int i = 0; i < params.num_of_dofs; ++i) {
        commandMsg.data.push_back(output_command[0][i].item<double>());
      }
      rlCommandPub_.publish(commandMsg);
  }

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
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher rlCommandPub_;
  ros::Subscriber robotStateSub_;
  ros::Rate *loop_rate_;

  rl_msgs::vmcObs vmcObsMsg_;
  rl_msgs::vmcAction vmcActionMsg_;
  rl_msgs::RobotState robotStateMsg_;
  ros::Publisher vmcObsPub_, vmcRawObsPub_, vmcActionPub_;
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

