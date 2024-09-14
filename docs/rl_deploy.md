## rl_deploy

#### rl_sdk

> 用`libtorch`完成和`.pt`文件的交互

主要内容如下：

```
model = torch::jit::load(model_path);
actions = this->model.forward({clamped_obs}).toTensor();
```

同时通过从`rosparam`获取`.yaml`文件中的参数（issacgym中训练时用到的各种scales等）。

不断的进行从controller中获取训练时obs所需要的信息（电机角度，速度和imu数据），并通过前向传播得到action，经过处理后通过topic发送出来。



#### robot_gazebo

> 完成ros和gazebo之间的交互

​	通过继承`gazebo_ros_control::DefaultRobotHWSim`[https://github.com/ros-controls/gazebo_ros_control]完成与`Gazebo`的交互。当我们通过`mon launch diablo load_gazebo.launch`加载`gazebo`时启动了一个线程，在该线程中不断执行如下循环（内容详见[https://github1s.com/ros-controls/gazebo_ros_control/blob/HEAD/src/gazebo_ros_control_plugin.cpp]):

```cpp
void GazeboRosControlPlugin::Update()
{
  // Get the simulation time and period
  gazebo::common::Time gz_time_now = parent_model_->GetWorld()->GetSimTime();
  ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
  ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  robot_hw_sim_->eStopActive(e_stop_active_);

  // Check if we should update the controllers
  if(sim_period >= control_period_) {
    // Store this simulation time
    last_update_sim_time_ros_ = sim_time_ros;

    // Update the robot simulation with the state of the gazebo model
    robot_hw_sim_->readSim(sim_time_ros, sim_period);

    // Compute the controller commands
    bool reset_ctrlrs;
    if (e_stop_active_)
    {
      reset_ctrlrs = false;
      last_e_stop_active_ = true;
    }
    else
    {
      if (last_e_stop_active_)
      {
        reset_ctrlrs = true;
        last_e_stop_active_ = false;
      }
      else
      {
        reset_ctrlrs = false;
      }
    }
    controller_manager_->update(sim_time_ros, sim_period, reset_ctrlrs);
  }

  // Update the gazebo model with the result of the controller
  // computation
  robot_hw_sim_->writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
  last_write_sim_time_ros_ = sim_time_ros;
}
```

循环执行`readSim`,`update`,`writeSim`三个函数

`readSim`：完成gazebo中电机，imu等定义硬件设备的信息读取

`update`：完成控制器的更新（会调用所有加载的控制器的`update`函数，例如下面`wheeled_bipedal_rl_controller`）

`writeSim`：将在`update`中的更新的电机命令发送给Gazebo中电机



#### wheeled_bipedal_rl_controller

> 与rl_sdk通过topic交互，并利用底层控制器计算力矩发送给gazebo中的电机

​	控制器通过`/rl/robot_state`和`/rl/command`两个话题完成和`rl_sdk`的交互，具体实现如下：

1. 通过`robot_gazebo`中的`effortJointInterface`拿到电机相关的信息，通过`ImuSensorInterface`拿到imu相关的信息。将数据整理成`rl_msgs`中定义的格式发在`/rl/robot_state`上给`rl_sdk`
2. 通过从`/rl/command`上获得`action`，并用设定的pid参数计算电机应该输出的力矩，然后通过`jointHandles_[0].setCommand`的形式发送力矩到gazebo中对应的电机

