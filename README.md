# rl_deploy
> Use ros_control to realize sim2sim in gazebo and sim2real

>Many thanks to DirectDriveTech-DDT[https://github.com/DirectDriveTech-DDT]  for the guidance and hardware provided for this project

### Frame

- `rl_sdk` mainly refers to https://github.com/fan-ziqi/rl_sar and uses `libtorch` to complete the loading of .pt files.

- `rl_msgs` ：For interaction between rl_sdk and rl_controller
-  `wheeled_bipedal_rl_controller`：use ros_control to interact with gazebo



#### Install

- ros_control

```python
sudo apt install ros-noetic-teleop-twist-keyboard ros-noetic-controller-interface  ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller
sudo apt-get install ros-noetic-imu*
```

- libtorch

```python
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip -d ./
echo 'export Torch_DIR=/path/to/your/torchlib' >> ~/.bashrc
```

- yaml-cpp

```python
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp && mkdir build && cd build
cmake -DYAML_BUILD_SHARED_LIBS=on .. && make
sudo make install
sudo ldconfig
```



#### Run

```
mon launch diablo load_gazebo.launch
mon launch diablo load_controllers.launch
mon launch diablo load_rl_interface.launch
```

#### Reference code
> Thank very much for these excellent open source projects！

- Very good sim2sim code reference：
```
https://github.com/fan-ziqi/rl_sar
```
- Top ros_control use cases
```aiignore
https://github.com/qiayuanl/legged_control
https://github.com/rm-controls/rm_control
```