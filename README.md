# rl_deploy
> Use ros_control to realize sim2sim and sim2real.

### Frame

- Among them, `rl_sdk` mainly refers to https://github.com/fan-ziqi/rl_sar and uses `libtorch` to complete the loading of .pt files.

- `rl_msgs` ：For interaction between rl_sdk and rl_controller
-  `wheeled_bipedal_rl_controller`：use ros_control to interact with gazebo



#### Install

- code
```python
cd ~
mkdir rl_ws
cd rl_ws 
git clone git@github.com:L-SY/rl_deploy.git -b whole_ws  ~rl_ws/src
cd src
catkin build
```
- ros_control

```python
sudo apt install ros-noetic-teleop-twist-keyboard ros-noetic-controller-interface  ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller
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