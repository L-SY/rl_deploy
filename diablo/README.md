## Diablo

> 因为不同的仿真环境对于并联结构的支持程度不同，导致diablo的urdf经常需要调整，所以利用.xacro格式来方便修改。

在`urdf`中的diablo.xacro下有如下参数
![diablo_urdf.png](images%2Fdiablo_urdf.png)

- `fixed_leg`：true时锁定腿部关节（例如当差速底盘时）
- `fixed_wheel`： true时锁定轮子
- `use_gazebo`：true时会加载gazebo相关的translation
- `close_chain`：true时会完整腿部结构（目前只能在gazebo中用）


![diablo_rviz.png](images%2Fdiablo_rviz.png)
##### link命名

一级腿命名为{direction}_thing_link (大腿)

二级腿命名为{direction}_shank_link  (小腿)

另外一侧在{direction}后增加fake

> 注意urdf中取较长的两根杆为主运动连杆



##### joint命名

一级joint为{direction}_hip_joint (髋关节)

二级joint为{direction}_knee_joint (膝关节)

另外一侧在{direction}后增加fake

##### 使用方法

> 在ubuntu20.04下ros noetic版本

- 生成urdf

```
rosrun xacro xacro -o diablo.xacro diablo.urdf
```

- Rviz中显示

```
mon launch diablo load_rviz.launch
```

- Gazebo中显示

```
mon launch diablo load_gazebo.launch
```

