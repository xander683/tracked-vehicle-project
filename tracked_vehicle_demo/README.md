# Gazebo 履带车仿真器

用于仿真履带车的简单模型和世界文件。

## 1. 履带车模型

Xacro 文件从 [gazebo tracked-vehicle demo](https://github.com/osrf/gazebo/blob/gazebo11/worlds/tracked_vehicle_wheeled.world) 转换而来。

使用 `roslaunch tracked_description empty_world.launch` 在空世界中启动机器人。

你可以使用键盘 `WADS` 控制机器人。

![empty_world](./image/empty_world.png)



### 如何将 sdf 转换为 xacro

1. 删除 **sdf** 文件中的所有 `plugin` 标签
2. 使用 [pysdf](https://github.com/andreasBihlmaier/pysdf) 将 **sdf** 中的 joint 和 link 标签转换为 **urdf** 格式
3. 使用 gazebo 标签将 `plugin` 添加到 **urdf** 中
4. 手动将 **urdf** 转换为 **xacro**，以便于调整模型


## 2. 自动生成地形

该地形世界文件由 [terrain_generator](https://github.com/Sarath18/terrain_generator) 生成。

使用脚本 `setup.bash` 将 `autogen_terrain` 世界模型移动到 `~/.gazebo/models`。

使用 `roslaunch tracked_description mountain_world.launch` 在地形世界中启动机器人。

你可以使用键盘 `WADS` 控制机器人。

![mounatin_world](./image/mounatin_world.png)

## 3. tracked_gazebo_plugins

传输 twist 命令：ros->gazebo。

基于 [gazebo_plugins/gazebo_ros_diff_drive.cpp](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_diff_drive.cpp)

修改 [WheelTrackedVehiclePlugin](https://github.com/osrf/gazebo/blob/gazebo9_9.13.1/plugins/WheelTrackedVehiclePlugin.cc) 以解决打滑问题。

## 4. 真值 & 立体相机 & IMU

真值发布器在 `gt_publisher_node.cpp` 中实现。

修改 `libgazebo_ros_p3d` 以消除坐标偏差。

![simulation_gif](./image/simulation.gif)

## 5. rtabmap/vins fusion 支持

`roslaunch rtabmap_ros euroc_datasets.launch args:="Odom/Strategy 9 OdomVINS/ConfigPath ~/catkin_vins/src/VINS-Fusion/config/euroc/euroc_stereo_config.yaml" MH_seq:=true raw_images_for_odom:=true`

