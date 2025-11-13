# Tracked Vehicle Project

履带式车辆仿真项目，包含 Gazebo 仿真环境和键盘控制功能。

## 环境要求

- ROS (推荐 Melodic 或 Noetic)
- Gazebo
- Python 2/3

## 启动仿真模型

### 启动 Gazebo 仿真

在终端中执行以下命令启动 `vehicle_all` 模型：

```bash
roslaunch vehicle_all gazebo.launch
```

#### 启动参数说明

`gazebo.launch` 支持以下参数：

- `world`: 世界文件名称（默认: `empty.world`）
- `paused`: 是否暂停仿真（默认: `false`）
- `use_sim_time`: 是否使用仿真时间（默认: `true`）
- `gui`: 是否显示 Gazebo GUI（默认: `true`）
- `headless`: 无头模式（默认: `false`）
- `debug`: 调试模式（默认: `false`）
- `x_pos`: 机器人初始 X 坐标（默认: `0`）
- `y_pos`: 机器人初始 Y 坐标（默认: `0`）
- `z_pos`: 机器人初始 Z 坐标（默认: `0.5`）

#### 示例：自定义启动参数

```bash
# 在指定位置生成机器人
roslaunch vehicle_all gazebo.launch x_pos:=2.0 y_pos:=1.0 z_pos:=0.5

# 无头模式启动（不显示 GUI）
roslaunch vehicle_all gazebo.launch gui:=false headless:=true
```

## 键盘控制

### 启动键盘控制节点

在启动 Gazebo 仿真后，打开**新的终端窗口**，执行以下命令启动键盘控制：

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

**注意**：确保新终端已 source ROS 环境：
```bash
source /opt/ros/<your_ros_distro>/setup.bash
source ~/catkin_ws/devel/setup.bash  # 如果使用 catkin 工作空间
```

### 键盘控制说明

#### 基本移动控制

```
   u    i    o
   j    k    l
   m    ,    .
```

- `i` - 前进
- `,` - 后退
- `j` - 左转
- `l` - 右转
- `u` - 前进并左转
- `o` - 前进并右转
- `m` - 后退并左转
- `.` - 后退并右转
- `k` - 停止

#### 全向移动模式（按住 Shift）

```
   U    I    O
   J    K    L
   M    <    >
```

按住 Shift 键可以实现侧向移动（全向移动模式）。

#### 垂直移动

- `t` - 上升 (+z)
- `b` - 下降 (-z)

#### 速度调节

- `q` / `z` - 增加/减少最大速度 10%
- `w` / `x` - 增加/减少线性速度 10%
- `e` / `c` - 增加/减少角速度 10%

#### 退出

- `CTRL-C` - 退出键盘控制节点

### 自定义键盘控制参数

可以自定义速度和角速度参数：

```bash
# 设置初始速度和角速度
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.9 _turn:=0.8

# 设置重复发布频率（Hz）
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _repeat_rate:=10.0

# 设置按键超时（秒），超时后自动停止
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _key_timeout:=0.6

# 发布到不同的话题
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=my_cmd_vel
```

## 完整使用流程

1. **启动 Gazebo 仿真**（终端 1）：
   ```bash
   roslaunch vehicle_all gazebo.launch
   ```

2. **启动键盘控制**（终端 2）：
   ```bash
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py
   ```

3. **控制机器人**：
   - 在键盘控制终端中，使用上述按键控制机器人移动
   - 确保键盘控制终端处于活动状态（点击终端窗口）

## 项目结构

```
src/
├── vehicle_all/              # 车辆模型包
│   ├── launch/
│   │   ├── gazebo.launch    # Gazebo 仿真启动文件
│   │   └── display.launch   # RViz 显示启动文件
│   ├── urdf/                # URDF 模型文件
│   └── meshes/              # 3D 模型文件
├── tracked_vehicle_demo/     # 履带车辆演示包
│   └── tracked_simulator/
│       └── teleop_twist_keyboard/  # 键盘控制节点
└── README.md                # 本文件
```

## 故障排除

### 问题：键盘控制无响应

- 确保键盘控制终端窗口处于活动状态（点击终端）
- 检查 `/cmd_vel` 话题是否正常发布：`rostopic echo /cmd_vel`
- 确认机器人模型已正确加载并订阅了 `/cmd_vel` 话题

### 问题：Gazebo 启动失败

- 检查 ROS 环境是否正确配置
- 确认所有依赖包已安装
- 查看终端错误信息

### 问题：机器人不移动

- 检查机器人模型是否正确订阅 `/cmd_vel` 话题
- 确认 Gazebo 仿真正在运行（不是暂停状态）
- 检查机器人关节控制器是否正常工作

## 许可证

请参考各子包的 LICENSE 文件。

