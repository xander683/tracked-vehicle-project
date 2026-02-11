# Tracked Vehicle Project

履带式车辆仿真项目，包含 Gazebo 仿真环境和键盘控制功能。

## 环境要求

- ROS (推荐 Melodic 或 Noetic)
- Gazebo
- Python 2/3

## 启动仿真模型

### 启动单个车辆仿真

在终端中执行以下命令启动单个 `vehicle_all` 模型：

```bash
roslaunch vehicle_all gazebo.launch
```

### 启动三个车辆仿真

在终端中执行以下命令同时启动三个履带车：

```bash
roslaunch vehicle_all gazebo_three_vehicles.launch
```

三个车辆将分别命名为 `vehicle_1`、`vehicle_2` 和 `vehicle_3`，初始位置分别为：
- `vehicle_1`: (0, 0, 0.5)
- `vehicle_2`: (2.0, 0, 0.5)
- `vehicle_3`: (-2.0, 0, 0.5)

每个车辆都有独立的命名空间，可以通过以下话题控制：
- `/vehicle_1/cmd_vel` - 控制车辆1
- `/vehicle_2/cmd_vel` - 控制车辆2
- `/vehicle_3/cmd_vel` - 控制车辆3

### 启动三个车辆仿真（带跟随功能）

在终端中执行以下命令启动三个履带车并启用跟随功能：

```bash
roslaunch vehicle_all gazebo_three_vehicles_with_following.launch
```

**跟随功能说明**：
- **轨迹复刻跟随**：后车沿着前车走过的轨迹移动，而不是同步转向
- 车辆2会自动跟随车辆1走过的轨迹，保持1.5米的距离
- 车辆3会自动跟随车辆2走过的轨迹，保持1.5米的距离
- 例如：当车辆1转弯时，车辆2会在1.5米后经过车辆1转弯的位置，然后才开始转弯
- 就像火车车厢一样，每节车厢都沿着前一节车厢的轨迹移动
- 您只需要控制车辆1，车辆2和车辆3会自动跟随

**启动参数**：
- `desired_distance`: 期望保持的距离（默认: 1.5米）
- `enable_following`: 是否启用跟随功能（默认: true）

**示例**：
```bash
# 自定义跟随距离为0.5米
roslaunch vehicle_all gazebo_three_vehicles_with_following.launch desired_distance:=0.5

# 启动三个车辆但不启用跟随功能
roslaunch vehicle_all gazebo_three_vehicles_with_following.launch enable_following:=false
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
# 控制车辆1
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/vehicle_1/cmd_vel
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

# 控制三个车辆中的特定车辆（例如车辆1）
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/vehicle_1/cmd_vel

# 控制车辆2
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/vehicle_2/cmd_vel

# 控制车辆3
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/vehicle_3/cmd_vel
```

## 完整使用流程

### 单个车辆控制

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

### 三个车辆控制

#### 方式1：手动控制所有车辆

1. **启动 Gazebo 仿真（三个车辆）**（终端 1）：
   ```bash
   roslaunch vehicle_all gazebo_three_vehicles.launch
   ```

2. **启动键盘控制（控制车辆1）**（终端 2）：
   ```bash
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/vehicle_1/cmd_vel
   ```

3. **启动键盘控制（控制车辆2）**（终端 3，可选）：
   ```bash
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/vehicle_2/cmd_vel
   ```

4. **启动键盘控制（控制车辆3）**（终端 4，可选）：
   ```bash
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/vehicle_3/cmd_vel
   ```

#### 方式2：使用跟随功能（推荐）

1. **启动 Gazebo 仿真（三个车辆 + 跟随功能）**（终端 1）：
   ```bash
   roslaunch vehicle_all gazebo_three_vehicles_with_following.launch
   ```

2. **启动键盘控制（只控制车辆1）**（终端 2）：
   ```bash
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/vehicle_1/cmd_vel
   ```

3. **观察跟随效果**：
   - 车辆1会响应您的键盘控制
   - 车辆2会沿着车辆1走过的轨迹移动，保持1.5米距离
   - 车辆3会沿着车辆2走过的轨迹移动，保持1.5米距离
   - 轨迹复刻效果：当车辆1转弯时，车辆2会在到达相同位置时才转弯
   - 就像真正的火车车厢，每节车厢都精确复刻前一节的运动轨迹

**注意**：使用跟随功能时，您只需要控制车辆1，车辆2和车辆3会自动跟随。如果需要单独控制某个车辆，可以禁用跟随功能或直接发布命令到相应的话题。

## S型（正弦波）轨迹跟随实验

### 快速开始

运行S型轨迹跟随实验，让第一辆车沿正弦波路径行驶，后续车辆自动跟随：

#### 方法1：一键启动（推荐）

在工作空间根目录下运行：

```bash
# 使用默认参数启动实验
./src/vehicle_all/scripts/run_sine_wave_experiment.sh

# 或自定义参数
./src/vehicle_all/scripts/run_sine_wave_experiment.sh --speed 0.6 --amplitude 1.0 --frequency 0.25
```

#### 方法2：手动启动

```bash
# 终端1：启动Gazebo仿真
roslaunch vehicle_all gazebo_three_vehicles_with_following.launch

# 终端2：启动数据记录
rosrun vehicle_all record_sine_wave_data.py

# 终端3：启动正弦波控制器
rosrun vehicle_all sine_wave_controller.py
```

### 参数配置

**正弦波轨迹参数：**
- `forward_speed`: 前进速度（默认: 0.5 m/s）
- `amplitude`: 振幅（默认: 0.8 m）
- `frequency`: 频率（默认: 0.3 Hz）
- `duration`: 持续时间（默认: 60 s）

**示例：**
```bash
# 大振幅慢频率
rosrun vehicle_all sine_wave_controller.py \
  _forward_speed:=0.4 _amplitude:=1.2 _frequency:=0.2 _duration:=90

# 快速小振幅
rosrun vehicle_all sine_wave_controller.py \
  _forward_speed:=0.8 _amplitude:=0.5 _frequency:=0.5 _duration:=30
```

### 数据可视化

实验完成后，可视化轨迹数据：

```bash
# 专用正弦波可视化工具
rosrun vehicle_all visualize_sine_wave_data.py /tmp/vehicle_data/sine_wave_experiment_*.csv

# 或使用通用可视化工具
rosrun vehicle_all visualize_vehicles_data.py /tmp/vehicle_data/sine_wave_experiment_*.csv
```

可视化内容包括：轨迹图、横向位移、速度曲线、角速度、跟随误差等。

### 详细文档

查看完整的S型轨迹实验指南：

```bash
cat src/vehicle_all/SINE_WAVE_GUIDE.md
```

或在线阅读: [SINE_WAVE_GUIDE.md](vehicle_all/SINE_WAVE_GUIDE.md)

## 项目结构

```
src/
├── vehicle_all/              # 车辆模型包
│   ├── launch/
│   │   ├── gazebo.launch                           # Gazebo 仿真启动文件（单个车辆）
│   │   ├── gazebo_three_vehicles.launch            # Gazebo 仿真启动文件（三个车辆）
│   │   ├── gazebo_three_vehicles_with_following.launch  # Gazebo 仿真启动文件（三个车辆+跟随功能）
│   │   ├── vehicle_following.launch                # 车辆跟随节点启动文件
│   │   └── display.launch                          # RViz 显示启动文件
│   ├── scripts/
│   │   ├── vehicle_follower.py                     # 车辆跟随节点（Python）
│   │   └── test_vehicle_control.sh                 # 车辆控制测试脚本
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

