# CLAUDE.md

XXU 机器人 ROS 2 + Gazebo Harmonic 仿真与导航项目。

## 构建与环境

```bash
cd ~/xxu_2026/xxu_nav_2026
colcon build --symlink-install
source install/setup.zsh
```

## 启动前清理残留进程

每次启动前必须先 kill 所有相关进程，否则残留进程会污染环境，导致状态异常。

```bash
src/xxu_description/scripts/kill_simulation.sh
```

## 启动 RViz（模型可视化）

```bash
ros2 launch xxu_description display.launch.py
```

使用 `src/xxu_description/urdf/xxu.urdf.xacro` 和 `src/xxu_description/rviz/display.rviz`。

## 启动 Gazebo 仿真 + Nav2 导航（推荐入口）

```bash
ros2 launch xxu_bringup simulation.launch.py
```

可传参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `start_navigation` | `true` | 是否同时启动 Nav2 |
| `map` | `maps/empty.yaml` | 地图文件 |
| `nav2_params_file` | `config/nav2_navigation.yaml` | Nav2 参数 |
| `rviz` | `true` | 是否启动 RViz |
| `enable_lio` | `false` | 启用 Small Point-LIO |
| `enable_cmd_vel_odom` | `false` | 启用 cmd_vel 里程计 |
| `use_livox_native` | `false` | 使用 Livox 原生仿真 |
| `use_fake_frame` | `true` | 使用底盘上的 gimbal_yaw_fake 替代 base_link |
| `auto_initial_pose` | `false` | 自动设置初始位姿 |
| `initial_pose_x/y/yaw` | `0.0/0.0/0.0` | 初始位姿 |

示例：

```bash
# 仅仿真，不启动导航
ros2 launch xxu_bringup simulation.launch.py start_navigation:=false

# 启用 LIO
ros2 launch xxu_bringup simulation.launch.py enable_lio:=true

# 单点导航仿真
ros2 launch xxu_bringup single_point_simulation.launch.py
```

启动后会自动：
- 打开 Gazebo Harmonic 仿真窗口，机器人生成于 x=1.75, y=0.0, z=0.05, yaw=180°
- ros_gz_bridge 桥接: `/cmd_vel`, `/joint_states_gz`, `/imu`, `/scan`, `/clock`
- 启动速度看门狗（`/cmd_vel_keyboard` → `cmd_vel_watchdog` → `/cmd_vel`，超时 0.3s 自动回零）
- 如果 `start_navigation:=true`，启动完整的 Nav2 导航栈（AMCL、控制器、规划器、行为树等）

### 单独启动导航（Gazebo 仿真已在运行）

```bash
ros2 launch xxu_bringup navigation.launch.py
```

## 键盘控制

```bash
ros2 run key_controll wasd_keyboard_teleop.py
```

按键: `W/S` 前后移动，`A/D` 左右平移，`Q/E` 左右旋转；小陀螺模式下 `Q/E` 切换旋转方向，`G` 切换小陀螺模式（边旋转边行进），空格停止，`Ctrl-C` 退出。

可调参数: `linear_speed`、`angular_speed`、`linear_acceleration`、`angular_acceleration`、`gyro_angular_speed`、`key_timeout`。例如：

```bash
ros2 run key_controll wasd_keyboard_teleop.py --ros-args -p linear_acceleration:=6.0 -p gyro_angular_speed:=1.8
```

如果启动时报 `This member is not been selected`，说明当前 FastDDS/FastCDR 运行库在创建 ROS 节点前失败。安装并切换 CycloneDDS：

```bash
sudo apt-get install ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run key_controll wasd_keyboard_teleop.py
```

## SLAM 建图

```bash
# 手动建图
ros2 launch xxu_slam_toolbox mapping.launch.py

# 自主探索建图
ros2 launch xxu_slam_toolbox autonomous_mapping.launch.py
```

## 项目结构

```
xxu_nav_2026/
└── src/
    ├── xxu_bringup/                 # 主启动包（仿真 + 导航入口）
    │   ├── launch/
    │   │   ├── simulation.launch.py                # Gazebo + Nav2 一体化启动
    │   │   ├── single_point_simulation.launch.py    # 单点导航仿真
    │   │   └── navigation.launch.py                # 独立 Nav2 启动
    │   ├── config/
    │   │   ├── nav2_navigation.yaml                # Nav2 导航参数
    │   │   └── robot_motion.yaml                   # 机器人运动学参数
    │   ├── maps/                                   # 地图文件 (pgm + yaml)
    │   ├── waypoints/                              # 航点 CSV
    │   ├── rviz/navigation.rviz                     # 导航 RViz 配置
    │   └── scripts/
    │
    ├── xxu_description/             # 机器人模型描述
    │   ├── launch/
    │   │   ├── display.launch.py                   # RViz 模型可视化
    │   │   └── gazebo.launch.py                    # Gazebo 仿真（底层）
    │   ├── urdf/
    │   │   ├── xxu.urdf.xacro                      # 纯显示用 Xacro
    │   │   ├── xxu_gazebo.urdf.xacro               # Gazebo 仿真用 Xacro（含传感器插件、ros2_control）
    │   │   ├── wheel.urdf.xacro                    # 轮子宏
    │   │   └── plugins/                            # Gazebo 插件 xacro
    │   ├── meshes/                                 # 网格文件
    │   ├── rviz/display.rviz                       # 显示用 RViz 配置
    │   ├── worlds/empty_with_sensors.sdf           # Gazebo 世界
    │   └── scripts/
    │       ├── kill_simulation.sh                   # 清理残留进程
    │       ├── cmd_vel_watchdog.py                  # 速度看门狗
    │       ├── cmd_vel_odometry.py                  # cmd_vel 里程计发布
    │       ├── imu_frame_republisher.py             # IMU 帧重发布
    │       ├── pointcloud_frame_republisher.py      # 点云帧重发布
    │       └── scan_frame_republisher.py            # 激光帧重发布
    │
    ├── key_controll/                # 键盘遥控
    │   └── scripts/wasd_keyboard_teleop.py
    │
    ├── xxu_chassis_controller/      # 底盘控制器（ros2_control 全向轮）
    ├── xxu_livox_sim/               # Livox Mid-360 激光雷达仿真
    ├── xxu_pointcloud_processing/   # 点云预处理（裁剪、降采样等）
    ├── small_point_lio/             # Small Point-LIO 里程计
    ├── xxu_slam_toolbox/            # SLAM 建图（online async + 自主探索）
    ├── pointcloud_to_laserscan/     # 点云转激光扫描
    ├── cpp_lidar_filter/            # 激光雷达 C++ 滤波器
    ├── fake_vel_transform/          # 虚拟速度 TF 变换
    ├── pb_nav2_plugins/             # 自定义 Nav2 插件
    ├── pb_omni_pid_pursuit_controller/  # 全向 PID 追踪控制器
    ├── goal_approach_controller/    # 目标接近控制器
    └── m-explore-ros2/              # 自主探索（submodule）
```
