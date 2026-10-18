# XXU RM 26

XXU 全向机器人 ROS 2 Jazzy + Gazebo Harmonic 仿真、定位、导航与底盘控制工程。

ROS 工作空间位于 `xxu_nav_2026/`。仓库根目录用于项目说明和版本管理，工作空间内的
`src/` 存放 ROS 包，`build/`、`install/`、`log/` 等目录是构建或运行产物。

## 文档入口

本文保存项目用途和基本使用方式，专项说明见下方入口。历史实验数据及报告已清理；
构建产物、运行日志和用户原始 CAD 输入不随源码发布。

相关专项文档：

- [小陀螺导航与回归验证](xxu_nav_2026/tools/navigation/README.md)
- [LIO 调试与回归验证](xxu_nav_2026/tools/lio/README.md)
- [Small Point-LIO](xxu_nav_2026/src/small_point_lio/README.md)
- [点云转 LaserScan](xxu_nav_2026/src/pointcloud_to_laserscan/README.md)

## 快速开始

### 1. 构建工作空间

```bash
cd ~/xxu_2026/xxu_nav_2026
source /opt/ros/jazzy/setup.zsh
git -C .. submodule update --init --recursive
vcs import src < simulation.repos
colcon build --symlink-install
source install/setup.zsh
```

### 2. 启动单点导航仿真

采用 [rm_sim_26](https://github.com/Neomelt/rm_sim_26) 的哨兵模型和 RMUC 2025 场地，
接现有 LIO、AMCL、Nav2 与速度保护。外部源码固定版本见
[simulation.repos](xxu_nav_2026/simulation.repos)，使用说明见
[导航工具说明](xxu_nav_2026/tools/navigation/README.md)。先停止已有仿真，避免重复节点。

```bash
cd ~/xxu_2026/xxu_nav_2026
source install/setup.zsh
ros2 launch xxu_bringup rm_sim_navigation.launch.py \
  rviz:=true \
  gazebo_gui:=true
```

启动后等待自动初始位姿完成，再在 RViz 中使用 `Nav2 Goal` 发送一个目标点。
默认地图为 `src/xxu_bringup/maps/rmuc_2025.yaml`，由固定版本场地碰撞几何生成，
默认世界取自 `rm_sim_26/worlds/rmuc_2025_world.world`。使用 RGL Mid360，首次需执行
`tools/navigation/build_rgl.sh`；可显式选择 `lidar_mode:=gpu_lidar`。
雷达对地航向保持；出生点与地图初始位姿默认 `(3.4, 9.5, 0)`。

用户提供的 RMUL 场地入口为 `ros2 launch xxu_bringup rmul_navigation.launch.py`，
出生点 `(2,2,0)`，目前仍使用上游原车。EGA 折叠车模正在转换，当前 GLB 为展开草稿；
折叠模型需在 SolidWorks 中切换配合后导出整车 STEP，操作边界见[CAD 说明](xxu_nav_2026/tools/navigation/cad/README.md)。

### 3. 检查地图显示

RViz 中应满足：

- `Fixed Frame`：`map`
- `Map` Display 话题：`/map`
- `map_server` 已激活

地图初始化依赖 `/scan`、`/odom` 和 TF。新入口按出生点发送初始位姿；
旧入口的扫描匹配初始化方式保留。

## 启动入口

### 旧 XXU 仿真与 Nav2（兼容入口）

```bash
cd ~/xxu_2026/xxu_nav_2026
source install/setup.zsh
src/xxu_description/scripts/kill_simulation.sh
ros2 launch xxu_bringup simulation.launch.py \
  start_navigation:=true \
  enable_localization:=true \
  rviz:=true \
  gazebo_gui:=true
```

### 仅启动 Gazebo

```bash
ros2 launch xxu_bringup rm_sim_navigation.launch.py \
  start_navigation:=false \
  rviz:=false \
  gazebo_gui:=true
```

### Gazebo 已运行时单独启动 Nav2 和 RViz

```bash
ros2 launch xxu_bringup navigation.launch.py \
  map:=$(ros2 pkg prefix --share xxu_bringup)/maps/rmuc_2025.yaml \
  enable_localization:=true \
  rviz:=true
```

### 实车入口

实车入口不启动 Gazebo，接入真实雷达、云台、里程计和底盘驱动：

```bash
ros2 launch xxu_bringup real_robot.launch.py
```

实车默认使用真实时间。接口、单位、字段顺序和 TF 归属以
`xxu_nav_2026/src/xxu_bringup/config/robot_interfaces.yaml` 为准。

## 重要启动参数

下表保留旧 `simulation.launch.py` 的默认值。新 `rm_sim_navigation.launch.py`
默认启用导航、定位、Gazebo GUI 和 RViz，并使用新场地的地图、出生点及扫描高度。

基础导航参数在 `nav2_navigation.yaml` / `nav2_exploration.yaml`；fake 模式的覆盖参数统一在 [nav2_fake_profile.yaml](xxu_nav_2026/src/xxu_bringup/config/nav2_fake_profile.yaml)。两种来源的职责与共享速度链见[配置说明](xxu_nav_2026/tools/navigation/README.md#共享启动与参数来源)。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `start_navigation` | `false` | 是否启动 Nav2 |
| `enable_localization` | `false` | 定位和地图导航总开关；单点导航建议显式设为 `true` |
| `enable_gicp` | `false` | 是否启用 GICP 定位；默认使用 AMCL 初始化 |
| `map` | `maps/complex_map.yaml` | Nav2 使用的栅格地图 |
| `nav2_params_file` | `config/nav2_navigation.yaml` | Nav2 参数文件 |
| `rviz` | `false` | 是否启动 RViz |
| `gazebo_gui` | `true` | 是否启动 Gazebo GUI |
| `enable_lio` | `true` | 是否启动 Small Point-LIO |
| `use_fake_frame` | `true` | 是否使用 `gimbal_yaw_fake` 作为 Nav2 参考底盘帧 |
| `gyro_spin_rate` | `1.5` | 移动时底盘自旋角速度，单位 rad/s；设为 `0` 可关闭 |
| `nav_velocity_smoothing_window` | `0.3` | fake 模式导航速度平均窗口，单位秒；设为 `0` 可关闭，重启节点生效 |
| `auto_initial_pose` | `true` | 是否自动匹配并发布 AMCL 初始位姿 |
| `initial_pose_relocalize` | `true` | 是否使用 `/scan` 对地图进行初始重定位 |

定位和地图导航默认关闭是为了避免未确认定位质量时误启动。修改 launch 参数只对新启动的节点生效。

## 速度与底盘链路

导航命令从 Nav2 到底盘的链路为：

```text
/cmd_vel_nav
  -> /cmd_vel_smoothed
  -> /cmd_vel_collision
  -> /cmd_vel_transformed（使用虚拟参考帧时）
  -> /cmd_vel
  -> /wheel_velocity_controller/commands
```

其中：

- `fake_vel_transform` 负责虚拟导航参考帧、速度坐标变换和小陀螺角速度叠加。
- `cmd_vel_watchdog` 检查命令、里程计、激光、关节状态和关键 TF；任一输入超时就发布零速。
- `xxu_chassis_controller` 把 `TwistStamped` 转成四个全向轮的角速度指令。
- 底盘控制器默认命令超时为 `0.3 s`，最大轮速参数为 `100 rad/s`。

测试源码及实验候选已于 2026-10-17 清理；当前保留的导航链路见上文，历史实验数据及报告已按用户要求删除。

## 建图与探索

### 手动建图

终端 1，启动仿真但不启动 Nav2：

```bash
cd ~/xxu_2026/xxu_nav_2026
source install/setup.zsh
src/xxu_description/scripts/kill_simulation.sh
ros2 launch xxu_bringup simulation.launch.py \
  start_navigation:=false \
  rviz:=false \
  gazebo_gui:=true
```

终端 2，启动 SLAM Toolbox：

```bash
cd ~/xxu_2026/xxu_nav_2026
source install/setup.zsh
ros2 launch xxu_slam_toolbox mapping.launch.py
```

### 自主探索

```bash
ros2 launch xxu_slam_toolbox autonomous_mapping.launch.py
```

建图保存的地图通常位于：

```text
xxu_nav_2026/src/xxu_bringup/maps/auto_map.yaml
xxu_nav_2026/src/xxu_bringup/maps/auto_map.pgm
```

## 键盘控制

```bash
ros2 run key_controll wasd_keyboard_teleop.py
```

按键：`W/S` 前后移动，`A/D` 左右平移，`Q/E` 左右旋转，`G` 切换小陀螺模式，空格停止。

参数示例：

```bash
ros2 run key_controll wasd_keyboard_teleop.py --ros-args \
  -p linear_acceleration:=6.0 \
  -p gyro_angular_speed:=1.8
```

## 源码结构

```text
xxu_nav_2026/src/
├── xxu_bringup/                    # 主启动包、地图、Nav2 参数、RViz 配置
├── xxu_description/                # URDF/Xacro、Gazebo 世界和仿真辅助节点
├── xxu_chassis_controller/         # 全向底盘 Twist 到四轮角速度转换
├── xxu_livox_sim/                  # Livox Mid-360 激光雷达仿真
├── xxu_pointcloud_processing/      # 点云预处理
├── small_point_lio/                # Small Point-LIO 里程计
├── xxu_slam_toolbox/               # SLAM 和自主探索入口
├── pointcloud_to_laserscan/        # PointCloud2 转 LaserScan
├── fake_vel_transform/             # 虚拟速度和参考坐标变换
├── key_controll/                   # WASD 键盘遥控
├── pb_nav2_plugins/                # 自定义 Nav2 行为和 costmap 插件
├── pb_omni_pid_pursuit_controller/ # 全向 PID 路径追踪控制器
├── goal_approach_controller/       # 目标接近控制器
└── m-explore-ros2/                 # explore_lite / map_merge 子模块
```

## 自定义 Nav2 插件

`pb_nav2_plugins` 当前包含：

- `pb_nav2_behaviors::BackUpFreeSpace`：根据局部 costmap 选择较宽的自由空间执行备份。
- `pb_nav2_costmap_2d::IntensityVoxelLayer`：按点云强度范围标记体素障碍。

相关配置：

```text
xxu_nav_2026/src/xxu_bringup/config/nav2_navigation.yaml
xxu_nav_2026/src/xxu_bringup/config/nav2_exploration.yaml
```

## 常用排查

检查 ROS 包是否可见：

```bash
ros2 pkg prefix xxu_bringup
ros2 pkg prefix small_point_lio
ros2 pkg prefix xxu_chassis_controller
```

检查关键节点和话题：

```bash
ros2 node list
ros2 topic list | rg '/map|/scan|/odom|cmd_vel|wheel_velocity'
ros2 topic info /map -v
ros2 run tf2_tools view_frames
```

如果 Gazebo 或 RViz 窗口消失，先确认启动终端没有被 `Ctrl-C` 终止；重新启动前运行：

```bash
src/xxu_description/scripts/kill_simulation.sh
```

不要把清理脚本和包含 `ros2 launch` 的命令写在同一条 shell 命令中，以免清理脚本匹配并终止正在启动的命令。
