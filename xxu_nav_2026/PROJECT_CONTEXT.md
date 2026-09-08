# XXU 项目快速熟悉指南

本文档用于帮助新窗口快速理解项目。开始处理代码前，建议先阅读本文，再根据具体问题查看相关模块。

## 1. 项目概览

这是一个基于 ROS 2 Jazzy 和 Gazebo Harmonic 的 XXU 全向机器人项目，主要包含：

- Gazebo 仿真与机器人模型
- 全向底盘控制
- Livox Mid-360 激光雷达仿真
- Small Point-LIO 里程计
- SLAM Toolbox 建图与自主探索
- AMCL 定位和 Nav2 导航
- 自定义 Nav2 控制器、行为和代价地图插件
- AI 控制器参数调节工具

项目工作空间：

```text
/home/naiwu/xxu_2026/xxu_nav_2026
```

## 2. 核心数据链路

```text
Gazebo 世界
  └─ 机器人模型、底盘、IMU、激光雷达
       ↓
ros_gz_bridge / 传感器处理
       ↓
Small Point-LIO
       ↓
/odom + odom → base_link（或 gimbal_yaw_fake）
       ↓
map_server + AMCL
       ↓
map → odom
       ↓
Nav2：规划、控制、行为树、避障
       ↓
/cmd_vel_nav → 速度平滑/碰撞监测 → 底盘控制器
```

当前导航通常使用 `gimbal_yaw_fake` 作为 Nav2 的机器人基座坐标系，由 `use_fake_frame:=true` 控制。

## 3. 重要目录

| 目录 | 作用 |
|---|---|
| `src/xxu_bringup` | 总启动入口、Nav2 参数、地图、RViz 配置、启动辅助脚本 |
| `src/xxu_description` | URDF/Xacro、Gazebo 世界、机器人传感器和 ros2_control |
| `src/xxu_chassis_controller` | 全向底盘控制器 |
| `src/small_point_lio` | 激光惯导里程计，提供导航所需的 odom/TF |
| `src/xxu_livox_sim` | Livox Mid-360 Gazebo 仿真插件 |
| `src/xxu_pointcloud_processing` | 点云裁剪、降采样等预处理 |
| `src/pointcloud_to_laserscan` | PointCloud2 与 LaserScan 转换 |
| `src/xxu_slam_toolbox` | 手动建图和自主探索建图 |
| `src/pb_nav2_plugins` | 自定义 Nav2 行为和 costmap 插件 |
| `src/pb_omni_mppi_controller` | 全向 MPPI 控制器 |
| `src/pb_omni_pid_pursuit_controller` | 全向 PID 路径追踪控制器 |
| `src/goal_approach_controller` | 目标接近控制器 |
| `src/ai_controller_tuner` | 控制器参数自动调节工具 |
| `src/m-explore-ros2` | 自主探索相关子模块，属于 submodule |

## 4. 常用命令

进入工作空间并加载环境：

```bash
cd /home/naiwu/xxu_2026/xxu_nav_2026
source install/setup.zsh
```

重新构建：

```bash
colcon build --symlink-install
source install/setup.zsh
```

启动前清理残留进程：

```bash
src/xxu_description/scripts/kill_simulation.sh
```

启动完整仿真、导航和 RViz：

```bash
ros2 launch xxu_bringup simulation.launch.py \
  start_navigation:=true rviz:=true
```

只启动 Gazebo 仿真：

```bash
ros2 launch xxu_bringup simulation.launch.py \
  start_navigation:=false rviz:=false
```

Gazebo 已经运行时单独启动 Nav2：

```bash
ros2 launch xxu_bringup navigation.launch.py rviz:=true
```

键盘控制：

```bash
ros2 run key_controll wasd_keyboard_teleop.py
```

按键：`W/S` 前后，`A/D` 横移，`Q/E` 旋转，`G` 小陀螺模式，空格停止。

## 5. 建图流程

手动建图通常分两个终端：

```bash
# 终端 1
ros2 launch xxu_bringup simulation.launch.py start_navigation:=false

# 终端 2
ros2 launch xxu_slam_toolbox mapping.launch.py
```

自主探索建图：

```bash
# 终端 1：只启动仿真
ros2 launch xxu_bringup simulation.launch.py start_navigation:=false

# 终端 2：启动 SLAM、Nav2 和 explore_lite
ros2 launch xxu_slam_toolbox autonomous_mapping.launch.py
```

默认地图保存位置：

```text
src/xxu_bringup/maps/auto_map.yaml
src/xxu_bringup/maps/auto_map.pgm
```

地图必须成对使用：`.yaml` 中的 `image` 字段应指向对应的 `.pgm` 文件。

## 6. 启动和排查顺序

遇到导航不动、节点未激活或 TF 异常时，按以下顺序检查：

1. 是否清理了旧的 Gazebo、Nav2 和 bridge 进程。
2. `/clock` 是否持续发布，所有相关节点是否使用 `use_sim_time:=true`。
3. Gazebo 是否生成机器人，`/joint_states`、`/scan`、`/imu` 是否存在。
4. `/odom` 是否持续发布，`odom → base_link` 或 `odom → gimbal_yaw_fake` 是否存在。
5. `map_server` 和 `amcl` 是否已激活。
6. 是否存在稳定的 `map → odom` TF。
7. Nav2 的 controller、planner、BT navigator 等节点是否已激活。
8. `/cmd_vel_nav`、`/cmd_vel_smoothed` 和底盘最终输入是否有数据。
9. 地图、机器人初始位置和目标点是否在可通行区域内。

常用检查命令：

```bash
ros2 topic hz /clock
ros2 topic hz /odom
ros2 topic list
ros2 node list
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 run tf2_tools view_frames
ros2 topic echo /cmd_vel_smoothed
ros2 service list | grep get_costmap
```

## 7. 自定义导航插件

`pb_nav2_plugins` 当前包含：

- `pb_nav2_behaviors::BackUpFreeSpace`：根据局部 costmap 选择较宽的自由空间执行后退。
- `pb_nav2_costmap_2d::IntensityVoxelLayer`：根据点云 intensity 范围筛选障碍物。

控制器相关参数主要位于：

```text
src/xxu_bringup/config/nav2_navigation.yaml
src/xxu_bringup/config/nav2_exploration.yaml
```

修改插件或控制器 C++ 代码后，需要重新构建对应包并重新加载环境：

```bash
colcon build --symlink-install --packages-select \
  pb_nav2_plugins pb_omni_mppi_controller \
  pb_omni_pid_pursuit_controller
source install/setup.zsh
```

## 8. 当前开发状态和注意事项

- 当前分支：`feature/omni-mppi`。
- 当前 HEAD 提交为“完善 Nav2 调参链路与全向控制器”。
- 工作区可能包含大量未提交的用户修改；处理任务前必须先查看 `git status`，不要覆盖或回退这些修改。
- `.cache/clangd` 等索引文件可能显示为修改，通常不属于功能代码。
- `README.md`、`CLAUDE.md` 与实际代码的默认参数可能存在差异；以当前 `src/xxu_bringup/launch/*.launch.py` 的实现为准。
- `simulation.launch.py` 当前代码默认：`start_navigation=false`、`rviz=false`、`enable_lio=true`，地图为 `complex_map.yaml`。
- 多次启动仿真前务必清理残留进程，否则可能引发 DDS participant、Gazebo、TF 或 Nav2 状态异常。
- API key 不应写入仓库配置文件，应通过运行时环境变量提供。

## 9. 推荐阅读顺序

1. 本文档
2. `README.md`
3. `src/xxu_bringup/launch/simulation.launch.py`
4. `src/xxu_description/launch/gazebo.launch.py`
5. `src/xxu_bringup/launch/navigation.launch.py`
6. `src/xxu_bringup/config/nav2_navigation.yaml`
7. 与当前问题直接相关的控制器、LIO 或传感器包
8. `../PROJECT_LOG.md`

