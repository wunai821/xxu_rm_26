# xxu_nav_2026

XXU 机器人 ROS 2 + Gazebo Harmonic 仿真与导航项目，包含机器人模型、仿真启动、Nav2 导航、SLAM 建图、点云处理、底盘控制器和自定义 Nav2 插件。

## 构建

```bash
cd ~/xxu_2026/xxu_nav_2026
colcon build --symlink-install
source install/setup.zsh
```

## 启动前清理

每次启动仿真前建议先清理残留进程，避免 Gazebo、Nav2 或桥接节点残留导致状态异常：

```bash
src/xxu_description/scripts/kill_simulation.sh
```

## 启动仿真与导航

完整启动命令行：

```bash
cd ~/xxu_2026/xxu_nav_2026
source install/setup.zsh
src/xxu_description/scripts/kill_simulation.sh
ros2 launch xxu_bringup simulation.launch.py start_navigation:=true rviz:=true
```

推荐入口：

```bash
ros2 launch xxu_bringup simulation.launch.py start_navigation:=true rviz:=true
```

常用参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `start_navigation` | `false` | 是否同时启动 Nav2 |
| `map` | `maps/complex_map.yaml` | 与 `complex_mapping.sdf` 对应的地图文件 |
| `nav2_params_file` | `config/nav2_navigation.yaml` | Nav2 参数文件 |
| `rviz` | `false` | 是否启动 RViz |
| `enable_lio` | `true` | 是否启用 Small Point-LIO |
| `enable_cmd_vel_odom` | `false` | 是否启用 cmd_vel 里程计 |
| `use_livox_native` | `true` | 是否使用带逐点时间戳的分批射线 MID-360 仿真 |
| `use_fake_frame` | `true` | 是否使用底盘上的 `gimbal_yaw_fake` 稳定速度参考系 |
| `gyro_spin_rate` | `31.416（5转/s）` | 移动过程中的小陀螺角速度，设为 `0` 关闭 |
| `enable_stamped_cmd_vel` | `true` | 使用 Jazzy 的 `TwistStamped` 命令链路 |
| `auto_initial_pose` | `true` | 导航启动时用 `/scan` 对栅格地图粗匹配并初始化 AMCL |
| `initial_pose_x/y/yaw` | `0.02/0.03/0.0` | 自动匹配失败时使用的后备初始位姿 |
| `initial_pose_covariance_xy/yaw` | `1.0/0.2741557` | 发布给 AMCL 的初始位姿 XY/Yaw 方差（m²/rad²） |

### MID-360 扫描与噪声模型

`use_livox_native=true` 时，`xxu_livox_sim` 的 Gazebo System 插件读取
`src/xxu_livox_sim/config/mid360.csv` 中的官方 Livox 扫描方向序列，按连续轨迹
分帧射线检测，并为每个命中点写入 0.1 s 扫描周期内的时间戳。当前仓库资源是从
官方 800,000 条记录均匀抽样的 2,400 条版本，插件也支持直接替换为完整 CSV。
默认测距噪声为零均值高斯噪声，标准差 `0.02 m`，可在
`gazebo_mid360_livox_system_plugin` 中通过 `range_noise_stddev` 和 `noise_seed`
调整。

官方轨迹来源：[Livox livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation)。

## 常用启动命令

所有命令默认先进入工作空间并加载环境：

```bash
cd ~/xxu_2026/xxu_nav_2026
source install/setup.zsh
```

## 实车启动与接口合同

实车入口不启动 Gazebo，直接接入真实 Livox、云台关节状态和底盘轮速适配器：

```bash
ros2 launch xxu_bringup real_robot.launch.py
```

实车默认使用真实时间（`use_sim_time=false`）。需要回放或仿真时只切换入口参数，
该参数会透传给 URDF TF、点云/IMU 补偿、LIO、AMCL、Nav2、碰撞监测和最终安全门控：

```bash
ros2 launch xxu_bringup real_robot.launch.py use_sim_time:=true
```

底盘、雷达、云台的消息类型、单位、字段顺序和 TF 归属以
`src/xxu_bringup/config/robot_interfaces.yaml` 为准。速度链路固定为：

```text
/cmd_vel_nav -> /cmd_vel_smoothed -> /cmd_vel_collision
  -> /cmd_vel_transformed（可选）-> /cmd_vel -> Float64MultiArray[4]
```

`/cmd_vel` 之前的速度消息统一为 `geometry_msgs/msg/TwistStamped`；最终 watchdog
同时检查速度、/odom、/scan、/joint_states 和关键 TF，任一输入断流就以 20 Hz 发布零
速度。底盘控制器自身还有 0.3 s 超时，因此上游节点退出时也会停止轮速输出。

## 接口和停车验证

无需 Gazebo 或真实硬件即可运行确定性接口测试。默认测试在 3 s 后插入 0.15 m
前向障碍，并由模拟底盘断言障碍出现后轮速为零：

```bash
ros2 launch xxu_bringup interface_validation.launch.py
```

还可以把输入流停止来验证 fail-safe（以下示例不启动碰撞监测，直接测试最终门控）：

```bash
ros2 launch xxu_bringup interface_validation.launch.py \
  enable_collision_monitor:=false obstacle_after:=-1 drop_odom_after:=3.0
ros2 launch xxu_bringup interface_validation.launch.py \
  enable_collision_monitor:=false obstacle_after:=-1 drop_tf_after:=3.0
ros2 launch xxu_bringup interface_validation.launch.py \
  enable_collision_monitor:=false obstacle_after:=-1 stop_test_command_after:=3.0
```

测试节点输出 `PASS` 表示在故障前观察到非零轮速、故障后只观察到零轮速；输出
`FAIL` 或进程异常退出都应视为接口/安全链路回归。

完整仿真 + Nav2 + RViz：

```bash
src/xxu_description/scripts/kill_simulation.sh
ros2 launch xxu_bringup simulation.launch.py start_navigation:=true rviz:=true
```

仅启动 Gazebo 仿真：

```bash
src/xxu_description/scripts/kill_simulation.sh
ros2 launch xxu_bringup simulation.launch.py start_navigation:=false rviz:=false
```

Gazebo 已经运行时，单独启动 Nav2 和 RViz：

```bash
ros2 launch xxu_bringup navigation.launch.py rviz:=true
```

单点导航 / 单点巡航仿真：

```bash
src/xxu_description/scripts/kill_simulation.sh
ros2 launch xxu_bringup single_point_simulation.launch.py rviz:=true
```

默认会加载与 `complex_mapping.sdf` 对应的 `src/xxu_bringup/maps/complex_map.yaml`，启动复杂地图世界、LIO、Nav2 和 RViz。启动后可在 RViz 里发送 `Nav2 Goal`。

启动依赖顺序由 launch 自动保证：Gazebo 与 `/clock` -> 机器人、传感器与 LIO 里程计 -> `map_server` 和 AMCL -> 自动初始位姿 -> `map -> odom` -> Nav2 控制、规划、行为树主链。各阶段按服务、订阅和 TF 就绪状态推进，初始位姿和导航激活不再依赖固定等待时长。

手动建图：

```bash
# 终端 1：启动仿真，不启动 Nav2
cd ~/xxu_2026/xxu_nav_2026
source install/setup.zsh
src/xxu_description/scripts/kill_simulation.sh
ros2 launch xxu_bringup simulation.launch.py start_navigation:=false rviz:=false

# 终端 2：启动 SLAM Toolbox 建图
cd ~/xxu_2026/xxu_nav_2026
source install/setup.zsh
ros2 launch xxu_slam_toolbox mapping.launch.py
```

建图退出时默认自动保存到：

```text
src/xxu_bringup/maps/auto_map.yaml
src/xxu_bringup/maps/auto_map.pgm
```

自主探索建图：

```bash
# 终端 1：启动仿真，不启动 Nav2
cd ~/xxu_2026/xxu_nav_2026
source install/setup.zsh
src/xxu_description/scripts/kill_simulation.sh
ros2 launch xxu_bringup simulation.launch.py start_navigation:=false rviz:=false

# 终端 2：启动 SLAM + Nav2 + explore_lite
cd ~/xxu_2026/xxu_nav_2026
source install/setup.zsh
ros2 launch xxu_slam_toolbox autonomous_mapping.launch.py
```

使用键盘控制辅助建图：

```bash
ros2 run key_controll wasd_keyboard_teleop.py
```

## 键盘控制

```bash
ros2 run key_controll wasd_keyboard_teleop.py
```

按键：

- `W/S`：前后移动
- `A/D`：左右平移
- `Q/E`：左右旋转
- `G`：切换小陀螺模式
- 空格：停止
- `Ctrl-C`：退出

参数示例：

```bash
ros2 run key_controll wasd_keyboard_teleop.py --ros-args -p linear_acceleration:=6.0 -p gyro_angular_speed:=1.8
```

## src 文件结构

```text
xxu_nav_2026/
└── src/
    ├── xxu_bringup/                    # 主启动包，负责仿真、导航、地图、RViz 和航点配置
    ├── xxu_description/                # 机器人模型、URDF/Xacro、Gazebo 世界、RViz 和辅助脚本
    ├── xxu_chassis_controller/         # ros2_control 全向底盘控制器
    ├── xxu_livox_sim/                  # Livox Mid-360 激光雷达仿真
    ├── xxu_pointcloud_processing/      # 点云预处理，例如裁剪、降采样等
    ├── small_point_lio/                # Small Point-LIO 里程计
    ├── xxu_slam_toolbox/               # SLAM 建图与自主探索相关启动配置
    ├── pointcloud_to_laserscan/        # 点云转 LaserScan
    ├── cpp_lidar_filter/               # C++ 激光雷达滤波器
    ├── fake_vel_transform/             # 虚拟速度/坐标变换相关节点
    ├── key_controll/                   # WASD 键盘遥控节点
    ├── pb_nav2_plugins/                # 自定义 Nav2 插件
    ├── pb_omni_pid_pursuit_controller/ # 全向 PID 路径追踪控制器
    ├── goal_approach_controller/       # 目标接近控制器
    ├── ai_controller_tuner/            # 控制器参数调节辅助工具
    └── m-explore-ros2/                 # explore_lite / map_merge 自主探索子模块
```

## 自定义 Nav2 插件

`src/pb_nav2_plugins` 当前提供两个插件：

- `pb_nav2_behaviors::BackUpFreeSpace`：替换 Nav2 默认 `backup` recovery 行为，先查询局部 costmap，再朝附近最宽自由空间方向移动。
- `pb_nav2_costmap_2d::IntensityVoxelLayer`：基于 `ObstacleLayer` 的体素障碍层，只把 `PointCloud2` 中 `intensity` 落在配置范围内的点标记为障碍物。

`BackUpFreeSpace` 在当前导航配置中作为 `backup` 行为使用：

```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "pb_nav2_behaviors::BackUpFreeSpace"
    wait:
      plugin: "nav2_behaviors::Wait"
    max_radius: 1.0
    service_name: "local_costmap/get_costmap"
    visualize: false
```

相关配置文件：

```text
src/xxu_bringup/config/nav2_navigation.yaml
src/xxu_bringup/config/nav2_exploration.yaml
```

## 常用检查

检查插件包是否可见：

```bash
ros2 pkg prefix pb_nav2_plugins
```

检查 costmap 服务：

```bash
ros2 service list | grep get_costmap
```

开启 `BackUpFreeSpace` 可视化后查看 marker：

```bash
ros2 topic echo /back_up_free_space_markers
```
