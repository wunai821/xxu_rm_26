# 小陀螺导航：职责与验证

仿真、实车、独立导航和自主探索入口的 `use_fake_frame` 默认均为 `true`。
分开启动时若需要普通底盘坐标系，必须在两端均设置 `use_fake_frame:=false`。
仿真入口的 `cmd_vel_out_topic` 会同步传给碰撞监测器、速度变换和 watchdog。

`enable_lio:=false enable_cmd_vel_odom:=true` 仅用于诊断：扫描改用云台补偿点云，
不包含 LIO 底盘运动去畸变，地图导航仍不启动。watchdog 的扫描新鲜度检查保留。
启动参数连接回归可在加载工作空间环境后运行：

```bash
python3 -m unittest discover -s src/xxu_bringup/test -v
```

只验证速度适配与里程计超时保护（包含重复、倒序、延迟和未来时间戳）：

```bash
python3 tools/navigation/validate_gyro_navigation.py --adapter-only
```

本项目的导航按二维平面全向底盘处理。底盘自旋、雷达云台旋转是两个独立运动：

| 模块 | 负责什么 |
|---|---|
| Small Point-LIO | 结合云台关节、点云/IMU 补偿，输出真实底盘 `/odom` |
| Nav2 MPPI | 在稳定参考系中规划 XY 平移和避障 |
| fake_vel_transform | 发布稳定参考系及 `/odom_nav`，实时转回底盘速度，叠加上位机小陀螺 |
| 底盘控制器/下位机 | 执行轮速，保留轮速限制和超时停车 |
| 云台驱动 | 执行独立的云台控制并发布带时间戳的 `gimbal_joint` 状态 |

## 只有一个 fake 坐标系

```text
map → odom → base_footprint
                  ├─ base_link → gimbal_link → 雷达
                  └─ gimbal_yaw_fake
```

`gimbal_yaw_fake` 原点现在与 `base_footprint` 一致，方向固定在 odom 中。
TF 由 `fake_vel_transform` 发布：抵消真实底盘的姿态。它不接在真实云台下面。
`base_link` 与 `base_footprint` 在现有模型中仅有 Z 高度差，平面速度旋转相同。
此实现面向二维地面导航，不是跨坡度的完整三维运动模型。

- `/odom`：真实 `odom → base_footprint`，twist 表达在真实底盘坐标系，包含自旋。
- `/odom_nav`：同一时刻的稳定参考系位姿，线速度旋转到稳定坐标系；虚拟角速度为零。
- `/odom_nav` 不是另一个定位器，也不发布第二条 `odom → base_footprint` TF。
- AMCL/GICP、云台补偿和 watchdog 继续使用真实底盘链路。

速度链路：

```text
MPPI → /cmd_vel_nav → 平滑 → /cmd_vel_smoothed
 → 碰撞监测 → /cmd_vel_collision
 → fake_vel_transform → /cmd_vel_transformed
 → watchdog → /cmd_vel → 底盘轮速
```

开启 `use_fake_frame` 时，启动文件自动将 controller、BT navigator 和速度平滑器的
里程计输入切换到 `/odom_nav`。MPPI 的 `wz_max` 和 `az_max` 为零，保留非零角速度
采样方差，避免采样代价中的除零。禁用角度评分，采用 PositionGoalChecker；到点只
检查 XY，不要求真实底盘或云台转到目标姿态。恢复行为树移除 Spin，保留清图、等待
和后退。关闭 fake 模式时保留原 YAML 的普通导航设置。

fake 模式的碰撞轮廓使用半径 0.42 m 的圆形旋转包络（包括模型中的突出车轮），
近场停车区改为各方向对称的八边形。实车外形改变时必须按实际旋转包络更新。

## 上位机控制小陀螺

仿真和实车入口均提供 `gyro_spin_rate`，默认 1.5 rad/s。正负值决定旋转方向，0 关闭。
该值由上位机速度变换节点叠加，下位机不应重复叠加一份。

```bash
# 仿真：同时启用地图导航与小陀螺；地图定位仍需先验证
ros2 launch xxu_bringup simulation.launch.py \
  start_navigation:=true enable_localization:=true \
  use_fake_frame:=true gyro_spin_rate:=1.5 rviz:=true

# 实车：需要真实雷达、云台 JointState、底盘接口已经就绪
ros2 launch xxu_bringup real_robot.launch.py \
  start_navigation:=true enable_localization:=true \
  use_fake_frame:=true gyro_spin_rate:=1.5
```

任务/决策节点可调用 `/fake_vel_transform/set_parameters` 自主切换，无需人工操控。
下列命令只是调试同一接口：

```bash
ros2 param set /fake_vel_transform spin_speed 0.0
ros2 param set /fake_vel_transform spin_speed 1.5
```

当前语义是**移动时自旋**：平移速度大于 0.01 m/s 时叠加；到点、零速度命令、
碰撞停车、指令或里程计超时均停止底盘自旋。雷达云台的持续旋转不受此开关控制。
若比赛策略需要到点后持续原地小陀螺，需要另外接入任务使能与安全状态，不能把
导航停车/碰撞停车的零速度直接覆盖掉。显式维护测试仍可发送 physical angular.z，
以兼容 `tools/lio/validate_motion.py`；正常 fake 导航输出的 angular.z 为零。

速度转换以 100 Hz 使用最新底盘航向更新持有的平移指令，watchdog 也以 100 Hz
转发；底盘轮速发布仍为 50 Hz。旧指令不会因重发而无限续期，变换节点同时检查
原指令时间戳和接收时间，超过 0.3 s 输出零速度；里程计超时为 0.5 s。

默认 1.5 rad/s 来自已完成的组合运动验证。轮速不饱和边界不是导航速度上限，
仍需用定位质量、制动距离、传感器延迟和障碍物场景确定比赛速度。

## 可重复回归

```bash
cd ~/xxu_2026/xxu_nav_2026
source install/setup.zsh
ROS_LOG_DIR=/tmp/xxu-gyro-navigation-logs python3 tools/navigation/validate_gyro_navigation.py
```

工具使用独立 ROS 域 187、本机通信，不发布 `/cmd_vel`，不会启动硬件。
请勿在该域同时运行其他机器人节点。它启动真实的 MPPI controller_server 和速度
变换节点，使用理想二维底盘积分及空旷扫描测试：

- fake/non-fake 参数选择；
- 旋转底盘的线速度反馈、虚拟角速度与输出旋转方向；
- 两条导航指令之间按最新航向更新输出；
- 动态开关小陀螺、零命令、指令和里程计断流停车；
- 不自旋/自旋两种状态下 FollowPath 到点，目标航向不影响到点判定。

此测试不替代 Gazebo 中旋转雷达、地图定位、避障以及实车完整链路验收。
