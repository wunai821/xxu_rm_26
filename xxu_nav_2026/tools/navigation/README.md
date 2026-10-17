# 小陀螺导航：职责与验证

新仿真入口为 `rm_sim_navigation.launch.py`，采用固定版本 rm_sim_26 哨兵及 RMUC 2025 场地，版本见 [simulation.repos](../../simulation.repos)。
默认使用 RGL Mid360；首次运行前执行 [build_rgl.sh](build_rgl.sh)，或显式设置 `lidar_mode:=gpu_lidar`。
缺少 RGL 库时启动会报错，需要完成构建或选择已有后端。

RMUC 地图由 [generate_rmuc_map.py](generate_rmuc_map.py) 根据固定场地碰撞几何生成。
用户 RMUL 场地入口为 `rmul_navigation.launch.py`，使用随包模型和地图，仍搭配上游原车。
重新转换场地使用 [prepare_rmul_field.py](prepare_rmul_field.py)；EGA CAD 读取与折叠姿态要求见 [CAD 说明](cad/README.md)。
RGL 当前按 link 规则忽略承载雷达的云台视觉几何，尚未实现逐零件本体遮挡。

仿真、实车、独立导航和自主探索入口的 `use_fake_frame` 默认均为 `true`。
分开启动时若需要普通底盘坐标系，必须在两端均设置 `use_fake_frame:=false`。
仿真入口的 `cmd_vel_out_topic` 会同步传给碰撞监测器、速度变换和 watchdog。

`enable_lio:=false enable_cmd_vel_odom:=true` 仅用于诊断：扫描改用云台补偿点云，
不包含 LIO 底盘运动去畸变，地图导航仍不启动。watchdog 的扫描新鲜度检查保留。
测试与实验候选源码已于 2026-10-17 清理，当前保留链路见下文。

本项目的导航按二维平面全向底盘处理。底盘自旋、雷达云台旋转是两个独立运动：

| 模块 | 负责什么 |
|---|---|
| Small Point-LIO | 结合云台关节、点云/IMU 补偿，输出真实底盘 `/odom` |
| Nav2 MPPI | 在稳定参考系中规划 XY 平移和避障 |
| fake_vel_transform | 发布稳定参考系及 `/odom_nav`，实时转回底盘速度，叠加上位机小陀螺 |
| 底盘控制器/下位机 | 执行轮速，保留轮速限制和超时停车 |
| 云台驱动 | 执行独立的云台控制并发布带时间戳的 `gimbal_joint` 状态 |

## 共享启动与参数来源

- [nav2_navigation.yaml](../../src/xxu_bringup/config/nav2_navigation.yaml) 与 [nav2_exploration.yaml](../../src/xxu_bringup/config/nav2_exploration.yaml)保存各自基础参数，non-fake 模式直接使用基础行为。
- [nav2_fake_profile.yaml](../../src/xxu_bringup/config/nav2_fake_profile.yaml)是 fake 模式覆盖值的唯一来源：`common` 为导航/探索共有的 15 项；`navigation` 为地图导航专用 PathFollow 三项；`exploration` 当前不增加覆盖，保留探索基础值。此文件是覆盖表，不能作为独立 Nav2 参数文件启动。调节 fake 模式时先确认目标值是否在此表中。
- [launch_helpers.py](../../src/xxu_description/xxu_description/launch_helpers.py)统一构造 Nav2 参数和速度链。仿真时间、坐标系、命令话题和规划容差由启动参数决定；行为树文件名在覆盖表中保存，路径按当前安装位置展开。
- 实车、旧 Gazebo、rm_sim 共用速度转换器和 watchdog 的定义。调用处仅指定平台差异：输入/输出话题、关节话题、自旋值与滤波窗口。rm_sim 最终安全输出仍是 `/rm_sim/cmd_vel_safe`，其底盘适配不变。

共享代码安装在既有 `xxu_description` 包内，没有增加运行节点。修改后需重新构建并加载工作空间；各平台的底盘、传感器和启用条件仍由各自入口负责。

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
- `/odom_nav`：保留最新位姿和时间戳，线速度先旋转到稳定坐标系，再作窗口平均；虚拟角速度为零。
- `/odom_nav` 不是另一个定位器，也不发布第二条 `odom → base_footprint` TF。
- AMCL/GICP、云台补偿和 watchdog 继续使用真实底盘链路。

仿真和实车入口默认使用 `nav_velocity_smoothing_window:=0.3`（秒），
设为 `0.0` 可关闭。该参数在节点启动时生效，已有实例需要重启。

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
导航停车/碰撞停车的零速度直接覆盖掉。维护指令仍可发送 physical angular.z；正常 fake 导航输出的 angular.z 为零。

速度转换以 100 Hz 更新持有的平移指令，使用最新有效测量航向；航向外推与位姿预测 B 已于 2026-10-17 撤除。接口见[fake模块说明](../../src/fake_vel_transform/README.md)。
watchdog 也以 100 Hz 转发；底盘轮速发布仍为 50 Hz。旧指令不会因重发而无限续期，变换节点同时检查
原指令时间戳和接收时间，超过 0.3 s 输出零速度；里程计超时为 0.5 s。
输出频率不等于输入新鲜度，应同时检查来源时间戳与接收间隔。

默认 1.5 rad/s 来自已完成的组合运动验证。轮速不饱和边界不是导航速度上限，
仍需用定位质量、制动距离、传感器延迟和障碍物场景确定比赛速度。

## MPPI 路线基准与速度链分析

保留以下只读记录与离线分析工具：

| 文件 | 用途 |
|---|---|
| `analyze_mppi_benchmark.py` | 离线分析历史 CSV 与 NPY 数据 |
| `observe_mppi_manual.py` | 记录手动导航的速度链和参数 |
| `record_localization_jumps.py` | 记录定位跳变 |
| `monitor_localization_integrity.py` | 持续观察定位与输入完整性 |

地图导航的 fake 模式当前默认是 GoalCritic 权重 15/启用距离 1.2 m，PathFollow 权重 14/前瞻偏移 20/退出距离 0.6 m，PathAlign 阈值 0.5 m。
参数按下方“共享启动与参数来源”分工维护；实际运行值应核对模式合并后的结果。
生成数据可写入 `bags/`，该目录不纳入版本管理。

分析已有记录时，输出目录必须尚不存在：

```bash
python3 tools/navigation/analyze_mppi_benchmark.py \
  bags/mppi_benchmark/existing_run/run.json \
  --output docs/mppi_benchmark_existing_run
```

离线报告提供每段路径 RMS/P95、各级速度峰值、末端采样误差、终点 0.5 m 内停留时间、连续有效采样中的终点回弹、
平滑降速与碰撞停车时间占比、消息新鲜度和采样缺口。路径误差沿用旧数据的
“到最新路径采样点最近距离”，便于对比；末端误差是最后一个有效采样时刻的结果，
可能早于 action 完成时刻。新增记录在 action 完成后等待 1 墙钟秒保存 `settled` 位置、消息年龄及里程计速度，用来识别制动后位移；其中定位修正不能视为物理滑行。速度模跨级比较没有补偿底盘响应延迟，只能定位调查方向。
有效观测秒不包含无效采样间隔，不能直接等同整段 action 耗时。

默认排除非正向或超过 0.25 s 的时间间隔；速度/TF 消息年龄阈值为 0.3 s，
低频全局路径单独使用 2.0 s 阈值，可通过 `--max-gap`、`--max-age` 和
`--max-plan-age` 调整。新数据的完整路线必须全部目标成功且没有运行或取消错误。规划预检查会发布 `/plan`，必须与正在执行的记录器分开运行。
旧 JSON/NPY 数据也可分析，但缺少时间戳和目标总数，报告不会将其自动判定为完整路线。

比较配置时，固定地图、起点、路线、自旋速度及定位模式，保存参数快照，至少重复数圈。
未完成路线只比较对应公共路段；不要用其累计耗时宣称整圈更快。

`monitor_localization_integrity.py --output 全新目录` 可只读记录定位异常，
包括同一 odom 位姿在前后 map→odom 下的大修正、扫描接收中断和粒子扩散。
它也记录原始/补偿 IMU 时间缺口及 odom 异常位移；`--max-odom-speed`
默认 3.5 m/s，仅用于告警，不修改机器人速度上限。
它保存 `health.json` 与 `events.jsonl`，不会拦截 TF、发送目标或停车。
使用与现场一致的 ROS 域和中间件；仿真默认 `use_sim_time=true`。
