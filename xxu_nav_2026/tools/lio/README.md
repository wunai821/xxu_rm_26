# MID360 旋转云台实机回归

这组工具用于把实机问题固定成可重复的数据集。录包同时保存原始 MID360
点云/IMU、云台关节、TF、Small Point-LIO 输出和可选的独立里程计。

## 仿真默认链路与 A/B 基线

`gazebo.launch.py` 默认启用 `compensate_gimbal_for_lio:=true`：按点时间将
云台旋转从点云和 IMU 中补偿到刚性虚拟帧 `lio_base_sensor`，再运行
Small Point-LIO。这是旋转云台仿真的正常运行链路。原始未补偿路径只用于
隔离测试：

```bash
ros2 launch xxu_description gazebo.launch.py \
  gimbal_mode:=spin gimbal_spin_rate:=1.0 \
  compensate_gimbal_for_lio:=false
```

MID360 原生仿真插件的射线请求和命中结果都必须使用其挂载模型实体坐标系；
不要把 `RaycastData` 的命中点直接当世界坐标。修改该插件后，完整重编译：

```bash
colcon build --symlink-install \
  --packages-select xxu_livox_sim xxu_description small_point_lio
```

## 启动补偿版 LIO

先启动 `livox_ros_driver2` 和带时间戳的云台 `/joint_states`，再运行：

```bash
source install/setup.bash
ros2 launch xxu_description mid360_lio.launch.py
```

该启动链将 `/livox/lidar` 按每个点的绝对时间插值云台角度，将
`/livox/imu` 的加速度从 g 换算为 m/s²，并把两者补偿到刚性虚拟帧
`lio_base_sensor` 后交给 Small Point-LIO。已有 robot_state_publisher 时使用：

```bash
ros2 launch xxu_description mid360_lio.launch.py \
  start_robot_state_publisher:=false
```

云台关节名必须为 `gimbal_joint`，其时间基准须与 Livox 点时间一致。实机安装尺寸或
Livox 标定外参变化后，应同步更新启动文件中的 `sensor_offset` 和 `sensor_rpy`。
若二次封装的 Livox 驱动已经输出 m/s²，须加 `input_acceleration_scale:=1.0`；编码器
零位与机械零位存在偏差时，可用 `joint_position_offset:=<rad>` 同时校正点云和 IMU。

## 录制

先启动实机驱动、动态云台 TF 和 Small Point-LIO，并 source ROS 2 与工作空间。
每个工况单独录一个包：

```bash
tools/lio/record_mid360_regression.sh locked_stationary --duration 20
tools/lio/record_mid360_regression.sh rotating_stationary --duration 20
tools/lio/record_mid360_regression.sh locked_motion
tools/lio/record_mid360_regression.sh rotating_motion
```

运动组应走同一条可测量的路线，旋转组使用云台正常工作速度；不要为了复现仿真
而强行把实机设为 4 rad/s。如果底盘或动捕提供独立里程计，增加例如：

```bash
tools/lio/record_mid360_regression.sh rotating_motion \
  --reference-odom /wheel_odom
```

为避免录包进程与 LIO 抢占 CPU，默认 MCAP 使用低开销 `fastwrite`，且不录
`/cloud_deskewed` 等重型派生点云。如确实需要检查派生云，再加
`--include-derived-clouds`；存储空间受限时可另选 `--storage-profile zstd_fast`。

默认关键话题为 `/livox/lidar`、`/livox/imu`、`/joint_states` 和 `/odom`，
均可通过脚本参数覆盖。缺少任一关键话题时脚本会拒绝开始，避免得到无法回放的包。
每个包还包含采集时的 Git commit、话题清单和 Small Point-LIO 参数快照。

## 检查

```bash
tools/lio/inspect_mid360_bag.py bags/lio_regression/<bag-directory>
```

检查器输出：

- 点云频率、每帧点数、有限坐标比例；
- 点内时间字段、单帧时间跨度、时间组数量，以及相邻帧的间隙、重叠和共享时间组；
- IMU 与云台关节频率；底盘静止时还可比较两者角速度；
- `/odom` 的路径长度、净位移和相对起点最大漂移；
- 可选独立里程计的同类统计。

静止旋转包首先看 `max_xy_from_start`；两组运动包再比较 LIO 与实测路线或独立里程计
的净位移。这样可将“旋转时漂移”拆分为输入时序/格式问题、静止旋转漂移和运动尺度
误差三个层次。

## 仿真动作验证与小陀螺速度上限

启动仿真后，可用 Gazebo 真值桥接和动作验证器检查 `/odom` 的帧、位姿和速度：

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/world/complex_mapping/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V'
python3 tools/lio/validate_motion.py static --duration 3
python3 tools/lio/validate_motion.py translate --duration 2.5 --speed 0.2
python3 tools/lio/validate_motion.py rotate --duration 2.5 --angular-speed 0.5
python3 tools/lio/validate_motion.py gyro_translate --duration 2.5 --speed 0.2
```

基础定位验收建议用 `gyro_spin_rate:=0`；组合工况可用键盘默认的小陀螺
`gyro_spin_rate:=1.5`。验证器要求 `/odom` 的 `header.frame_id=odom`、
`child_frame_id=base_footprint`，并同时报告 Gazebo 真值误差、`Odometry.twist`、
轮速和云台速度。

当前底盘参数为轮半径 0.0762 m、轮速上限 100 rad/s。对纯 X 平移叠加小陀螺角速度，
轮速逆解为：

```text
wheel_i = (tx_i (vx - wz y_i) + ty_i (wz x_i)) / wheel_radius
```

由四轮几何计算得到“不触发全局轮速缩放”的平移范围。这里的范围是控制器
轮速约束，不是安全运行速度；当纯小陀螺项本身已超过轮速上限时，不存在可行的
平移速度范围：

| 小陀螺 `wz` | 平移范围 `vx` | 说明 |
|---:|---:|---|
| 1.5 rad/s | -10.11 ～ +10.11 m/s | 远高于当前 Nav2 上限 1.5 m/s |
| 31.416 rad/s | 不存在 | 纯小陀螺轮速约 130.18 rad/s，已超过 100 rad/s |

仅旋转、不平移时，小陀螺角速度约不超过 24.13 rad/s 才不会触发轮速缩放。
超过该范围时控制器会按比例缩放四个轮速，不能把输入命令值当作实际底盘速度。
仿真中 `gyro_spin_rate:=1.5`、`vx=5.0` 的轮速峰值约 66.3 rad/s，`vx=10.5`
达到 100 rad/s 饱和；这两个高速度点仅用于测边界，不代表安全运行速度。

## 仿真平移丢失诊断

在工作空间根目录编译并加载环境，然后开启诊断，先测锁定云台：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select xxu_livox_sim xxu_description small_point_lio --symlink-install
source install/setup.bash
ros2 launch xxu_description gazebo.launch.py \
  gimbal_mode:=hold lio_motion_diagnostics:=true \
  2>&1 | tee /tmp/lio_locked_diagnostics.log
```

沿用已有底盘运动测试方法，同步记录 `/odom` 和 Gazebo 的 `xxu` 模型真值。
若复测旋转组，将 `gimbal_mode:=hold` 改为 `gimbal_mode:=spin gimbal_spin_rate:=1.0`。
诊断参数默认关闭；直接启动 Small Point-LIO 时可在参数文件中设置
`motion_diagnostics_en: true`。该开关在启动时读取，修改后需重启节点。

初始化完成且滤波器时间推进后，每至少 1 个仿真秒输出一组 `LIO_DIAG` / `LIO_MATCH`。
`t` 是滤波器处理到的传感器时间，`dt` 是统计窗口长度；不要用 ROS 日志前缀的墙钟时间
与真值对齐。停止接收数据或初始化未完成时不会持续输出；最后不足 1 秒的窗口不会输出。

- `p_xy` / `v_xy`：滤波器内部位置与速度，在 odom 坐标系中，单位 m / m/s；
  内部位置是估计器原点的位置，`/odom` 发布的是经过 TF 换算的底盘位置。
- `net_dp_xy`：窗口净位移；`predict_dp_xy`：状态传播累计位移；
  `point_dp_xy`：点面更新累计位置修正；`imu_dp_xy`：IMU 更新累计位置修正。
  这些是有符号向量累加，不是路径长度。`balance_norm` 检查三维净位移与各步骤
  （含平面约束）修正之和的差，正常应接近浮点舍入误差。
- `predict_dv_xy` / `point_dv_xy` / `imu_dv_xy`：对应步骤的累计水平速度修正，单位 m/s。
- `attempted` / `accepted` / `ratio`：实际进入点面模型的点数、有效匹配数及比例。
  不是原始点云点数，也不包含预处理滤掉的点。
- `no_neighbors` / `nonplanar` / `residual_rejected`：邻居不足 5 个、邻居平面检查失败、
  残差门限拒绝的数量。三者加 `accepted` 应等于 `attempted`。
- `residual_rms`：有效匹配更新前的点面残差 RMS，单位 m；没有有效点时填 0，
  必须结合 `accepted` 判断，不能把此时的 0 当成完美匹配。
- `normal_xy_eigen`：有效平面单位法向的水平外积均值的两个特征值，按小到大排列。
  两者都接近 0 表示水平位置约束弱；仅一个接近 0 表示至少有一个水平方向约束弱。
  这是几何诊断，不是完整位姿可观测性或滤波协方差。
- `LIO_PROP`：`rpy_deg` 是滤波器内部姿态（roll / pitch / yaw）；`acc_meas_body` 是最近
  IMU 比力（按 LIO 的加速度尺度换算），`acc_est_body` 是滤波器状态中的比力，
  `acc_world=R·acc_est_body+g` 是状态传播实际使用的世界系加速度。该量的水平分量用于
  判断预测位移为何正向、反向或接近零。`normal_xy_weak_dir` / `normal_xy_strong_dir`
  是 `normal_xy_eigen` 对应的小/大特征值方向，均在 odom 的 XY 坐标系；特征向量正负号
  等价。它们仅说明平面法向约束的方向性，不能单独代表完整位姿可观测性。
- `imu_updates` / `imu_failed`：IMU 更新尝试数及失败数；
  `late_points` / `late_imus`：主处理循环因落后于当前滤波器时间而丢弃的数量，
  不包含补偿器或预处理阶段的丢弃。

判断时将每个统计窗口与真值运动对齐：预测位移长期接近零时，结合速度和速度修正
检查传播；预测有位移但点面修正近似反向抵消时，检查匹配和地图；匹配率低时查看拒绝
原因；匹配率高但水平法向特征值弱时，检查场景几何。内部位置已跟随真值而 `/odom`
未跟随时，再追踪 TF 与发布链路。单个指标不足以直接认定根因。

```bash
rg 'LIO_DIAG|LIO_MATCH' /tmp/lio_locked_diagnostics.log
```

若怀疑仿真点云没有随底盘真值运动，录制 `/mid360/livox_points` 与 Gazebo
`/world/complex_mapping/dynamic_pose/info` 的 JSONL 后运行：

```bash
python3 tools/lio/check_cloud_truth_alignment.py <bag目录> <dynamic_pose.jsonl>
```

该工具比较相隔 10 秒的两段点云。静态场景在真值对齐后应提高体素重叠，并降低
较大分位数的最近邻误差；中位数会受重复 Livox 扫描图样影响，不能单独作为结论。
