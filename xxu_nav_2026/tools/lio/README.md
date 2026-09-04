# MID360 旋转云台实机回归

这组工具用于把实机问题固定成可重复的数据集。录包同时保存原始 MID360
点云/IMU、云台关节、TF、Small Point-LIO 输出和可选的独立里程计。

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
`/cloud_registered` 等重型派生点云。如确实需要检查派生云，再加
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
