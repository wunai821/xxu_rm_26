# fake_vel_transform

本功能包启动时，Fake Velocity Transform 会在底盘 `robot_base_frame` 上创建一个稳定的 `fake_robot_base_frame`（默认 `gimbal_yaw_fake`）。它与底盘原点一致，但使用 `/odom` 的底盘 yaw 做反向补偿，使该速度参考系不随底盘航向变化。云台真实的 `base_link -> gimbal_link -> mid360_link` TF 保持不变，供 LiDAR/LIO 使用。节点同时会订阅 `input_cmd_vel_topic`，将速度转换到底盘 `robot_base_frame` 后发布到 `output_cmd_vel_topic`。

主要目的是适配 LiDAR 固连在旋转云台上的 NAV2 链路：NAV2 使用稳定的 `gimbal_yaw_fake` 作为速度参考系，云台继续使用真实旋转 TF 扫描；输出命令再转换为底盘 `base_link` 坐标系，交给底盘轮速逆解。

## Published Topics

* `tf` (`tf2_msgs/msg/TFMessage`) - 与机器人可移动关节相对应的变换
* `output_cmd_vel_topic` (`geometry_msgs/msg/TwistStamped`) - 转换后的速度指令，保留并更新坐标系信息

## Subscribed Topics

* `input_cmd_vel_topic` (`geometry_msgs/msg/TwistStamped`) - 机器人的速度指令
* `odom_topic` (`nav_msgs/msg/Odometry`) - 里程计数据

## Parameters

* `odom_topic` (`string`, default: "odom") - 里程计话题；节点使用 odometry 的时间戳建立 yaw 历史
* `robot_base_frame` (`string`, default: "base_link") - 底盘速度执行坐标系，也是 fake frame 的父坐标系
* `fake_robot_base_frame` (`string`, default: "gimbal_yaw_fake") - 稳定的 NAV2 速度参考坐标系
* `input_cmd_vel_topic` (`string`, default: "") - 输入速度指令的话题
* `output_cmd_vel_topic` (`string`, default: "") - 输出速度指令的话题。将原本基于 `fake_robot_base_frame` 的速度变换到 `robot_base_frame` 后发布
* `spin_speed` (`double`, default: 0.0) - 移动时叠加到底盘角速度上的小陀螺速度；输入速度指令中的 `angular.z` 会被保留，设为 `0` 禁用
* `gyro_linear_threshold` (`double`, default: 0.01) - 判定机器人正在平移的速度阈值（m/s）

## Launch 集成

在 `xxu_description`、`xxu_bringup`、`xxu_slam_toolbox` 的主启动链路中，fake frame 模式默认关闭。需要启用时添加：

```bash
use_fake_frame:=true
```

默认关闭时，NAV2 使用 `base_link`；启用后，NAV2 使用 `gimbal_yaw_fake`，并启动 `fake_vel_transform` 将碰撞监测后的 `/cmd_vel_collision` 转换为 `/cmd_vel_transformed`。最终由 watchdog 发布到 `/cmd_vel`，再交给底盘控制器执行。
