# fake_vel_transform

上位机小陀螺与导航速度适配节点。详细职责、TF、启动及回归说明见
[小陀螺导航说明](../../tools/navigation/README.md)。

- 读取真实 `/odom`，发布唯一的稳定坐标系 `gimbal_yaw_fake`。
- fake TF 的父坐标系是 `/odom.child_frame_id`（项目中为 `base_footprint`）。
- 发布 `/odom_nav`：稳定参考系的位姿与平面速度，虚拟角速度为零。
- 将稳定参考系的导航平移指令转为 `base_link` 指令，移动时叠加 `spin_speed`。
- 真实的 `base_link → gimbal_link → 雷达` TF 保持独立，供传感器补偿与定位使用。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `odom_topic` | `Odometry` | 原始底盘里程计；项目 launch 设置为 `/odom` |
| `nav_odom_topic` | `/odom_nav` | 导航用里程计 |
| `robot_base_frame` | `base_link` | 输出指令坐标系 |
| `fake_robot_base_frame` | `gimbal_yaw_fake` | 唯一稳定导航参考系 |
| `input_cmd_vel_topic` | `cmd_vel` | 项目使用 `/cmd_vel_collision` |
| `output_cmd_vel_topic` | `aft_cmd_vel` | 项目使用 `/cmd_vel_transformed` |
| `spin_speed` | `0.0` | 可运行时更新；项目仿真/实车入口默认传入 `1.5` rad/s |
| `gyro_linear_threshold` | `0.01` | 叠加自旋的平移速度阈值 |
| `publish_rate` | `100.0` | 按最新底盘航向重算输出的频率 |
| `command_timeout` | `0.3` | 原始速度指令超时 |
| `odom_timeout` | `0.5` | 里程计接收间隔及消息时间戳的最大允许年龄（秒） |

到点/碰撞零指令会停止底盘自旋。无输入时不输出运动；断流时输出零速度。
重复、倒序、过期或超前当前 ROS 时钟超过 0.1 秒的里程计会被丢弃，
不会刷新超时计时或更新航向。有效且时间戳递增的数据恢复后可继续输出。
仿真时间回退后，在时间戳超过上一条有效数据之前保持停车；重置仿真时应重启此节点。
运行时只支持修改 `spin_speed`；其余节点参数修改后重启。
