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
| `nav_velocity_smoothing_window` | `0.0` | 水平速度的因果平均窗口（秒）；项目仿真/实车入口设为 `0.3`，设 `0` 可关闭 |
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
运行时仅支持修改 `spin_speed`；其他参数修改后重启。

2026-10-17 已按用户确认删除有界航向外推、对应参数及诊断。节点只保存最新有效里程计航向，按最新测量值转换指令；帧变化重置速度窗口。历史实验数据与报告已删除。

导航速度先从底盘系旋转到稳定的 odom 系，再对当前及最近窗口内的有效消息做
算术平均。只滤波 `/odom_nav.twist` 的水平速度，保留最新位姿和时间戳；原始 `/odom`
继续用于定位、TF、底盘航向与数据新鲜度判断。角速度仍为虚拟参考系定义的零。
不同父/子坐标系之间不混合历史，断流超过窗口后第一条有效速度直接使用新样本。
没有速度死区；低速运动不被强制置零。滤波协方差保持原有保守值，未经实测标定。

0.3 秒窗口在约 9 Hz 里程计上通常平均 3 条消息，带来约 0.1 秒量级的速度反馈延迟。


```bash
python3 tools/navigation/validate_gyro_navigation.py --nav-velocity-window 0.3
```
