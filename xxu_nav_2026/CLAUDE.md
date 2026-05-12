# CLAUDE.md

XXU 机器人 ROS 2 + Gazebo Harmonic 仿真项目。



## 构建与环境

```bash
cd ~/xxu_2026/xxu_nav_2026
colcon build --symlink-install
source install/setup.zsh
```

## 启动 RViz / Gazebo 前必须清理残留进程

每次启动前必须先 kill 所有相关进程，否则残留进程会污染环境，导致状态异常。

```bash
src/xxu_description/scripts/kill_simulation.sh
```

## 启动 RViz（模型可视化）

```bash
ros2 launch xxu_description display.launch.py
```

使用 `src/xxu_description/urdf/xxu.urdf.xacro` 和 `src/xxu_description/rviz/display.rviz`。

## 启动 Gazebo（物理仿真）

```bash
ros2 launch xxu_description gazebo.launch.py
```

Gazebo 世界: `src/xxu_description/worlds/empty_with_sensors.sdf`。

启动后会自动：
- 打开 Gazebo Harmonic 仿真窗口，机器人生成于 x=1.75, y=0.0, z=0.05, yaw=180°
- ros_gz_bridge 桥接: `/cmd_vel`, `/joint_states_gz`, `/imu`, `/scan`, `/clock`
- 打开键盘遥控终端（teleop_twist_keyboard → /cmd_vel_keyboard → cmd_vel_watchdog → /cmd_vel，超时 0.3s 自动回零）

## 项目结构

```
xxu_nav_2026/
└── src/
    └── xxu_description/
        ├── launch/
        │   ├── display.launch.py    # RViz 启动
        │   └── gazebo.launch.py     # Gazebo 仿真启动
        ├── urdf/                    # 机器人 URDF/Xacro 模型及 Gazebo 插件
        ├── meshes/                  # 网格文件
        ├── rviz/display.rviz        # RViz 配置
        ├── worlds/                  # Gazebo 世界 SDF
        ├── scripts/                 # Python 脚本（cmd_vel_watchdog 等）
        ├── CMakeLists.txt
        └── package.xml
```
