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
- 启动速度看门狗（/cmd_vel_keyboard → cmd_vel_watchdog → /cmd_vel，超时 0.3s 自动回零）

键盘控制单独开一个终端运行：

```bash
ros2 run key_controll wasd_keyboard_teleop.py
```

按键: `W/S` 前后移动，`A/D` 左右平移，`Q/E` 左右旋转；小陀螺模式下 `Q/E` 切换旋转方向，`G` 切换小陀螺模式（边旋转边行进），空格停止，`Ctrl-C` 退出。

可调参数: `linear_speed`、`angular_speed`、`linear_acceleration`、`angular_acceleration`、`gyro_angular_speed`、`key_timeout`。例如：

```bash
ros2 run key_controll wasd_keyboard_teleop.py --ros-args -p linear_acceleration:=6.0 -p gyro_angular_speed:=1.8
```

如果启动时报 `This member is not been selected`，说明当前 FastDDS/FastCDR 运行库在创建 ROS 节点前失败。安装并切换 CycloneDDS：

```bash
sudo apt-get install ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run key_controll wasd_keyboard_teleop.py
```

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
    └── key_controll/
        ├── scripts/wasd_keyboard_teleop.py  # WASD/QE 键盘控制
        ├── CMakeLists.txt
        └── package.xml
```
