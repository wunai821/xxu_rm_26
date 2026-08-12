# 总项目运行与修复日志

## 2026-07-16：仿真、导航与 AI 调参链路

### 状态摘要

本轮 AI 调参无法获得有效导航样本，并非单一控制器参数问题。已定位并修复仿真时钟、Nav2 生命周期、默认地图选择、自动初始位姿 QoS 与调参器时钟配置等跨模块问题。当前运行实例的关键 Nav2 节点已可全部激活；仍需一次完整冷启动与 AI 调参闭环验收。

### 已修复

- **仿真时间**：`bridge_clock` 曾早于 Gazebo world 就绪，出现 ROS 中有 `/clock` 发布者但无时钟样本的竞态。Gazebo transport 侧 `/clock` 正常递增，重启 bridge 后 ROS `/clock` 恢复（约 330 Hz）。主 launch 已改为延迟 2 秒启动 clock bridge。
- **时间使用统一**：AI tuner 与目标发送器已默认 `use_sim_time=true`，直接订阅仿真主链 `/clock`；仅在外部 Gazebo 未提供时钟桥时，AI launch 才可显式开启 clock bridge 兜底。
- **保存地图一致性**：保存地图链路为 `complex_mapping.sdf + auto_map.yaml -> auto_map.pgm`。独立 `navigation.launch.py` 的默认地图已由错误的 `empty.yaml` 改为 `auto_map.yaml`。
- **Nav2 生命周期**：原先 Nav2 在初始位姿前激活规划器，导致后半段节点 inactive。自动初始位姿由第 28 秒提前至 LIO 启动后的第 20 秒；当前已验证剩余节点可成功激活。
- **自动定位 QoS**：`auto_initial_pose` 的 `/scan` 订阅改为 sensor-data QoS，兼容 best-effort 激光数据。
- **运行时库**：补充 `small_point_lio` 组件库安装规则；启动 overlay 需要包含 `fake_vel_transform` 与 `small_point_lio` 的动态库路径。
- **DDS 稳定性**：多次手动启动造成残留进程时，CycloneDDS 会耗尽 participant index。新增仿真 DDS 配置以提高自动 participant index 上限；启动前仍应清理旧仿真实例。
- **AI 调参可靠性**：API key 已从仓库 YAML 移除，仅从运行时环境变量读取；指标优先采集 `/cmd_vel_smoothed`，无该数据时才回退 `/cmd_vel_nav`，日志记录 `command_source`。
- **控制器能力**：PID 积分限幅已接入；增加路径局部平滑、曲率前瞻、横向加速度与减速距离联合限速。

### 当前待验收

1. 完整冷启动保存地图仿真，确认 `/clock` 无需人工重启 bridge 即持续发布。
2. 确认 AMCL 持续发布并稳定维护 `map -> odom`。
3. 启动 AI tuner，执行 `auto_map` 内有效路线，确认 `/cmd_vel_smoothed` 有样本且 tuner 日志的 `command_source` 为 `cmd_vel_smoothed`。
4. 核验曲率调参路线均位于 `auto_map` 可通行区域。
5. 修复或重新生成顶层 `install/setup.bash` overlay，消除手工补环境变量的需求。
