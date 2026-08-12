# pb_omni_mppi_controller

全向底盘 Nav2 MPPI 控制器插件。控制器在 `base_link` 坐标系内采样
`vx/vy/wz` 控制序列，并使用局部路径和 local costmap 计算滚动优化代价。

默认配置先采用较小的预测窗和采样数，适合当前 15 Hz 的仿真控制循环：

- `time_steps: 15`、`model_dt: 0.1`：1.5 秒预测窗；
- `batch_size: 256`：每周期 256 条控制序列；
- `path_weight` / `heading_weight`：路径跟踪代价；
- `collision_weight` / `collision_cost_threshold`：local costmap 碰撞代价。

动态参数仍使用 Nav2 的 `FollowPath.<parameter>` 命名空间。若仿真控制周期
无法稳定保持，可先降低 `batch_size`，再调节噪声和温度。
