# flybrain_nav_bridge

This independent ROS 2 package attaches the MaleCNS V1 controller as a bounded
threat-response bias. It deliberately leaves Nav2 MPPI, velocity smoothing,
collision monitoring and the final command watchdog unchanged.

## Current topic contract

The checked XXU navigation stack is:

```text
/cmd_vel_nav → /cmd_vel_smoothed → /cmd_vel_collision → /cmd_vel
```

For Phase 1/2, run only the default shadow configuration:

```bash
ros2 launch flybrain_nav_bridge flybrain_bridge.launch.py mode:=shadow
```

It reads `/cmd_vel_nav`, `/scan`, and `/odom`, while publishing only
`/flybrain/*` debug streams and optional append-only runtime JSONL logs.

The launch file uses `/home/naiwu/fly_brain/.venv/bin/python`, retaining the
MaleCNS-tested NumPy/SciPy/CUDA stack while making ROS's system PyYAML available
to `rclpy`. Running the installed console script directly is suitable for the
MPPI-only / heuristic checks, but the launch file is the supported way to run
the real neural groups.

Active fusion is intentionally rejected if input and output topics are equal.
When a benchmark launcher has remapped the upstream MPPI output to
`/cmd_vel_mppi`, the safe active contract is:

```text
/cmd_vel_mppi → flybrain_nav_bridge → /cmd_vel_nav → velocity_smoother → collision_monitor → watchdog → chassis
```

The included `benchmark.launch.py` uses an equivalent no-source-change route
available in this workspace:

```text
/cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed
  → flybrain_nav_bridge → /cmd_vel_fused
  → collision_monitor → watchdog → chassis
```

The bridge applies its shared velocity, acceleration, jerk and emergency gate
to the post-smoothing fused command; collision monitoring and the final
watchdog remain downstream. Run one group with:

```bash
ros2 launch flybrain_nav_bridge benchmark.launch.py control_group:=heuristic
```

`benchmark.launch.py` also starts a scenario manager by default. It spawns only
`flybrain_obstacle`, drives it from the scenario YAML using simulation time, and
publishes physical contacts on `/benchmark/contacts`. Scenario coordinates are
explicit Gazebo-world coordinates and must be calibrated once for the selected
map/spawn pose, then held fixed across the paired A/B/C/D seeds.

For each run, `benchmark_metrics` writes an append-only JSON result under
`/home/naiwu/fly_brain/gazebo_results/raw/<scenario>/<controller>/`. It records
contacts, minimum distance/TTC, response timing, path length, safety dynamics,
lambda and controller latency distributions. A repeated seed/controller file is
rejected rather than overwritten.

Use `benchmark_plan.example.yaml` as the batch manifest. It requires explicit
validated map goals and refuses to run Group D without a separate rewired asset.
Its `--dry-run` mode prints the full paired 5-trial matrix before any simulator
is started.

All four groups use exactly the same extractor, TTC smoothstep, velocity limits,
acceleration limits, and safety gate. `mppi` only uses a zero auxiliary command;
`heuristic` enables the deterministic escape baseline; `flybrain` enables the
external controller adapter. `rewired` requires an explicit independent
`brain.rewired_data_dir`; an empty value disables the neural contribution
instead of accidentally reusing the real MaleCNS data. The bridge never writes
or mutates the original MaleCNS data.

The current V1 public API returns a command already blended with a linear TTC
function. To meet the benchmark protocol, the adapter advances it once per
control tick with no MPPI base command, obtains that same tick's diagnostic
`last_debug["fly_command"]` weighted readout, then applies the bridge's shared
smoothstep exactly once. If that diagnostic is absent or invalid, the bridge
fails closed to MPPI. This compatibility dependency is logged as a limitation
until V1 exposes a public raw-readout method.

V1's default `nominal_speed=0.2` is a cruise policy. The bridge sets
`brain.nominal_speed=0.0`, so Group C/D contributes only its threat-response
bias; adding the cruise term only to the neural groups would make the heuristic
comparison invalid.

V1 is right-positive for lateral and yaw avoidance while the XXU ROS base frame
is REP-103 (`+y` left, `+z` left turn). The adapter applies
`brain.ros_vy_sign=-1` and `brain.ros_wz_sign=-1` exactly once before fusion.
The measured shadow check is left threat → negative ROS `vy`/`wz`, and right
threat → positive ROS `vy`/`wz`.
