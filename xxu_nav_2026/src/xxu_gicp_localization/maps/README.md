# GICP maps

`xxu_gicp_localization` expects a PCD in the `map` frame. A real robot PCD
must be made from a surveyed/stationary scan and kept separate from the
simulation fixture below; do not use the fixture as a physical calibration
map.

To create a repeatable test map corresponding to the collision geometry in
`xxu_description/worlds/complex_mapping.sdf`:

```bash
ros2 run xxu_gicp_localization generate_complex_mapping_pcd.py \
  --sdf $(ros2 pkg prefix xxu_description)/share/xxu_description/worlds/complex_mapping.sdf \
  --output /tmp/complex_mapping_sim.pcd --spacing 0.10
```

Then launch simulation with:

```bash
ros2 launch xxu_bringup simulation.launch.py \
  start_navigation:=true enable_gicp:=true \
  gicp_pcd_map:=/tmp/complex_mapping_sim.pcd
```

The generator parses every box and cylinder collision surface from the selected
SDF, including model/collision poses and yaw. By default it also applies the
simulation spawn inverse transform (`spawn_x=1.75`, `spawn_y=0`,
`spawn_yaw=pi`), so the result is in the Nav2 `map` frame rather than the
Gazebo world frame. It is intentionally labelled a simulation map and is not
installed as a default real-robot map. For a different spawn, pass the three
`--spawn-*` options; a real-robot PCD should not use this conversion.
