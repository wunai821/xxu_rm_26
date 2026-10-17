# Maps

Store Nav2 map files here.

`rmuc_2025.yaml` / `rmuc_2025.pgm` belong to the pinned rm_sim_26 field.
They are rasterized from collision geometry, including low barriers; they are
not a measured SLAM map. The upstream version is pinned in
[simulation.repos](../../../simulation.repos); regenerate with
[generate_rmuc_map.py](../../../tools/navigation/generate_rmuc_map.py).
Use `ros2 launch xxu_bringup rm_sim_navigation.launch.py` for this field.
The examples below retain the old XXU field entry points.

Use `nav2_map_server` to save a map:

```bash
ros2 run nav2_map_server map_saver_cli -f src/xxu_bringup/maps/your_map
```

Start navigation with the map corresponding to the `complex_mapping.sdf` world:

```bash
ros2 launch xxu_bringup navigation.launch.py
```

Start simulation, load the matching `complex_map.yaml`, and use RViz to set the
initial pose and send a single Nav2 goal:

```bash
ros2 launch xxu_bringup single_point_simulation.launch.py rviz:=true
```

If Gazebo and the robot are already running, start only Nav2 and RViz:

```bash
ros2 launch xxu_bringup navigation.launch.py rviz:=true
```
