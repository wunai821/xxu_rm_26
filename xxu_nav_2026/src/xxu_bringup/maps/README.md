# Maps

Store Nav2 map files here.

Use `nav2_map_server` to save a map:

```bash
ros2 run nav2_map_server map_saver_cli -f src/xxu_bringup/maps/your_map
```

Start navigation with the saved `auto_map.yaml`:

```bash
ros2 launch xxu_bringup navigation.launch.py
```

Start simulation, load the saved `auto_map.yaml`, and use RViz to set the
initial pose and send a single Nav2 goal:

```bash
ros2 launch xxu_bringup single_point_simulation.launch.py rviz:=true
```

If Gazebo and the robot are already running, start only Nav2 and RViz:

```bash
ros2 launch xxu_bringup navigation.launch.py rviz:=true
```
