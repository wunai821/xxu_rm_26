# pb_nav2_plugins

`pb_nav2_plugins` provides extra Navigation2 plugins ported into this workspace and adapted for ROS 2 Jazzy / Nav2 1.3.

## Plugins

### BackUpFreeSpace

Behavior plugin: `pb_nav2_behaviors::BackUpFreeSpace`

This replaces Nav2's default `backup` behavior with a fallback behavior that queries a costmap, searches for the widest nearby free sector, and drives the robot toward that free space for the requested BackUp action distance. The command is published in the configured `robot_base_frame`, so it works with both `base_link` and the project's `base_link_fake` mode.

Supported parameters are declared on `behavior_server.ros__parameters`:

- `max_radius` (`double`, default: `1.0`) - Radius used to test free-space directions.
- `service_name` (`string`, default: `local_costmap/get_costmap`) - `nav2_msgs/srv/GetCostmap` service used for the free-space search.
- `visualize` (`bool`, default: `false`) - Publish RViz markers on `back_up_free_space_markers`.

XXU uses it as the Nav2 recovery/fallback `backup` behavior:

```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "pb_nav2_behaviors::BackUpFreeSpace"
    wait:
      plugin: "nav2_behaviors::Wait"
    max_radius: 1.0
    service_name: "local_costmap/get_costmap"
    visualize: false
```

The older lookup name `pb_nav2_behaviors/BackUpFreeSpace` is also exported as a compatibility alias.

### IntensityVoxelLayer

Costmap layer plugin: `pb_nav2_costmap_2d::IntensityVoxelLayer`

This is a voxel obstacle layer that consumes `PointCloud2` observations and only marks points whose `intensity` field is within the configured range.

Additional parameters beyond the usual Nav2 voxel/obstacle layer settings:

- `min_obstacle_intensity` (`double`, default: `0.1`)
- `max_obstacle_intensity` (`double`, default: `2.0`)

Example:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["intensity_voxel_layer", "inflation_layer"]
      intensity_voxel_layer:
        plugin: "pb_nav2_costmap_2d::IntensityVoxelLayer"
        enabled: true
        observation_sources: terrain_map
        min_obstacle_intensity: 0.1
        max_obstacle_intensity: 2.0
        terrain_map:
          data_type: PointCloud2
          topic: /terrain_map
          marking: true
          clearing: false
          obstacle_max_range: 5.0
          obstacle_min_range: 0.2
```

## License

Apache-2.0. See `LICENSE` and `NOTICE`.
