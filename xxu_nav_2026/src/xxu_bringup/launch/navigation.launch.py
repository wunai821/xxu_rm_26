"""Start Nav2 localization and navigation for the XXU robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("xxu_bringup")
    gicp_share = FindPackageShare("xxu_gicp_localization")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    autostart = LaunchConfiguration("autostart")
    use_fake_frame = LaunchConfiguration("use_fake_frame")
    enable_localization = LaunchConfiguration("enable_localization")
    enable_gicp = LaunchConfiguration("enable_gicp")
    gicp_pcd_map = LaunchConfiguration("gicp_pcd_map")
    cmd_vel_in_topic = LaunchConfiguration("cmd_vel_in_topic")
    cmd_vel_out_topic = LaunchConfiguration("cmd_vel_out_topic")
    planner_tolerance = LaunchConfiguration("planner_tolerance")

    # Keep standalone Nav2 aligned with the complex_mapping Gazebo world.
    default_map = PathJoinSubstitution([bringup_share, "maps", "complex_map.yaml"])
    default_params = PathJoinSubstitution([bringup_share, "config", "nav2_navigation.yaml"])
    default_rviz_config = PathJoinSubstitution([bringup_share, "rviz", "navigation.rviz"])
    default_gicp_params = PathJoinSubstitution(
        [gicp_share, "config", "gicp_localization.yaml"]
    )

    localization_lifecycle_nodes = [
        "map_server",
        "amcl",
    ]
    navigation_lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
        "collision_monitor",
    ]

    nav2_common_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    nav2_base_frame = PythonExpression([
        "'gimbal_yaw_fake' if '",
        use_fake_frame,
        "' == 'true' else 'base_link'",
    ])
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            # The YAML files keep usable defaults for standalone launches, but
            # the launch argument is the single source of truth.  A leaf-key
            # rewrite updates every Nav2 node, including nested costmaps.
            "use_sim_time": use_sim_time,
            "bt_navigator.ros__parameters.robot_base_frame": nav2_base_frame,
            "local_costmap.local_costmap.ros__parameters.robot_base_frame": nav2_base_frame,
            "global_costmap.global_costmap.ros__parameters.robot_base_frame": nav2_base_frame,
            "behavior_server.ros__parameters.robot_base_frame": nav2_base_frame,
            "collision_monitor.ros__parameters.base_frame_id": nav2_base_frame,
            "collision_monitor.ros__parameters.cmd_vel_in_topic": cmd_vel_in_topic,
            "collision_monitor.ros__parameters.cmd_vel_out_topic": cmd_vel_out_topic,
            # Benchmark callers may require the exact requested endpoint.
            # Standalone navigation retains the configuration-file default.
            "planner_server.ros__parameters.GridBased.tolerance": planner_tolerance,
        },
        convert_types=True,
    )

    # The fake frame is fixed in odom: MPPI controls XY only. Physical chassis
    # spin is owned by fake_vel_transform and is absent from /odom_nav.
    gyro_params = RewrittenYaml(
        source_file=configured_params,
        root_key="",
        param_rewrites={
            "controller_server.ros__parameters.odom_topic": "/odom_nav",
            "bt_navigator.ros__parameters.odom_topic": "/odom_nav",
            "velocity_smoother.ros__parameters.odom_topic": "/odom_nav",
            "controller_server.ros__parameters.goal_checker.plugin": "nav2_controller::PositionGoalChecker",
            "controller_server.ros__parameters.FollowPath.wz_max": "0.0",
            "controller_server.ros__parameters.FollowPath.az_max": "0.0",
            # With yaw decoupled, retain enough goal attraction to overcome
            # the asymmetric acceleration/deceleration sampling bias.
            "controller_server.ros__parameters.FollowPath.GoalCritic.cost_weight": "15.0",
            # Circular swept footprint includes the protruding omni wheels.
            "local_costmap.local_costmap.ros__parameters.robot_radius": "0.42",
            "global_costmap.global_costmap.ros__parameters.robot_radius": "0.42",
            "collision_monitor.ros__parameters.EmergencyStop.points": "[[0.45, 0.0], [0.3182, 0.3182], [0.0, 0.45], [-0.3182, 0.3182], [-0.45, 0.0], [-0.3182, -0.3182], [0.0, -0.45], [0.3182, -0.3182]]",
            # Keep nonzero sampling variance: MPPI uses its inverse in costs.
            "controller_server.ros__parameters.FollowPath.GoalAngleCritic.enabled": "false",
            "controller_server.ros__parameters.FollowPath.PathAngleCritic.enabled": "false",
            "controller_server.ros__parameters.FollowPath.TwirlingCritic.enabled": "false",
            "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml": PathJoinSubstitution(
                [bringup_share, "config", "navigate_to_pose_translation.xml"]),
            "bt_navigator.ros__parameters.default_nav_through_poses_bt_xml": PathJoinSubstitution(
                [bringup_share, "config", "navigate_through_poses_translation.xml"]),
        },
        convert_types=True,
    )
    selected_nav_params = PythonExpression([
        "'", gyro_params, "' if '", use_fake_frame,
        "' == 'true' else '", configured_params, "'",
    ])


    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_localization",
            default_value="false",
            description="Re-enable paused AMCL/GICP localization and map navigation",
        ),
        LogInfo(msg="AMCL/GICP and map navigation require enable_localization:=true (temporarily disabled by default)."),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Full path to the map yaml file",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Full path to the Nav2 parameters file",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Start RViz",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="Full path to the RViz navigation config file",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically configure and activate lifecycle nodes",
        ),
        DeclareLaunchArgument(
            "use_fake_frame",
            default_value="true",
            description="Use gimbal_yaw_fake as the Nav2 robot base frame",
        ),
        DeclareLaunchArgument(
            "enable_gicp",
            default_value="false",
            description="Use AMCL only for initialization and small_gicp for map->odom",
        ),
        DeclareLaunchArgument(
            "gicp_pcd_map",
            default_value="",
            description="Map-frame PCD consumed by xxu_gicp_localization",
        ),
        DeclareLaunchArgument(
            "cmd_vel_in_topic",
            default_value="cmd_vel_smoothed",
            description="Velocity input consumed by collision_monitor",
        ),
        DeclareLaunchArgument(
            "cmd_vel_out_topic",
            default_value="/cmd_vel_collision",
            description="Collision-monitored velocity output",
        ),
        DeclareLaunchArgument(
            "planner_tolerance",
            default_value="1.0",
            description="Navfn endpoint tolerance in metres",
        ),
        GroupAction(
            condition=IfCondition(enable_localization),
            actions=[
                Node(
                    package="nav2_map_server",
                    executable="map_server",
                    name="map_server",
                    output="screen",
                    parameters=[configured_params, {"yaml_filename": map_file, "use_sim_time": use_sim_time}],
                ),
                Node(
                    package="nav2_amcl",
                    executable="amcl",
                    name="amcl",
                    output="both",
                    # AMCL remains the coarse initializer/relocalizer, but must not
                    # publish map->odom while GICP owns that transform.
                    parameters=[configured_params, {
                        "tf_broadcast": ParameterValue(PythonExpression([
                            "'false' if '", enable_gicp, "' == 'true' else 'true'",
                        ]), value_type=bool),
                    }],
                    remappings=nav2_common_remaps,
                ),
                Node(
                    package="xxu_gicp_localization",
                    executable="gicp_localization_node",
                    name="gicp_localization",
                    output="both",
                    condition=IfCondition(enable_gicp),
                    parameters=[default_gicp_params, {
                        "use_sim_time": use_sim_time,
                        "pcd_map": gicp_pcd_map,
                    }],
                ),
                Node(
                    package="nav2_controller",
                    executable="controller_server",
                    output="screen",
                    parameters=[selected_nav_params],
                    remappings=nav2_common_remaps + [("cmd_vel", "cmd_vel_nav")],
                ),
                Node(
                    package="nav2_smoother",
                    executable="smoother_server",
                    name="smoother_server",
                    output="screen",
                    parameters=[selected_nav_params],
                    remappings=nav2_common_remaps,
                ),
                Node(
                    package="nav2_planner",
                    executable="planner_server",
                    name="planner_server",
                    output="screen",
                    parameters=[selected_nav_params],
                    remappings=nav2_common_remaps,
                ),
                Node(
                    package="nav2_behaviors",
                    executable="behavior_server",
                    name="behavior_server",
                    output="screen",
                    parameters=[selected_nav_params],
                    remappings=nav2_common_remaps + [("cmd_vel", "cmd_vel_nav")],
                ),
                Node(
                    package="nav2_bt_navigator",
                    executable="bt_navigator",
                    name="bt_navigator",
                    output="screen",
                    parameters=[selected_nav_params],
                    remappings=nav2_common_remaps,
                ),
                Node(
                    package="nav2_waypoint_follower",
                    executable="waypoint_follower",
                    name="waypoint_follower",
                    output="screen",
                    parameters=[selected_nav_params],
                    remappings=nav2_common_remaps,
                ),
                Node(
                    package="nav2_velocity_smoother",
                    executable="velocity_smoother",
                    name="velocity_smoother",
                    output="screen",
                    parameters=[selected_nav_params],
                    remappings=nav2_common_remaps + [("cmd_vel", "cmd_vel_nav")],
                ),
                Node(
                    package="nav2_collision_monitor",
                    executable="collision_monitor",
                    name="collision_monitor",
                    output="screen",
                    parameters=[selected_nav_params],
                    remappings=nav2_common_remaps,
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_localization",
                    output="screen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": localization_lifecycle_nodes,
                    }],
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_navigation",
                    output="screen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        # Planner activation needs AMCL's map -> odom transform.
                        # nav2_navigation_startup starts this manager once it exists.
                        "autostart": False,
                        "node_names": navigation_lifecycle_nodes,
                    }],
                ),
                Node(
                    package="xxu_bringup",
                    executable="nav2_navigation_startup.py",
                    name="nav2_navigation_startup",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                ),
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2_navigation",
                    output="screen",
                    condition=IfCondition(rviz),
                    arguments=["-d", rviz_config],
                    parameters=[{"use_sim_time": use_sim_time}],
                ),
            ],
        ),
    ])
