"""Start Nav2 localization and navigation for the XXU robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("xxu_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    autostart = LaunchConfiguration("autostart")
    use_fake_frame = LaunchConfiguration("use_fake_frame")

    default_map = PathJoinSubstitution([bringup_share, "maps", "auto_map.yaml"])
    default_params = PathJoinSubstitution([bringup_share, "config", "nav2_navigation.yaml"])
    default_rviz_config = PathJoinSubstitution([bringup_share, "rviz", "navigation.rviz"])

    lifecycle_nodes = [
        "map_server",
        "amcl",
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
        "'base_link_fake' if '",
        use_fake_frame,
        "' == 'true' else 'base_link'",
    ])
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            "bt_navigator.ros__parameters.robot_base_frame": nav2_base_frame,
            "local_costmap.local_costmap.ros__parameters.robot_base_frame": nav2_base_frame,
            "global_costmap.global_costmap.ros__parameters.robot_base_frame": nav2_base_frame,
            "behavior_server.ros__parameters.robot_base_frame": nav2_base_frame,
            "collision_monitor.ros__parameters.base_frame_id": nav2_base_frame,
        },
        convert_types=True,
    )

    return LaunchDescription([
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
            default_value="false",
            description="Use base_link_fake as the Nav2 robot base frame",
        ),
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
            output="screen",
            parameters=[configured_params],
            remappings=nav2_common_remaps,
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            output="screen",
            parameters=[configured_params],
            remappings=nav2_common_remaps + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[configured_params],
            remappings=nav2_common_remaps,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[configured_params],
            remappings=nav2_common_remaps,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[configured_params],
            remappings=nav2_common_remaps + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[configured_params],
            remappings=nav2_common_remaps,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[configured_params],
            remappings=nav2_common_remaps,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[configured_params],
            remappings=nav2_common_remaps + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            parameters=[configured_params],
            remappings=nav2_common_remaps,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": lifecycle_nodes,
            }],
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
    ])
