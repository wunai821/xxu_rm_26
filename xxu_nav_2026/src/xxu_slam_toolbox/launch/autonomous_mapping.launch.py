"""Launch SLAM Toolbox, Nav2, and explore_lite for autonomous mapping."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("xxu_slam_toolbox")
    bringup_share = FindPackageShare("xxu_bringup")

    default_slam_params_file = PathJoinSubstitution(
        [pkg_share, "config", "slam_toolbox_mapping.yaml"]
    )
    default_nav2_params_file = PathJoinSubstitution(
        [bringup_share, "config", "nav2_exploration.yaml"]
    )
    default_explore_params_file = PathJoinSubstitution(
        [pkg_share, "config", "explore_lite.yaml"]
    )
    default_rviz_config_file = PathJoinSubstitution(
        [pkg_share, "rviz", "autonomous_mapping.rviz"]
    )
    default_map_save_path = os.path.join(
        os.getcwd(), "src", "xxu_bringup", "maps", "auto_map"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    explore_params_file = LaunchConfiguration("explore_params_file")
    enable_explore = LaunchConfiguration("enable_explore")
    auto_save_map = LaunchConfiguration("auto_save_map")
    map_save_path = LaunchConfiguration("map_save_path")
    use_fake_frame = LaunchConfiguration("use_fake_frame")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    nav2_base_frame = PythonExpression([
        "'gimbal_yaw_fake' if '",
        use_fake_frame,
        "' == 'true' else 'base_link'",
    ])
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key="",
        param_rewrites={
            # The launch argument is the single time source for every Nav2
            # node, including nested local/global costmaps.
            "use_sim_time": use_sim_time,
            "bt_navigator.ros__parameters.robot_base_frame": nav2_base_frame,
            "local_costmap.local_costmap.ros__parameters.robot_base_frame": nav2_base_frame,
            "global_costmap.global_costmap.ros__parameters.robot_base_frame": nav2_base_frame,
            "behavior_server.ros__parameters.robot_base_frame": nav2_base_frame,
            "collision_monitor.ros__parameters.base_frame_id": nav2_base_frame,
        },
        convert_types=True,
    )

    # The fake frame is fixed in odom: MPPI controls XY only. Physical chassis
    # spin is owned by fake_vel_transform and is absent from /odom_nav.
    gyro_params = RewrittenYaml(
        source_file=configured_nav2_params,
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
        "' == 'true' else '", configured_nav2_params, "'",
    ])

    configured_explore_params = RewrittenYaml(
        source_file=explore_params_file,
        root_key="",
        param_rewrites={
            "/**.ros__parameters.robot_base_frame": nav2_base_frame,
        },
        convert_types=True,
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )
    declare_slam_params_file = DeclareLaunchArgument(
        "slam_params_file",
        default_value=default_slam_params_file,
        description="Full path to the SLAM Toolbox mapping parameters file",
    )
    declare_nav2_params_file = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=default_nav2_params_file,
        description="Full path to the Nav2 exploration parameters file",
    )
    declare_explore_params_file = DeclareLaunchArgument(
        "explore_params_file",
        default_value=default_explore_params_file,
        description="Full path to the explore_lite parameters file",
    )
    declare_enable_explore = DeclareLaunchArgument(
        "enable_explore",
        default_value="true",
        description="Start explore_lite frontier goal generation",
    )
    declare_auto_save_map = DeclareLaunchArgument(
        "auto_save_map",
        default_value="true",
        description="Save the current map when this launch shuts down",
    )
    declare_map_save_path = DeclareLaunchArgument(
        "map_save_path",
        default_value=default_map_save_path,
        description="Output path without .yaml/.pgm suffix for automatic map saving",
    )
    declare_use_fake_frame = DeclareLaunchArgument(
        "use_fake_frame",
        default_value="true",
        description="Use gimbal_yaw_fake as the Nav2 robot base frame",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Start RViz with the autonomous mapping display configuration",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config_file,
        description="Full path to the RViz config file",
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            ])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
            "autostart": "true",
        }.items(),
    )

    nav2_nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            output="screen",
            parameters=[selected_nav_params],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[selected_nav_params],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[selected_nav_params],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[selected_nav_params],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[selected_nav_params],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[selected_nav_params],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            parameters=[selected_nav_params],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "velocity_smoother",
                    "collision_monitor",
                ],
            }],
        ),
    ]

    explore_lite = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        output="screen",
        condition=IfCondition(enable_explore),
        parameters=[configured_explore_params, {"use_sim_time": use_sim_time}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_autonomous_mapping",
        output="screen",
        condition=IfCondition(rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    map_autosaver = Node(
        package="xxu_slam_toolbox",
        executable="map_autosaver.py",
        name="map_autosaver",
        output="screen",
        condition=IfCondition(auto_save_map),
        parameters=[{
            "map_path": map_save_path,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        declare_nav2_params_file,
        declare_explore_params_file,
        declare_enable_explore,
        declare_auto_save_map,
        declare_map_save_path,
        declare_use_fake_frame,
        declare_rviz,
        declare_rviz_config,
        slam_toolbox,
        map_autosaver,
        TimerAction(
            period=8.0,
            actions=[
                *nav2_nodes,
                explore_lite,
            ],
        ),
        rviz_node,
    ])
