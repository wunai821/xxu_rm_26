"""Launch SLAM Toolbox, Nav2, and explore_lite for autonomous mapping."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("xxu_slam_toolbox")

    default_slam_params_file = PathJoinSubstitution(
        [pkg_share, "config", "slam_toolbox_mapping.yaml"]
    )
    default_nav2_params_file = PathJoinSubstitution(
        [pkg_share, "config", "nav2_exploration.yaml"]
    )
    default_explore_params_file = PathJoinSubstitution(
        [pkg_share, "config", "explore_lite.yaml"]
    )
    default_rviz_config_file = PathJoinSubstitution(
        [pkg_share, "rviz", "autonomous_mapping.rviz"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    explore_params_file = LaunchConfiguration("explore_params_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

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
            parameters=[nav2_params_file],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[nav2_params_file],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params_file],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params_file],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_params_file],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[nav2_params_file],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            parameters=[nav2_params_file],
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
        parameters=[explore_params_file, {"use_sim_time": use_sim_time}],
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

    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        declare_nav2_params_file,
        declare_explore_params_file,
        declare_rviz,
        declare_rviz_config,
        slam_toolbox,
        TimerAction(
            period=8.0,
            actions=[
                *nav2_nodes,
                explore_lite,
            ],
        ),
        rviz_node,
    ])
