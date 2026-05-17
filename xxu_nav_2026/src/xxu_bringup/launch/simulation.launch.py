"""Start the XXU Gazebo simulation and optionally the Nav2 stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("xxu_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_navigation = LaunchConfiguration("start_navigation")
    map_file = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    rviz = LaunchConfiguration("rviz")
    enable_lio = LaunchConfiguration("enable_lio")
    enable_cmd_vel_odom = LaunchConfiguration("enable_cmd_vel_odom")
    use_livox_native = LaunchConfiguration("use_livox_native")
    use_fake_frame = LaunchConfiguration("use_fake_frame")

    default_map = PathJoinSubstitution([bringup_share, "maps", "empty.yaml"])
    default_nav2_params = PathJoinSubstitution(
        [bringup_share, "config", "nav2_navigation.yaml"]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("xxu_description"),
                "launch",
                "gazebo.launch.py",
            ])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "enable_lio": enable_lio,
            "enable_cmd_vel_odom": enable_cmd_vel_odom,
            "use_livox_native": use_livox_native,
            "use_fake_frame": use_fake_frame,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, "launch", "navigation.launch.py"])
        ]),
        condition=IfCondition(start_navigation),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_file,
            "params_file": nav2_params_file,
            "rviz": rviz,
            "use_fake_frame": use_fake_frame,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),
        DeclareLaunchArgument(
            "start_navigation",
            default_value="false",
            description="Start Nav2 localization and navigation after Gazebo starts",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Full path to the map yaml used by Nav2 map_server",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=default_nav2_params,
            description="Full path to the Nav2 parameters file",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Start RViz from navigation.launch.py",
        ),
        DeclareLaunchArgument(
            "enable_lio",
            default_value="true",
            description="Start Small Point-LIO in the Gazebo launch",
        ),
        DeclareLaunchArgument(
            "enable_cmd_vel_odom",
            default_value="false",
            description="Publish planar odometry by integrating /cmd_vel",
        ),
        DeclareLaunchArgument(
            "use_livox_native",
            default_value="false",
            description="Use the xxu_livox_sim Gazebo System plugin",
        ),
        DeclareLaunchArgument(
            "use_fake_frame",
            default_value="false",
            description="Use base_link_fake and fake_vel_transform for Nav2",
        ),
        gazebo,
        TimerAction(period=8.0, actions=[navigation]),
    ])
