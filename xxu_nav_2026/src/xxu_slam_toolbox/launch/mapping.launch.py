"""Launch SLAM Toolbox mapping for the XXU robot."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    default_rviz_config_file = PathJoinSubstitution(
        [pkg_share, "rviz", "mapping.rviz"]
    )
    default_map_save_path = os.path.join(
        os.getcwd(), "src", "xxu_bringup", "maps", "auto_map"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    auto_save_map = LaunchConfiguration("auto_save_map")
    map_save_path = LaunchConfiguration("map_save_path")
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
    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Start RViz with the mapping display configuration",
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_mapping",
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
        declare_auto_save_map,
        declare_map_save_path,
        declare_rviz,
        declare_rviz_config,
        slam_toolbox,
        map_autosaver,
        rviz_node,
    ])
