from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    declare_config_file = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("small_point_lio"),
                "config",
                "xxu_gazebo_mid360.yaml",
            ]
        ),
        description="Small Point-LIO parameter file. Defaults to the Gazebo MID360 adapter.",
    )

    small_point_lio_node = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        parameters=[
            config_file,
        ],
    )

    return LaunchDescription([
        declare_config_file,
        small_point_lio_node,
    ])
