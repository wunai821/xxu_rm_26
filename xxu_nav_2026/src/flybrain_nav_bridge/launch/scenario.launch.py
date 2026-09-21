"""Expose Gazebo entity services and run a configured benchmark obstacle timeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("flybrain_nav_bridge")
    world_name = LaunchConfiguration("world_name")
    return LaunchDescription([
        DeclareLaunchArgument("world_name", default_value="complex_mapping"),
        DeclareLaunchArgument("scenario", default_value="static_front"),
        DeclareLaunchArgument("seed", default_value="1000"),
        DeclareLaunchArgument(
            "scenario_file",
            default_value=PathJoinSubstitution([share, "config", "benchmark_scenarios.yaml"]),
        ),
        DeclareLaunchArgument("autostart", default_value="false"),
        # GZ service bridges are separate from the existing robot launch so
        # this package can be removed without changing the base simulation.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="flybrain_benchmark_gz_bridge",
            output="screen",
            arguments=[
                # Service bridges resolve their Gazebo request/response types
                # from the ROS service mapping; unlike topic bridges they do
                # not use a direction suffix.
                ["/world/", world_name, "/create@ros_gz_interfaces/srv/SpawnEntity"],
                ["/world/", world_name, "/set_pose@ros_gz_interfaces/srv/SetEntityPose"],
                "/benchmark/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
            ],
        ),
        Node(
            package="flybrain_nav_bridge",
            executable="scenario_manager",
            name="flybrain_scenario_manager",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "world_name": world_name,
                "scenario": LaunchConfiguration("scenario"),
                "seed": LaunchConfiguration("seed"),
                "scenario_file": LaunchConfiguration("scenario_file"),
                "obstacle_sdf": PathJoinSubstitution([share, "models", "flybrain_obstacle", "model.sdf"]),
                "autostart": LaunchConfiguration("autostart"),
            }],
        ),
    ])
