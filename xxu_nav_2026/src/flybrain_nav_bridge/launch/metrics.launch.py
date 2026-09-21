"""Record one benchmark trial without altering its control or scenario nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="static_front"),
        DeclareLaunchArgument("controller", default_value="mppi"),
        DeclareLaunchArgument("seed", default_value="1000"),
        DeclareLaunchArgument("trial_timeout_s", default_value="30.0"),
        DeclareLaunchArgument("goal_success_guard_m", default_value="0.30"),
        DeclareLaunchArgument("response_delta_threshold", default_value="0.02"),
        DeclareLaunchArgument("nav2_ready_delay_s", default_value="1.0"),
        DeclareLaunchArgument("results_root", default_value="/home/naiwu/fly_brain/gazebo_results"),
        DeclareLaunchArgument("dispatch_goal", default_value="false"),
        DeclareLaunchArgument("goal_frame", default_value="map"),
        DeclareLaunchArgument("goal_x", default_value="0.0"),
        DeclareLaunchArgument("goal_y", default_value="0.0"),
        DeclareLaunchArgument("goal_yaw", default_value="0.0"),
        Node(
            package="flybrain_nav_bridge",
            executable="benchmark_metrics",
            name="flybrain_benchmark_metrics",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "scenario": LaunchConfiguration("scenario"),
                "controller": LaunchConfiguration("controller"),
                "seed": LaunchConfiguration("seed"),
                "trial_timeout_s": LaunchConfiguration("trial_timeout_s"),
                "goal_success_guard_m": LaunchConfiguration("goal_success_guard_m"),
                "response_delta_threshold": LaunchConfiguration("response_delta_threshold"),
                "nav2_ready_delay_s": LaunchConfiguration("nav2_ready_delay_s"),
                "results_root": LaunchConfiguration("results_root"),
                "dispatch_goal": LaunchConfiguration("dispatch_goal"),
                "goal_frame": LaunchConfiguration("goal_frame"),
                "goal_x": LaunchConfiguration("goal_x"),
                "goal_y": LaunchConfiguration("goal_y"),
                "goal_yaw": LaunchConfiguration("goal_yaw"),
            }],
        ),
    ])
