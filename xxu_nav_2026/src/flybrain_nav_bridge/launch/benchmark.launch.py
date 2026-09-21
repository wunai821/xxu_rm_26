"""Run one safely routed FlyBrain benchmark group in the existing Gazebo stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge_share = FindPackageShare("flybrain_nav_bridge")
    bringup_share = FindPackageShare("xxu_bringup")
    group = LaunchConfiguration("control_group")
    device = LaunchConfiguration("brain_device")
    rewired_data_dir = LaunchConfiguration("rewired_data_dir")
    real_data_dir = LaunchConfiguration("real_data_dir")
    brain_checksum_sha256 = LaunchConfiguration("brain_checksum_sha256")
    map_file = LaunchConfiguration("map")
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gazebo_gui")
    scenario = LaunchConfiguration("scenario")
    seed = LaunchConfiguration("seed")
    scenario_file = LaunchConfiguration("scenario_file")
    run_scenario = LaunchConfiguration("run_scenario")
    record_metrics = LaunchConfiguration("record_metrics")
    trial_timeout_s = LaunchConfiguration("trial_timeout_s")
    goal_success_guard_m = LaunchConfiguration("goal_success_guard_m")
    response_delta_threshold = LaunchConfiguration("response_delta_threshold")
    nav2_ready_delay_s = LaunchConfiguration("nav2_ready_delay_s")
    results_root = LaunchConfiguration("results_root")
    dispatch_goal = LaunchConfiguration("dispatch_goal")
    goal_frame = LaunchConfiguration("goal_frame")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    goal_yaw = LaunchConfiguration("goal_yaw")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    initial_pose_relocalize = LaunchConfiguration("initial_pose_relocalize")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    scan_self_filter_radius = LaunchConfiguration("scan_self_filter_radius")
    scan_min_height = LaunchConfiguration("scan_min_height")
    planner_tolerance = LaunchConfiguration("planner_tolerance")
    return LaunchDescription([
        DeclareLaunchArgument("control_group", default_value="mppi"),
        DeclareLaunchArgument("brain_device", default_value="auto"),
        DeclareLaunchArgument("rewired_data_dir", default_value=""),
        DeclareLaunchArgument("real_data_dir", default_value="/home/naiwu/fly-data"),
        DeclareLaunchArgument("brain_checksum_sha256", default_value=""),
        DeclareLaunchArgument("gazebo_gui", default_value="false"),
        DeclareLaunchArgument("run_scenario", default_value="true"),
        DeclareLaunchArgument("record_metrics", default_value="true"),
        DeclareLaunchArgument("scenario", default_value="static_front"),
        DeclareLaunchArgument("seed", default_value="1000"),
        DeclareLaunchArgument(
            "scenario_file",
            default_value=PathJoinSubstitution([bridge_share, "config", "benchmark_scenarios.yaml"]),
        ),
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
        DeclareLaunchArgument("spawn_x", default_value="1.75"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="3.14159"),
        DeclareLaunchArgument("initial_pose_relocalize", default_value="true"),
        DeclareLaunchArgument("initial_pose_x", default_value="0.02"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.03"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=PathJoinSubstitution([bringup_share, "config", "nav2_navigation.yaml"]),
        ),
        DeclareLaunchArgument("scan_self_filter_radius", default_value="0.42"),
        DeclareLaunchArgument("scan_min_height", default_value="0.30"),
        DeclareLaunchArgument("planner_tolerance", default_value="0.0"),
        DeclareLaunchArgument(
            "map",
            default_value=PathJoinSubstitution([bringup_share, "maps", "complex_map.yaml"]),
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare("xxu_description"), "worlds", "complex_mapping.sdf"
            ]),
        ),
        # The bridge has the single output publisher for /cmd_vel_fused. Nav2
        # continues to publish only /cmd_vel_nav; collision_monitor is retargeted
        # through its existing launch argument, leaving the watchdog/chassis path
        # unchanged.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([bridge_share, "launch", "flybrain_bridge.launch.py"])
            ]),
            launch_arguments={
                "mode": "active",
                "control_group": group,
                "input_cmd_topic": "/cmd_vel_smoothed",
                "output_cmd_topic": "/cmd_vel_fused",
                "brain_device": device,
                "rewired_data_dir": rewired_data_dir,
                "real_data_dir": real_data_dir,
                "brain_checksum_sha256": brain_checksum_sha256,
                "benchmark_scenario": scenario,
                "benchmark_seed": seed,
                "logging": "true",
                "logging_root": results_root,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([bringup_share, "launch", "simulation.launch.py"])
            ]),
            launch_arguments={
                "start_navigation": "true",
                "enable_localization": "true",
                "gazebo_gui": gui,
                "rviz": "false",
                "map": map_file,
                "world": world,
                "nav2_params_file": nav2_params_file,
                "spawn_x": spawn_x,
                "spawn_y": spawn_y,
                "spawn_yaw": spawn_yaw,
                "initial_pose_relocalize": initial_pose_relocalize,
                "initial_pose_x": initial_pose_x,
                "initial_pose_y": initial_pose_y,
                "initial_pose_yaw": initial_pose_yaw,
                "scan_self_filter_radius": scan_self_filter_radius,
                "scan_min_height": scan_min_height,
                "planner_tolerance": planner_tolerance,
                "cmd_vel_in_topic": "/cmd_vel_fused",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([bridge_share, "launch", "scenario.launch.py"])
            ]),
            condition=IfCondition(run_scenario),
            launch_arguments={
                "world_name": "complex_mapping",
                "scenario": scenario,
                "seed": seed,
                "scenario_file": scenario_file,
                "autostart": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([bridge_share, "launch", "metrics.launch.py"])
            ]),
            condition=IfCondition(record_metrics),
            launch_arguments={
                "scenario": scenario,
                "controller": group,
                "seed": seed,
                "trial_timeout_s": trial_timeout_s,
                "goal_success_guard_m": goal_success_guard_m,
                "response_delta_threshold": response_delta_threshold,
                "nav2_ready_delay_s": nav2_ready_delay_s,
                "results_root": results_root,
                "dispatch_goal": dispatch_goal,
                "goal_frame": goal_frame,
                "goal_x": goal_x,
                "goal_y": goal_y,
                "goal_yaw": goal_yaw,
            }.items(),
        ),
    ])
