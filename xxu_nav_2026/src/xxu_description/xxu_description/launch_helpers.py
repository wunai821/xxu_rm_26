"""Construct shared launch actions without starting processes or changing settings."""

import json
from pathlib import Path

from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import yaml


def nav2_parameters(source_file, use_sim_time, use_fake_frame, bringup_share,
                    *, mode, extra_rewrites=None):
    """Resolve base YAML plus the explicitly selected fake-frame profile."""
    base_frame = PythonExpression([
        "'gimbal_yaw_fake' if '", use_fake_frame, "' == 'true' else 'base_link'",
    ])
    rewrites = {"use_sim_time": use_sim_time}
    for key in (
        "bt_navigator.ros__parameters.robot_base_frame",
        "local_costmap.local_costmap.ros__parameters.robot_base_frame",
        "global_costmap.global_costmap.ros__parameters.robot_base_frame",
        "behavior_server.ros__parameters.robot_base_frame",
        "collision_monitor.ros__parameters.base_frame_id",
    ):
        rewrites[key] = base_frame
    rewrites.update(extra_rewrites or {})
    configured = RewrittenYaml(source_file=source_file, root_key="",
                               param_rewrites=rewrites, convert_types=True)
    profile = yaml.safe_load((Path(bringup_share) / "config/nav2_fake_profile.yaml").read_text())
    fake_rewrites = dict(profile["common"])
    fake_rewrites.update(profile[mode])
    fake_rewrites = {key: value if isinstance(value, str) else json.dumps(value)
                     for key, value in fake_rewrites.items()}
    for key in (
        "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml",
        "bt_navigator.ros__parameters.default_nav_through_poses_bt_xml",
    ):
        fake_rewrites[key] = PathJoinSubstitution([bringup_share, "config", fake_rewrites[key]])
    fake_params = RewrittenYaml(source_file=configured, root_key="",
                                param_rewrites=fake_rewrites, convert_types=True)
    selected = PythonExpression([
        "'", fake_params, "' if '", use_fake_frame,
        "' == 'true' else '", configured, "'",
    ])
    return configured, selected, base_frame


def velocity_chain(*, use_sim_time, use_fake_frame, cmd_vel_topic, spin_speed,
                   velocity_window, joint_states_topic="/joint_states",
                   output_topic="/cmd_vel", watchdog_parameters=None):
    """Build the same velocity adapter and freshness gate for each platform."""
    fake = Node(
        package="fake_vel_transform", executable="fake_vel_transform_node",
        name="fake_vel_transform", output="screen", condition=IfCondition(use_fake_frame),
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_base_frame": "base_link", "fake_robot_base_frame": "gimbal_yaw_fake",
            "odom_topic": "/odom", "input_cmd_vel_topic": cmd_vel_topic,
            "output_cmd_vel_topic": "/cmd_vel_transformed", "spin_speed": spin_speed,
            "gyro_linear_threshold": 0.01, "odom_timeout": 0.5,
            "nav_velocity_smoothing_window": velocity_window,
        }],
    )
    parameters = {
        "use_sim_time": use_sim_time,
        "input_topic": PythonExpression([
            "'/cmd_vel_transformed' if '", use_fake_frame,
            "' == 'true' else '", cmd_vel_topic, "'",
        ]),
        "output_topic": output_topic, "output_frame_id": "base_link",
        "timeout": 0.3, "publish_rate": 100.0,
        "require_odom": True, "odom_topic": "/odom", "odom_timeout": 0.5,
        "require_scan": True, "scan_topic": "/scan", "scan_timeout": 0.5,
        "require_joint_states": True, "joint_states_topic": joint_states_topic,
        "joint_states_timeout": 0.5, "require_tf": True,
        "tf_target_frame": "odom", "tf_source_frame": "base_link", "tf_timeout": 0.5,
        "required_tf_pairs": ["odom->base_footprint", "base_link->gimbal_link"],
    }
    parameters.update(watchdog_parameters or {})
    watchdog = Node(package="xxu_description", executable="cmd_vel_watchdog.py",
                    name="cmd_vel_watchdog", output="screen", parameters=[parameters])
    return fake, watchdog
