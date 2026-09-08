"""Run deterministic hardware-contract and fail-safe smoke tests.

The default test publishes a forward TwistStamped, inserts a 0.15 m scan
return after three seconds, and asserts that the wheel command sink observes
motion before the obstacle and only zero commands afterwards.  The drop_* and
stop_test_command_after arguments inject the same stale-stream conditions
that occur when a driver or an upstream node exits.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("xxu_bringup")
    description_share = FindPackageShare("xxu_description")
    collision_monitor_share = FindPackageShare("nav2_collision_monitor")

    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_collision_monitor = LaunchConfiguration("enable_collision_monitor")
    command_topic = PythonExpression([
        "'/cmd_vel_smoothed' if '",
        enable_collision_monitor,
        "' == 'true' else '/cmd_vel_collision'",
    ])
    wheel_command_topic = LaunchConfiguration("wheel_command_topic")
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    model = PathJoinSubstitution([description_share, "urdf", "xxu.urdf.xacro"])
    collision_config = PathJoinSubstitution(
        [bringup_share, "config", "collision_monitor_validation.yaml"]
    )
    collision_launch = PathJoinSubstitution(
        [
            collision_monitor_share,
            "launch",
            "collision_monitor_node.launch.py",
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_validation",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", model]), value_type=str
            ),
            "use_sim_time": use_sim_time,
        }],
        remappings=[("joint_states", joint_states_topic)],
    )

    mock_io = Node(
        package="xxu_bringup",
        executable="mock_robot_io.py",
        name="mock_robot_io",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "command_topic": command_topic,
            "command_frame_id": "base_link",
            "scan_frame_id": "base_link",
            "publish_test_command": True,
            "joint_states_topic": joint_states_topic,
            "wheel_command_topic": wheel_command_topic,
            "obstacle_range": LaunchConfiguration("obstacle_range"),
            "obstacle_after": LaunchConfiguration("obstacle_after"),
            "drop_odom_after": LaunchConfiguration("drop_odom_after"),
            "drop_scan_after": LaunchConfiguration("drop_scan_after"),
            "drop_joint_states_after": LaunchConfiguration("drop_joint_states_after"),
            "drop_tf_after": LaunchConfiguration("drop_tf_after"),
            "stop_test_command_after": LaunchConfiguration("stop_test_command_after"),
            "test_duration": LaunchConfiguration("test_duration"),
            "failure_grace": LaunchConfiguration("failure_grace"),
            "assert_safety": True,
        }],
    )

    collision_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(collision_launch),
        condition=IfCondition(enable_collision_monitor),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": collision_config,
            # The upstream launch uses PythonExpression(['not ', value]), so
            # use a Python boolean spelling here rather than lower-case YAML.
            "use_composition": "False",
        }.items(),
    )

    watchdog = Node(
        package="xxu_description",
        executable="cmd_vel_watchdog.py",
        name="cmd_vel_watchdog",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": "/cmd_vel_collision",
            "output_topic": "/cmd_vel",
            "status_topic": "/cmd_vel_watchdog/healthy",
            "timeout": 0.3,
            "publish_rate": 20.0,
            "output_frame_id": "base_link",
            "require_odom": True,
            "odom_topic": "/odom",
            "odom_timeout": 0.5,
            "require_scan": True,
            "scan_topic": "/scan",
            "scan_timeout": 0.5,
            "require_joint_states": True,
            "joint_states_topic": joint_states_topic,
            "joint_states_timeout": 0.5,
            "require_tf": True,
            "tf_topic": "/tf",
            "tf_target_frame": "odom",
            "tf_source_frame": "base_link",
            "tf_timeout": 0.5,
            "required_tf_pairs": [
                "odom->base_footprint",
                "base_link->gimbal_link",
            ],
        }],
    )

    chassis = Node(
        package="xxu_chassis_controller",
        executable="chassis_controller",
        name="chassis_controller",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "wheel_radius": 0.0762,
            "wheel_command_sign": -1.0,
            "wheel_x": [0.223275, 0.223275, -0.223275, -0.223275],
            "wheel_y": [0.223275, -0.223275, 0.223275, -0.223275],
            "drive_direction_angle": [-0.785398, -2.356194, 0.785398, 2.356194],
            "max_wheel_speed": 100.0,
            "timeout": 0.3,
            "publish_rate": 50.0,
            "cmd_vel_topic": "/cmd_vel",
            "wheel_command_topic": wheel_command_topic,
            "expected_frame_id": "base_link",
            "accept_empty_frame_id": True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "enable_collision_monitor",
            default_value="true",
            description="Run the real nav2_collision_monitor stop zone",
        ),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument(
            "wheel_command_topic",
            default_value="/wheel_velocity_controller/commands",
        ),
        DeclareLaunchArgument(
            "obstacle_range",
            default_value="-1.0",
            description="Obstacle range in metres; -1 means insert 0.15 m after obstacle_after",
        ),
        DeclareLaunchArgument(
            "obstacle_after",
            default_value="3.0",
            description="Inject the obstacle after this many wall-clock seconds; -1 disables it",
        ),
        DeclareLaunchArgument("drop_odom_after", default_value="-1.0"),
        DeclareLaunchArgument("drop_scan_after", default_value="-1.0"),
        DeclareLaunchArgument("drop_joint_states_after", default_value="-1.0"),
        DeclareLaunchArgument("drop_tf_after", default_value="-1.0"),
        DeclareLaunchArgument(
            "stop_test_command_after",
            default_value="-1.0",
            description="Stop the command source after this time to model an upstream node exit",
        ),
        DeclareLaunchArgument("test_duration", default_value="8.0"),
        DeclareLaunchArgument("failure_grace", default_value="1.0"),
        robot_state_publisher,
        mock_io,
        collision_monitor,
        watchdog,
        chassis,
    ])
