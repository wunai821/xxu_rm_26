"""Run the real MID360 gimbal compensation chain and Small Point-LIO."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    model = LaunchConfiguration("model")
    config_file = LaunchConfiguration("config_file")
    lidar_topic = LaunchConfiguration("lidar_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    joint_position_offset = LaunchConfiguration("joint_position_offset")
    input_acceleration_scale = LaunchConfiguration("input_acceleration_scale")
    start_robot_state_publisher = LaunchConfiguration(
        "start_robot_state_publisher"
    )

    default_model = PathJoinSubstitution(
        [FindPackageShare("xxu_description"), "urdf", "xxu.urdf.xacro"]
    )
    default_config = PathJoinSubstitution(
        [
            FindPackageShare("small_point_lio"),
            "config",
            "xxu_mid360_gimbal_compensated.yaml",
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        condition=IfCondition(start_robot_state_publisher),
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", model]), value_type=str
            ),
            "use_sim_time": use_sim_time,
        }],
        remappings=[("joint_states", joint_states_topic)],
    )

    pointcloud_motion_compensator = Node(
        package="xxu_description",
        executable="pointcloud_frame_republisher.py",
        name="pointcloud_motion_compensator",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": "lio_base_sensor",
            "expected_source_frame": "livox_frame",
            "use_point_timestamps": True,
            "use_joint_interpolation": True,
            "joint_name": "gimbal_joint",
            "joint_axis": [0.0, 0.0, 1.0],
            "sensor_offset": [0.0, 0.08637, 0.0],
            "sensor_rpy": [-0.2967059728, 0.0, 0.0],
            "joint_position_offset": ParameterValue(
                joint_position_offset, value_type=float
            ),
            "max_joint_sample_gap": 0.05,
            "time_reset_threshold": 0.5,
            "pending_timeout": 0.25,
            "pending_queue_size": 3,
        }],
        remappings=[
            ("points_in", lidar_topic),
            ("points_out", "/livox/lidar_compensated"),
            ("joint_states", joint_states_topic),
        ],
    )

    gimbal_imu_compensator = Node(
        package="xxu_description",
        executable="gimbal_imu_compensator.py",
        name="gimbal_imu_compensator",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": "lio_base_sensor",
            "joint_name": "gimbal_joint",
            "joint_axis": [0.0, 0.0, 1.0],
            # Gimbal-axis to the internal IMU origin. This combines the
            # mechanical MID360 mount and the calibrated lidar-to-IMU offset.
            "sensor_offset": [0.011, 0.0957429, -0.0490015],
            "sensor_rpy": [-0.2967059728, 0.0, 0.0],
            "joint_position_offset": ParameterValue(
                joint_position_offset, value_type=float
            ),
            # livox_ros_driver2 exposes the MID360 accelerometer in g.
            "input_acceleration_scale": ParameterValue(
                input_acceleration_scale, value_type=float
            ),
            "angular_acceleration_time_constant": 0.05,
            "joint_acceleration_time_constant": 0.05,
            "max_joint_acceleration": 100.0,
            "max_joint_sample_gap": 0.05,
            "time_reset_threshold": 0.5,
        }],
        remappings=[
            ("imu_in", imu_topic),
            ("imu_out", "/livox/imu_lio_compensated"),
            ("joint_states", joint_states_topic),
        ],
    )

    small_point_lio = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        parameters=[config_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the ROS simulation clock when true",
        ),
        DeclareLaunchArgument(
            "model",
            default_value=default_model,
            description="Real robot URDF/Xacro path",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Small Point-LIO compensated parameter file",
        ),
        DeclareLaunchArgument(
            "lidar_topic",
            default_value="/livox/lidar",
            description="Raw livox_ros_driver2 PointCloud2 topic",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/livox/imu",
            description="Raw livox_ros_driver2 IMU topic (acceleration in g)",
        ),
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="/joint_states",
            description="Timestamped gimbal joint-state topic",
        ),
        DeclareLaunchArgument(
            "joint_position_offset",
            default_value="0.0",
            description="Encoder-to-mechanical gimbal zero offset in radians",
        ),
        DeclareLaunchArgument(
            "input_acceleration_scale",
            default_value="9.81",
            description="Raw IMU acceleration multiplier; official Livox uses g",
        ),
        DeclareLaunchArgument(
            "start_robot_state_publisher",
            default_value="true",
            description="Publish the real robot TF tree from the included Xacro",
        ),
        robot_state_publisher,
        pointcloud_motion_compensator,
        gimbal_imu_compensator,
        small_point_lio,
    ])
