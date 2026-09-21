"""Bring up the XXU hardware-facing navigation stack without Gazebo.

The hardware drivers remain replaceable: a chassis adapter consumes the
configured wheel-command topic, the Livox driver provides the raw radar
topics, and the gimbal driver provides a timestamped ``JointState``.  This
launch only owns the common perception, localization, Nav2, command
transformation, and final safety gate.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
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

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_navigation = LaunchConfiguration("start_navigation")
    enable_localization = LaunchConfiguration("enable_localization")
    map_file = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_fake_frame = LaunchConfiguration("use_fake_frame")
    gyro_spin_rate = LaunchConfiguration("gyro_spin_rate")
    enable_lio = LaunchConfiguration("enable_lio")
    enable_gicp = LaunchConfiguration("enable_gicp")
    gicp_pcd_map = LaunchConfiguration("gicp_pcd_map")
    lidar_topic = LaunchConfiguration("lidar_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    joint_position_offset = LaunchConfiguration("joint_position_offset")
    input_acceleration_scale = LaunchConfiguration("input_acceleration_scale")
    model = LaunchConfiguration("model")
    start_robot_state_publisher = LaunchConfiguration("start_robot_state_publisher")
    mock_hardware = LaunchConfiguration("mock_hardware")
    wheel_command_topic = LaunchConfiguration("wheel_command_topic")

    default_model = PathJoinSubstitution(
        [description_share, "urdf", "xxu.urdf.xacro"]
    )
    default_map = PathJoinSubstitution([bringup_share, "maps", "auto_map.yaml"])
    default_nav2_params = PathJoinSubstitution(
        [bringup_share, "config", "nav2_navigation.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [bringup_share, "rviz", "navigation.rviz"]
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
            "publish_frequency": 100.0,
        }],
        remappings=[("joint_states", joint_states_topic)],
    )

    lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([description_share, "launch", "mid360_lio.launch.py"])
        ]),
        condition=IfCondition(enable_lio),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "model": model,
            "config_file": PathJoinSubstitution([
                FindPackageShare("small_point_lio"),
                "config",
                "xxu_mid360_gimbal_compensated.yaml",
            ]),
            "lidar_topic": lidar_topic,
            "imu_topic": imu_topic,
            "joint_states_topic": joint_states_topic,
            "joint_position_offset": joint_position_offset,
            "input_acceleration_scale": input_acceleration_scale,
            "start_robot_state_publisher": "false",
        }.items(),
    )

    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_scan",
        output="screen",
        condition=IfCondition(enable_lio),
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": "base_footprint",
            "transform_tolerance": 0.10,
            "min_height": 0.05,
            "max_height": 0.50,
            "angle_min": -3.1415926,
            "angle_max": 3.1415926,
            "angle_increment": 0.0087266,
            "scan_time": 0.1,
            "range_min": 0.30,
            "range_max": 40.0,
            "use_inf": True,
            "inf_epsilon": 1.0,
            "scan_qos_reliability": "reliable",
            "publish_processed_cloud": True,
            "processed_cloud_frame": "base_footprint",
            "processed_cloud_project_to_2d": True,
            "processed_cloud_z_value": 0.0,
            "crop_min_x": -40.0,
            "crop_max_x": 40.0,
            "crop_min_y": -40.0,
            "crop_max_y": 40.0,
        }],
        remappings=[
            ("cloud_in", "/cloud_deskewed"),
            ("scan", "/scan"),
            ("processed_cloud", "/mid360/points_navigation"),
        ],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, "launch", "navigation.launch.py"])
        ]),
        condition=IfCondition(AndSubstitution(start_navigation, enable_localization)),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_file,
            "params_file": nav2_params_file,
            "rviz": rviz,
            "rviz_config": rviz_config,
            "use_fake_frame": use_fake_frame,
            "enable_gicp": enable_gicp,
            "enable_localization": enable_localization,
            "gicp_pcd_map": gicp_pcd_map,
            "cmd_vel_in_topic": "cmd_vel_smoothed",
            "cmd_vel_out_topic": "/cmd_vel_collision",
        }.items(),
    )

    fake_vel_transform = Node(
        package="fake_vel_transform",
        executable="fake_vel_transform_node",
        name="fake_vel_transform",
        output="screen",
        condition=IfCondition(use_fake_frame),
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_base_frame": "base_link",
            "fake_robot_base_frame": "gimbal_yaw_fake",
            "odom_topic": "/odom",
            "input_cmd_vel_topic": "/cmd_vel_collision",
            "output_cmd_vel_topic": "/cmd_vel_transformed",
            "spin_speed": ParameterValue(gyro_spin_rate, value_type=float),
            "gyro_linear_threshold": 0.01,
            "odom_timeout": 0.5,
        }],
    )

    watchdog_input = PythonExpression([
        "'/cmd_vel_transformed' if '",
        use_fake_frame,
        "' == 'true' else '/cmd_vel_collision'",
    ])
    cmd_vel_watchdog = Node(
        package="xxu_description",
        executable="cmd_vel_watchdog.py",
        name="cmd_vel_watchdog",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": watchdog_input,
            "output_topic": "/cmd_vel",
            "status_topic": "/cmd_vel_watchdog/healthy",
            "timeout": 0.3,
            "publish_rate": 100.0,
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

    chassis_controller = Node(
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

    initial_pose = Node(
        package="xxu_bringup",
        executable="auto_initial_pose.py",
        name="auto_initial_pose",
        output="both",
        condition=IfCondition(AndSubstitution(start_navigation, LaunchConfiguration("auto_initial_pose"))),
        parameters=[{
            "use_sim_time": use_sim_time,
            "relocalize": LaunchConfiguration("initial_pose_relocalize"),
            "map_yaml": map_file,
            "x": LaunchConfiguration("initial_pose_x"),
            "y": LaunchConfiguration("initial_pose_y"),
            "yaw": LaunchConfiguration("initial_pose_yaw"),
            "covariance_x": LaunchConfiguration("initial_pose_covariance_xy"),
            "covariance_y": LaunchConfiguration("initial_pose_covariance_xy"),
            "covariance_yaw": LaunchConfiguration("initial_pose_covariance_yaw"),
        }],
    )

    mock_io = Node(
        package="xxu_bringup",
        executable="mock_robot_io.py",
        name="mock_robot_io",
        output="screen",
        condition=IfCondition(mock_hardware),
        parameters=[{
            "use_sim_time": use_sim_time,
            "joint_states_topic": joint_states_topic,
            "wheel_command_topic": wheel_command_topic,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock when true; real robot default is wall time",
        ),
        DeclareLaunchArgument(
            "start_navigation",
            default_value="true",
            description="Start localization, Nav2, collision monitoring, and RViz option",
        ),
        DeclareLaunchArgument(
            "enable_localization",
            default_value="false",
            description="Re-enable paused AMCL/GICP localization and map navigation",
        ),
        DeclareLaunchArgument("map", default_value=default_map, description="Map YAML"),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=default_nav2_params,
            description="Nav2 parameter YAML",
        ),
        DeclareLaunchArgument("rviz", default_value="false", description="Start RViz"),
        DeclareLaunchArgument(
            "rviz_config", default_value=default_rviz_config, description="RViz config"
        ),
        DeclareLaunchArgument(
            "use_fake_frame",
            default_value="true",
            description="Use gimbal_yaw_fake as Nav2 frame and transform commands to base_link",
        ),
        DeclareLaunchArgument(
            "gyro_spin_rate",
            default_value="1.5",
            description="Upper-computer chassis spin while translating (rad/s); 0 disables",
        ),
        DeclareLaunchArgument(
            "enable_lio",
            default_value="true",
            description="Run the compensated MID-360 Small Point-LIO chain",
        ),
        DeclareLaunchArgument(
            "enable_gicp",
            default_value="false",
            description="Run GICP localization after AMCL initialization",
        ),
        DeclareLaunchArgument(
            "gicp_pcd_map", default_value="", description="Map-frame PCD for GICP"
        ),
        DeclareLaunchArgument(
            "model", default_value=default_model, description="Real robot URDF/Xacro"
        ),
        DeclareLaunchArgument(
            "lidar_topic",
            default_value="/livox/lidar",
            description="Raw Livox PointCloud2; frame livox_frame, x/y/z+tag+timestamp fields",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/livox/imu",
            description="Raw Livox Imu; frame livox_frame, acceleration in g",
        ),
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="/joint_states",
            description="Gimbal JointState containing gimbal_joint position and velocity",
        ),
        DeclareLaunchArgument(
            "joint_position_offset",
            default_value="0.0",
            description="Gimbal encoder-to-mechanical zero offset in radians",
        ),
        DeclareLaunchArgument(
            "input_acceleration_scale",
            default_value="9.81",
            description="Raw Livox acceleration scale (g to m/s^2)",
        ),
        DeclareLaunchArgument(
            "start_robot_state_publisher",
            default_value="true",
            description="Publish the URDF TF tree from real joint states",
        ),
        DeclareLaunchArgument(
            "wheel_command_topic",
            default_value="/wheel_velocity_controller/commands",
            description="Float64MultiArray [fl, fr, rl, rr] wheel speeds in rad/s",
        ),
        DeclareLaunchArgument(
            "mock_hardware",
            default_value="false",
            description="Start the deterministic interface mock; use with enable_lio=false",
        ),
        DeclareLaunchArgument(
            "auto_initial_pose",
            default_value="false",
            description="Scan-match the map and publish an AMCL initial pose",
        ),
        DeclareLaunchArgument("initial_pose_relocalize", default_value="true"),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_covariance_xy", default_value="1.0"),
        DeclareLaunchArgument("initial_pose_covariance_yaw", default_value="0.2741557"),
        robot_state_publisher,
        lio,
        pointcloud_to_scan,
        navigation,
        fake_vel_transform,
        cmd_vel_watchdog,
        chassis_controller,
        initial_pose,
        mock_io,
    ])
