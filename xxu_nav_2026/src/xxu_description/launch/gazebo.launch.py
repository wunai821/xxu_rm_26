"""启动 Gazebo Harmonic 并加载 XXU 机器人模型."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("xxu_description")

    model_path = PathJoinSubstitution(
        [pkg_share, "urdf", "xxu_gazebo.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [pkg_share, "config", "xxu_ros2_control.yaml"]
    )
    nav2_tb3_models_path = PathJoinSubstitution(
        [FindPackageShare("nav2_minimal_tb3_sim"), "models"]
    )
    xxu_description_share_parent = PathJoinSubstitution(
        [FindPackagePrefix("xxu_description"), "share"]
    )
    compensate_gimbal_for_lio = LaunchConfiguration("compensate_gimbal_for_lio")
    small_point_lio_config = PathJoinSubstitution([
        FindPackageShare("small_point_lio"),
        "config",
        PythonExpression([
            "'xxu_gazebo_mid360_gimbal_compensated.yaml' if '",
            compensate_gimbal_for_lio,
            "' == 'true' else 'xxu_gazebo_mid360.yaml'",
        ]),
    ])
    mid360_scan_mode_csv = PathJoinSubstitution(
        [FindPackageShare("xxu_livox_sim"), "config", "mid360.csv"]
    )
    livox_plugin_path = PathJoinSubstitution(
        [FindPackagePrefix("xxu_livox_sim"), "lib", "xxu_livox_sim"]
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_lio = LaunchConfiguration("enable_lio")
    enable_cmd_vel_odom = LaunchConfiguration("enable_cmd_vel_odom")
    use_livox_native = LaunchConfiguration("use_livox_native")
    use_fake_frame = LaunchConfiguration("use_fake_frame")
    gimbal_mode = LaunchConfiguration("gimbal_mode")
    gimbal_spin_rate = LaunchConfiguration("gimbal_spin_rate")
    gimbal_spin_start_delay = LaunchConfiguration("gimbal_spin_start_delay")
    gyro_spin_rate = LaunchConfiguration("gyro_spin_rate")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world = LaunchConfiguration("world")

    gz_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        [
            xxu_description_share_parent,
            ":",
            nav2_tb3_models_path,
            ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH"),
        ],
    )
    gz_system_plugin_path = SetEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        [
            livox_plugin_path,
            ":",
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
        ],
    )
    rmw_implementation = SetEnvironmentVariable(
        "RMW_IMPLEMENTATION",
        "rmw_cyclonedds_cpp",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )
    declare_lio_motion_diagnostics = DeclareLaunchArgument(
        "lio_motion_diagnostics", default_value="false",
        description="Log LIO motion propagation and matching diagnostics once per simulation second",
    )
    declare_enable_lio = DeclareLaunchArgument(
        "enable_lio",
        default_value="true",
        description="Start Small Point-LIO and publish /odom",
    )
    declare_compensate_gimbal_for_lio = DeclareLaunchArgument(
        "compensate_gimbal_for_lio",
        default_value="true",
        description=(
            "Remove known gimbal motion from point cloud and IMU before LIO; "
            "false restores the raw mid360_link estimator baseline"
        ),
    )
    declare_enable_cmd_vel_odom = DeclareLaunchArgument(
        "enable_cmd_vel_odom",
        default_value="false",
        description="Publish simulation planar /odom by integrating /cmd_vel",
    )
    declare_use_livox_native = DeclareLaunchArgument(
        "use_livox_native",
        default_value="true",
        description="Use batch-raycast MID360 simulation with per-point timestamps",
    )
    declare_use_fake_frame = DeclareLaunchArgument(
        "use_fake_frame",
        default_value="true",
        description="Use fake_vel_transform and gimbal_yaw_fake for Nav2 velocity commands",
    )
    declare_gimbal_mode = DeclareLaunchArgument(
        "gimbal_mode",
        default_value="spin",
        description="Radar gimbal mode: spin or hold",
    )
    declare_gimbal_spin_rate = DeclareLaunchArgument(
        "gimbal_spin_rate",
        default_value="6.283185307179586",
        description="Continuous radar gimbal speed in rad/s when gimbal_mode=spin",
    )
    declare_gimbal_spin_start_delay = DeclareLaunchArgument(
        "gimbal_spin_start_delay",
        default_value="1.0",
        description="Seconds to hold the MID360 after the first LIO odometry output",
    )
    declare_gyro_spin_rate = DeclareLaunchArgument(
        "gyro_spin_rate",
        default_value="31.4159265359",
        description="Chassis gyro angular speed while translating (rad/s); 0 disables it",
    )
    declare_gazebo_gui = DeclareLaunchArgument(
        "gazebo_gui",
        default_value="true",
        description="Start the Gazebo Sim GUI; disable it to reduce CPU load during mapping",
    )
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([pkg_share, "worlds", "complex_mapping.sdf"]),
        description="Gazebo world SDF path",
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command([
                    "xacro ",
                    model_path,
                    " controllers_file:=",
                    controllers_file,
                    " use_livox_native:=",
                    use_livox_native,
                    " scan_mode_csv:=",
                    mid360_scan_mode_csv,
                ]),
                value_type=str,
            ),
            "use_sim_time": use_sim_time,
            # Joint states are already available at ~166 Hz.  Publish the
            # moving gimbal TF at 100 Hz so MID-360 point-time lookups do not
            # routinely extrapolate into the future.
            "publish_frequency": 100.0,
        }],
    )

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={
            # Server-only mode keeps the physics/sensors running while leaving
            # CPU for LIO, SLAM and Nav2. RViz remains available separately.
            "gz_args": [
                PythonExpression([
                    "'-r ' if '", gazebo_gui,
                    "' == 'true' else '-s -r '",
                ]),
                world,
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # 生成机器人
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description",
                   "-name", "xxu",
                   "-x", "1.75",
                   "-y", "0.0",
                   "-z", "0.05",
                   "-Y", "3.14159",
                   "-allow_renaming", "false"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    wheel_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_velocity_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    gimbal_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gimbal_velocity_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                wheel_velocity_controller_spawner,
                gimbal_velocity_controller_spawner,
            ],
        )
    )

    # 默认模拟实车搜索目标时的连续旋转，也可切回世界航向保持模式。
    # 动态关节状态由 robot_state_publisher 转换成 base_link -> gimbal_link TF，
    # 点云与 IMU 共用这条真实运动链。
    gimbal_stabilizer = Node(
        package="xxu_description",
        executable="gimbal_stabilizer.py",
        name="gimbal_stabilizer",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "mode": gimbal_mode,
            "spin_rate": gimbal_spin_rate,
            "spin_start_delay": gimbal_spin_start_delay,
            "command_topic": "/gimbal_velocity_controller/commands",
            "command_rate": 200.0,
            "wait_for_odom": enable_lio,
            "odom_topic": "/odom",
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
            "wheel_command_topic": "/wheel_velocity_controller/commands",
            "expected_frame_id": "base_link",
            "accept_empty_frame_id": True,
        }],
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
            # Stable velocity frame located at the chassis origin.  The real
            # base_link -> gimbal_link TF remains dynamic for LiDAR/LIO.
            "fake_robot_base_frame": "gimbal_yaw_fake",
            "odom_topic": "/odom",
            "input_cmd_vel_topic": "/cmd_vel_collision",
            "output_cmd_vel_topic": "/cmd_vel_transformed",
            "spin_speed": gyro_spin_rate,
            "gyro_linear_threshold": 0.01,
            "odom_timeout": 0.5,
        }],
    )
    # IMU bridge (GZ -> ROS 单向)
    bridge_imu = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_imu",
        arguments=[
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        output="screen",
        remappings=[
            ("/imu", "/imu_raw"),
        ],
    )

    imu_frame_republisher = Node(
        package="xxu_description",
        executable="imu_frame_republisher.py",
        name="imu_frame_republisher",
        output="screen",
        # 仿真实车 MID360 内置 IMU，数值和坐标系都随雷达运动。
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": "mid360_link",
        }],
        remappings=[
            ("imu_in", "/imu_raw"),
            ("imu_out", "/imu"),
        ],
    )

    # Fallback GPU lidar bridge. The native Livox plugin publishes its own
    # PointCloud2 and does not need this extra sensor stream.
    bridge_lidar = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_lidar",
        arguments=[
            "/mid360/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        output="screen",
        condition=UnlessCondition(use_livox_native),
        remappings=[
            ("/mid360/points", "/mid360/points_raw"),
        ],
    )

    bridge_livox_native = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_livox_native",
        arguments=[
            "/mid360/livox_points_gz@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        output="screen",
        condition=IfCondition(use_livox_native),
        remappings=[
            # The native Gazebo plugin already publishes mid360_link and
            # per-point timestamps; avoid a frame-id-only republisher hop.
            ("/mid360/livox_points_gz", "/mid360/livox_points"),
        ],
    )

    # Transform each acquisition-time group exactly once. In production the
    # result is expressed at the rigid gimbal-axis frame and shared by LIO and
    # navigation. The false branch keeps the raw-LIO rollback while navigation
    # still receives a deskewed cloud in base_footprint.
    pointcloud_motion_compensator = Node(
        package="xxu_description",
        executable="pointcloud_frame_republisher.py",
        name="pointcloud_motion_compensator",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": PythonExpression([
                "'lio_base_sensor' if '", compensate_gimbal_for_lio,
                "' == 'true' else 'base_footprint'",
            ]),
            "expected_source_frame": "mid360_link",
            "tf_lookup_timeout": 0.0,
            "use_point_timestamps": True,
            "use_joint_interpolation": ParameterValue(
                compensate_gimbal_for_lio, value_type=bool
            ),
            "joint_name": "gimbal_joint",
            "joint_axis": [0.0, 0.0, 1.0],
            "sensor_offset": [0.0, 0.08637, 0.0],
            "sensor_rpy": [-0.2967059728, 0.0, 0.0],
            "joint_position_offset": 0.0,
            "max_joint_sample_gap": 0.05,
            "time_reset_threshold": 0.5,
            "pending_timeout": 0.25,
            "pending_queue_size": 3,
        }],
        remappings=[
            ("points_in", "/mid360/livox_points"),
            ("points_out", "/mid360/livox_points_compensated"),
            ("joint_states", "/joint_states"),
        ],
    )

    gimbal_imu_compensator = Node(
        package="xxu_description",
        executable="gimbal_imu_compensator.py",
        name="gimbal_imu_compensator",
        output="screen",
        condition=IfCondition(PythonExpression([
            "'", enable_lio, "' == 'true' and '",
            compensate_gimbal_for_lio, "' == 'true'",
        ])),
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": "lio_base_sensor",
            "joint_name": "gimbal_joint",
            "joint_axis": [0.0, 0.0, 1.0],
            "sensor_offset": [0.0, 0.08637, 0.0],
            "sensor_rpy": [-0.2967059728, 0.0, 0.0],
            "joint_position_offset": 0.0,
            "input_acceleration_scale": 1.0,
            "angular_acceleration_time_constant": 0.05,
            "joint_acceleration_time_constant": 0.05,
            "max_joint_acceleration": 100.0,
            "max_joint_sample_gap": 0.05,
            "time_reset_threshold": 0.5,
        }],
        remappings=[
            ("imu_in", "/imu"),
            ("imu_out", "/imu_lio_compensated"),
            ("joint_states", "/joint_states"),
        ],
    )
    pointcloud_frame_republisher = Node(
        package="xxu_description",
        executable="pointcloud_frame_republisher.py",
        name="pointcloud_frame_republisher",
        output="screen",
        condition=UnlessCondition(use_livox_native),
        # 保留雷达原始坐标。LIO先估计 mid360_link 位姿，再利用动态云台
        # TF将输出换算为底盘位姿，与实车数据链一致。
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": "mid360_link",
            "tf_lookup_timeout": 0.1,
        }],
        remappings=[
            ("points_in", "/mid360/points_raw"),
            ("points_out", "/mid360/points_lio"),
        ],
    )

    # Gazebo Harmonic names a sensor attached to gimbal_link as
    # xxu/gimbal_link/mid360_lidar and does not preserve the SDF frame_id.
    # This is the real sensor mount transform from the URDF, exposed in TF so
    # pointcloud_frame_republisher can transform by message timestamp.
    gpu_lidar_sensor_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gpu_lidar_sensor_tf",
        output="screen",
        condition=UnlessCondition(use_livox_native),
        arguments=[
            "--x", "0.0",
            "--y", "0.08637",
            "--z", "0.0",
            "--roll", "-0.2967059728",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "gimbal_link",
            "--child-frame-id", "xxu/gimbal_link/mid360_lidar",
        ],
    )

    livox_pointcloud_shaper = Node(
        package="xxu_livox_sim",
        executable="livox_pointcloud_shaper",
        name="livox_pointcloud_shaper",
        output="screen",
        condition=UnlessCondition(use_livox_native),
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": "mid360_link",
            # Gazebo gpu_lidar publishes one instantaneous point-cloud frame.
            # Do not fabricate per-point times across 0.1 s: a fast gimbal would
            # make LIO deskew an already coherent frame and bend it by tens of degrees.
            "scan_period": 0.0,
            "max_points": 12000,
            "scan_mode_csv": mid360_scan_mode_csv,
        }],
        remappings=[
            ("points_in", "/mid360/points_lio"),
            ("points_out", "/mid360/livox_points"),
        ],
    )

    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": "base_footprint",
            "transform_tolerance": 0.05,
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
            # Navigation stays independent of LIO and shares the same strict
            # point-time compensated cloud in the production configuration.
            ("cloud_in", "/mid360/livox_points_compensated"),
            ("scan", "/scan"),
            ("processed_cloud", "/mid360/points_navigation"),
        ],
    )

    small_point_lio = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        condition=IfCondition(enable_lio),
        parameters=[small_point_lio_config, {
            "motion_diagnostics_en": ParameterValue(
                LaunchConfiguration("lio_motion_diagnostics"), value_type=bool),
        }],
    )

    # Clock bridge
    bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_clock",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    cmd_vel_watchdog = Node(
        package="xxu_description",
        executable="cmd_vel_watchdog.py",
        name="cmd_vel_watchdog",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_topic": PythonExpression([
                "'/cmd_vel_transformed' if '",
                use_fake_frame,
                "' == 'true' else '/cmd_vel_collision'",
            ]),
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
            "joint_states_topic": "/joint_states",
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

    cmd_vel_odometry = Node(
        package="xxu_description",
        executable="cmd_vel_odometry.py",
        name="cmd_vel_odometry",
        output="screen",
        condition=IfCondition(enable_cmd_vel_odom),
        parameters=[{
            "use_sim_time": use_sim_time,
            "cmd_vel_topic": "/cmd_vel",
            "odom_topic": "/odom",
            "odom_frame": "odom",
            "base_frame": "base_footprint",
            "publish_rate": 50.0,
            "cmd_timeout": 0.3,
        }],
    )

    return LaunchDescription([
        rmw_implementation,
        gz_resource_path,
        gz_system_plugin_path,
        declare_use_sim_time,
        declare_enable_lio,
        declare_lio_motion_diagnostics,
        declare_compensate_gimbal_for_lio,
        declare_enable_cmd_vel_odom,
        declare_use_livox_native,
        declare_use_fake_frame,
        declare_gimbal_mode,
        declare_gimbal_spin_rate,
        declare_gimbal_spin_start_delay,
        declare_gyro_spin_rate,
        declare_gazebo_gui,
        declare_world,
        robot_state_publisher,
        gz_sim,
        # Gazebo advertises /clock only after the world is running. Starting
        # the bridge immediately can leave it connected but receiving no clock
        # samples, so defer it briefly until the world transport is ready.
        TimerAction(period=2.0, actions=[bridge_clock]),
        spawn_robot,
        spawn_controllers,
        gimbal_stabilizer,
        chassis_controller,
        fake_vel_transform,
        bridge_imu,
        imu_frame_republisher,
        bridge_lidar,
        bridge_livox_native,
        pointcloud_motion_compensator,
        gimbal_imu_compensator,
        gpu_lidar_sensor_tf,
        pointcloud_frame_republisher,
        # These processing nodes only subscribe and wait until sensor data is
        # available, so start them immediately instead of adding fixed delays.
        livox_pointcloud_shaper,
        pointcloud_to_scan,
        small_point_lio,
        cmd_vel_watchdog,
        cmd_vel_odometry,
    ])
