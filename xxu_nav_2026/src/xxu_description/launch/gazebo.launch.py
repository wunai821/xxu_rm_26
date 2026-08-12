"""启动 Gazebo Harmonic 并加载 XXU 机器人模型."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
    small_point_lio_config = PathJoinSubstitution(
        [FindPackageShare("small_point_lio"), "config", "xxu_gazebo_mid360.yaml"]
    )
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
    declare_enable_lio = DeclareLaunchArgument(
        "enable_lio",
        default_value="true",
        description="Start Small Point-LIO and publish /odom",
    )
    declare_enable_cmd_vel_odom = DeclareLaunchArgument(
        "enable_cmd_vel_odom",
        default_value="false",
        description="Publish simulation planar /odom by integrating /cmd_vel",
    )
    declare_use_livox_native = DeclareLaunchArgument(
        "use_livox_native",
        default_value="false",
        description="Use xxu_livox_sim Gazebo System plugin for Small Point-LIO input",
    )
    declare_use_fake_frame = DeclareLaunchArgument(
        "use_fake_frame",
        default_value="false",
        description="Use fake_vel_transform and base_link_fake for Nav2 velocity commands",
    )
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([pkg_share, "worlds", "empty_with_sensors.sdf"]),
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
                ]),
                value_type=str,
            ),
            "use_sim_time": use_sim_time,
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
            "gz_args": ["-r ", world],
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

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                joint_state_broadcaster_spawner,
                wheel_velocity_controller_spawner,
            ],
        )
    )

    chassis_controller = Node(
        package="xxu_chassis_controller",
        executable="chassis_controller",
        name="chassis_controller",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "wheel_radius": 0.07,
            "wheel_command_sign": -1.0,
            "wheel_x": [0.127278, 0.127278, -0.127278, -0.127278],
            "wheel_y": [0.127278, -0.127278, 0.127278, -0.127278],
            "drive_direction_angle": [-0.785398, -2.356194, 0.785398, 2.356194],
            "max_wheel_speed": 100.0,
            "timeout": 0.3,
            "publish_rate": 50.0,
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
            "fake_robot_base_frame": "base_link_fake",
            "odom_topic": "/odom",
            "input_cmd_vel_topic": "/cmd_vel_keyboard",
            "output_cmd_vel_topic": "/cmd_vel_transformed",
            "spin_speed": 0.0,
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
        parameters=[{"target_frame": "base_footprint"}],
        remappings=[
            ("imu_in", "/imu_raw"),
            ("imu_out", "/imu"),
        ],
    )

    # MID360 LiDAR bridge (GZ -> ROS 单向)
    bridge_lidar = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_lidar",
        arguments=[
            "/mid360@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/mid360/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        output="screen",
        remappings=[
            ("/mid360", "/scan_raw"),
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
            ("/mid360/livox_points_gz", "/mid360/livox_points_raw"),
        ],
    )

    scan_frame_republisher = Node(
        package="xxu_description",
        executable="scan_frame_republisher.py",
        name="scan_frame_republisher",
        output="screen",
        parameters=[{"target_frame": "base_footprint"}],
        remappings=[
            ("scan_in", "/scan_raw"),
            ("scan_out", "/scan_raw_fixed"),
        ],
    )

    pointcloud_frame_republisher = Node(
        package="xxu_description",
        executable="pointcloud_frame_republisher.py",
        name="pointcloud_frame_republisher",
        output="screen",
        condition=UnlessCondition(use_livox_native),
        parameters=[{"target_frame": "base_footprint"}],
        remappings=[
            ("points_in", "/mid360/points_raw"),
            ("points_out", "/mid360/points_lio"),
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
            "target_frame": "base_footprint",
            "scan_period": 0.1,
            "max_points": 12000,
            "scan_mode_csv": mid360_scan_mode_csv,
        }],
        remappings=[
            ("points_in", "/mid360/points_lio"),
            ("points_out", "/mid360/livox_points"),
        ],
    )

    livox_native_frame_republisher = Node(
        package="xxu_description",
        executable="pointcloud_frame_republisher.py",
        name="livox_native_frame_republisher",
        output="screen",
        condition=IfCondition(use_livox_native),
        parameters=[{"target_frame": "base_footprint"}],
        remappings=[
            ("points_in", "/mid360/livox_points_raw"),
            ("points_out", "/mid360/livox_points"),
        ],
    )

    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[{
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
            ("cloud_in", "/mid360/livox_points"),
            ("scan", "/scan"),
            ("processed_cloud", "/mid360/points"),
        ],
    )

    small_point_lio = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        condition=IfCondition(enable_lio),
        parameters=[small_point_lio_config],
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
            "input_topic": PythonExpression([
                "'/cmd_vel_transformed' if '",
                use_fake_frame,
                "' == 'true' else '/cmd_vel_keyboard'",
            ]),
            "output_topic": "/cmd_vel",
            "timeout": 0.3,
            "publish_rate": 20.0,
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
        declare_enable_cmd_vel_odom,
        declare_use_livox_native,
        declare_use_fake_frame,
        declare_world,
        robot_state_publisher,
        gz_sim,
        # Gazebo advertises /clock only after the world is running. Starting
        # the bridge immediately can leave it connected but receiving no clock
        # samples, so defer it briefly until the world transport is ready.
        TimerAction(period=2.0, actions=[bridge_clock]),
        spawn_robot,
        spawn_controllers,
        chassis_controller,
        fake_vel_transform,
        bridge_imu,
        imu_frame_republisher,
        bridge_lidar,
        bridge_livox_native,
        scan_frame_republisher,
        pointcloud_frame_republisher,
        TimerAction(period=15.0, actions=[livox_pointcloud_shaper]),
        livox_native_frame_republisher,
        pointcloud_to_scan,
        TimerAction(period=20.0, actions=[small_point_lio]),
        cmd_vel_watchdog,
        cmd_vel_odometry,
    ])
