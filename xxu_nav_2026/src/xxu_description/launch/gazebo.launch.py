"""启动 Gazebo Harmonic 并加载 XXU 机器人模型."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
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
    use_livox_native = LaunchConfiguration("use_livox_native")

    gz_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        [nav2_tb3_models_path, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH")],
    )
    gz_system_plugin_path = SetEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        [
            livox_plugin_path,
            ":",
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
        ],
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
    declare_use_livox_native = DeclareLaunchArgument(
        "use_livox_native",
        default_value="false",
        description="Use xxu_livox_sim Gazebo System plugin for Small Point-LIO input",
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
            "gz_args": ["-r ", PathJoinSubstitution([pkg_share, "worlds", "empty_with_sensors.sdf"])],
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
            "wheel_x": [0.127278, 0.127278, -0.127278, -0.127278],
            "wheel_y": [0.127278, -0.127278, 0.127278, -0.127278],
            "drive_direction_angle": [-0.785398, -2.356194, 0.785398, 2.356194],
            "max_wheel_speed": 100.0,
            "timeout": 0.3,
            "publish_rate": 50.0,
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
        parameters=[{"target_frame": "radar_link"}],
        remappings=[
            ("scan_in", "/scan_raw"),
            ("scan_out", "/scan"),
        ],
    )

    pointcloud_frame_republisher = Node(
        package="xxu_description",
        executable="pointcloud_frame_republisher.py",
        name="pointcloud_frame_republisher",
        output="screen",
        condition=UnlessCondition(use_livox_native),
        parameters=[{"target_frame": "radar_link"}],
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
            "target_frame": "radar_link",
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
        parameters=[{"target_frame": "radar_link"}],
        remappings=[
            ("points_in", "/mid360/livox_points_raw"),
            ("points_out", "/mid360/livox_points"),
        ],
    )

    pointcloud_processor = Node(
        package="xxu_pointcloud_processing",
        executable="pointcloud_processor",
        name="pointcloud_processor",
        output="screen",
        parameters=[{
            "input_frame": "radar_link",
            "output_frame": "base_footprint",
            "project_to_2d": True,
            "z_value": 0.0,
            "min_height": 0.05,
            "max_height": 0.5,
            "min_range": 0.05,
            "max_range": 40.0,
            "min_x": -40.0,
            "max_x": 40.0,
            "min_y": -40.0,
            "max_y": 40.0,
        }],
        remappings=[
            ("points_in", "/mid360/points_raw"),
            ("points_out", "/mid360/points"),
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
            "input_topic": "/cmd_vel_keyboard",
            "output_topic": "/cmd_vel",
            "timeout": 0.3,
            "publish_rate": 20.0,
        }],
    )

    keyboard_teleop = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--",
            "ros2",
            "run",
            "teleop_twist_keyboard",
            "teleop_twist_keyboard",
            "--ros-args",
            "-r",
            "cmd_vel:=/cmd_vel_keyboard",
        ],
        output="screen",
    )

    return LaunchDescription([
        gz_resource_path,
        gz_system_plugin_path,
        declare_use_sim_time,
        declare_enable_lio,
        declare_use_livox_native,
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        spawn_controllers,
        chassis_controller,
        bridge_imu,
        bridge_lidar,
        bridge_livox_native,
        scan_frame_republisher,
        pointcloud_frame_republisher,
        livox_pointcloud_shaper,
        livox_native_frame_republisher,
        pointcloud_processor,
        small_point_lio,
        bridge_clock,
        cmd_vel_watchdog,
        keyboard_teleop,
    ])
