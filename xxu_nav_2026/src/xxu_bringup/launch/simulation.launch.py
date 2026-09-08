"""Start the XXU Gazebo simulation and optionally the Nav2 stack."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("xxu_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_navigation = LaunchConfiguration("start_navigation")
    map_file = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    rviz = LaunchConfiguration("rviz")
    enable_lio = LaunchConfiguration("enable_lio")
    enable_cmd_vel_odom = LaunchConfiguration("enable_cmd_vel_odom")
    use_livox_native = LaunchConfiguration("use_livox_native")
    use_fake_frame = LaunchConfiguration("use_fake_frame")
    gyro_spin_rate = LaunchConfiguration("gyro_spin_rate")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    gimbal_mode = LaunchConfiguration("gimbal_mode")
    world = LaunchConfiguration("world")
    auto_initial_pose = LaunchConfiguration("auto_initial_pose")
    enable_gicp = LaunchConfiguration("enable_gicp")
    gicp_pcd_map = LaunchConfiguration("gicp_pcd_map")
    initial_pose_relocalize = LaunchConfiguration("initial_pose_relocalize")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_covariance_xy = LaunchConfiguration("initial_pose_covariance_xy")
    initial_pose_covariance_yaw = LaunchConfiguration("initial_pose_covariance_yaw")
    cmd_vel_in_topic = LaunchConfiguration("cmd_vel_in_topic")
    cmd_vel_out_topic = LaunchConfiguration("cmd_vel_out_topic")

    default_map = PathJoinSubstitution([bringup_share, "maps", "complex_map.yaml"])
    default_nav2_params = PathJoinSubstitution(
        [bringup_share, "config", "nav2_navigation.yaml"]
    )
    # Small Point-LIO owns /odom and odom -> base_footprint when enabled.
    # Only allow the command-integrating fallback when LIO is disabled.
    effective_cmd_vel_odom = AndSubstitution(
        enable_cmd_vel_odom,
        NotSubstitution(enable_lio),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("xxu_description"),
                "launch",
                "gazebo.launch.py",
            ])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "enable_lio": enable_lio,
            "enable_cmd_vel_odom": effective_cmd_vel_odom,
            "use_livox_native": use_livox_native,
            "use_fake_frame": use_fake_frame,
            "gyro_spin_rate": gyro_spin_rate,
            "gazebo_gui": gazebo_gui,
            "gimbal_mode": gimbal_mode,
            "world": world,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, "launch", "navigation.launch.py"])
        ]),
        condition=IfCondition(start_navigation),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_file,
            "params_file": nav2_params_file,
            "rviz": rviz,
            "use_fake_frame": use_fake_frame,
            "enable_gicp": enable_gicp,
            "gicp_pcd_map": gicp_pcd_map,
            "cmd_vel_in_topic": cmd_vel_in_topic,
            "cmd_vel_out_topic": cmd_vel_out_topic,
        }.items(),
    )

    initial_pose = Node(
        package="xxu_bringup",
        executable="auto_initial_pose.py",
        name="auto_initial_pose",
        output="both",
        condition=IfCondition(AndSubstitution(start_navigation, auto_initial_pose)),
        parameters=[{
            "use_sim_time": use_sim_time,
            "relocalize": initial_pose_relocalize,
            "map_yaml": map_file,
            "x": initial_pose_x,
            "y": initial_pose_y,
            "yaw": initial_pose_yaw,
            "covariance_x": initial_pose_covariance_xy,
            "covariance_y": initial_pose_covariance_xy,
            "covariance_yaw": initial_pose_covariance_yaw,
        }],
    )

    odom_ready_gate = Node(
        package="xxu_bringup",
        executable="wait_for_odom.py",
        name="wait_for_odom",
        output="screen",
        condition=IfCondition(start_navigation),
        parameters=[{"use_sim_time": use_sim_time, "topic": "/odom"}],
    )

    start_navigation_after_odom = RegisterEventHandler(
        OnProcessExit(
            target_action=odom_ready_gate,
            on_exit=[navigation],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),
        DeclareLaunchArgument(
            "start_navigation",
            default_value="false",
            description="Start Nav2 localization and navigation after Gazebo starts",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Full path to the map yaml used by Nav2 map_server",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=default_nav2_params,
            description="Full path to the Nav2 parameters file",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Start RViz from navigation.launch.py",
        ),
        DeclareLaunchArgument(
            "enable_gicp",
            default_value="false",
            description="Use AMCL for coarse pose and small_gicp for map->odom",
        ),
        DeclareLaunchArgument(
            "gicp_pcd_map",
            default_value="",
            description="Map-frame PCD for small_gicp localization",
        ),
        DeclareLaunchArgument(
            "cmd_vel_in_topic",
            default_value="cmd_vel_smoothed",
            description="Velocity input consumed by collision_monitor",
        ),
        DeclareLaunchArgument(
            "cmd_vel_out_topic",
            default_value="/cmd_vel_collision",
            description="Collision-monitored velocity output",
        ),
        DeclareLaunchArgument(
            "enable_lio",
            default_value="true",
            description="Start Small Point-LIO in the Gazebo launch",
        ),
        DeclareLaunchArgument(
            "enable_cmd_vel_odom",
            default_value="false",
            description="Publish planar odometry by integrating /cmd_vel",
        ),
        DeclareLaunchArgument(
            "use_livox_native",
            default_value="true",
            description="Use the batch-raycast MID360 simulation with per-point timestamps",
        ),
        DeclareLaunchArgument(
            "use_fake_frame",
            default_value="true",
            description="Use gimbal_yaw_fake and fake_vel_transform for Nav2",
        ),
        DeclareLaunchArgument(
            "gyro_spin_rate",
            default_value="31.4159265359",
            description="Chassis gyro angular speed while translating (rad/s); 0 disables it",
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Start the Gazebo Sim GUI; disable it to reduce CPU load during mapping",
        ),
        DeclareLaunchArgument(
            "gimbal_mode",
            default_value="spin",
            description="Radar gimbal mode: spin or hold",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([
                FindPackageShare("xxu_description"),
                "worlds",
                "complex_mapping.sdf",
            ]),
            description="Gazebo world SDF path",
        ),
        DeclareLaunchArgument(
            "auto_initial_pose",
            default_value="true",
            description="Scan-match the saved map and publish an AMCL initial pose",
        ),
        DeclareLaunchArgument(
            "initial_pose_relocalize",
            default_value="true",
            description="Estimate the initial pose by matching /scan against the map",
        ),
        DeclareLaunchArgument(
            "initial_pose_x",
            default_value="0.02",
            description="Fixed simulation initial pose x in map",
        ),
        DeclareLaunchArgument(
            "initial_pose_y",
            default_value="0.03",
            description="Fixed simulation initial pose y in map",
        ),
        DeclareLaunchArgument(
            "initial_pose_yaw",
            default_value="0.0",
            description="Fixed simulation initial pose yaw in map",
        ),
        DeclareLaunchArgument(
            "initial_pose_covariance_xy",
            default_value="1.0",
            description="Initial x/y variance in m^2; increase for a rough hand-set pose",
        ),
        DeclareLaunchArgument(
            "initial_pose_covariance_yaw",
            default_value="0.2741557",
            description="Initial yaw variance in rad^2 (default is about 30 deg sigma)",
        ),
        gazebo,
        # Start Nav2 only after the simulator has produced a real /odom message.
        # navigation.launch.py then gates lifecycle activation on map -> odom.
        odom_ready_gate,
        start_navigation_after_odom,
        initial_pose,
    ])
