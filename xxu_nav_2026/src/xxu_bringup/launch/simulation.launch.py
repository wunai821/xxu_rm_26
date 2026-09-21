"""Start the XXU Gazebo simulation and optionally the Nav2 stack."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
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
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("xxu_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_navigation = LaunchConfiguration("start_navigation")
    enable_localization = LaunchConfiguration("enable_localization")
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
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    auto_initial_pose = LaunchConfiguration("auto_initial_pose")
    enable_gicp = LaunchConfiguration("enable_gicp")
    amcl_particle_diagnostics = LaunchConfiguration("amcl_particle_diagnostics")
    amcl_diagnostic_reference_enabled = LaunchConfiguration("amcl_diagnostic_reference_enabled")
    amcl_diagnostic_reference_x = LaunchConfiguration("amcl_diagnostic_reference_x")
    amcl_diagnostic_reference_y = LaunchConfiguration("amcl_diagnostic_reference_y")
    amcl_diagnostic_reference_yaw = LaunchConfiguration("amcl_diagnostic_reference_yaw")
    gicp_pcd_map = LaunchConfiguration("gicp_pcd_map")
    initial_pose_relocalize = LaunchConfiguration("initial_pose_relocalize")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_covariance_xy = LaunchConfiguration("initial_pose_covariance_xy")
    initial_pose_covariance_yaw = LaunchConfiguration("initial_pose_covariance_yaw")
    cmd_vel_in_topic = LaunchConfiguration("cmd_vel_in_topic")
    cmd_vel_out_topic = LaunchConfiguration("cmd_vel_out_topic")
    scan_self_filter_radius = LaunchConfiguration("scan_self_filter_radius")
    scan_min_height = LaunchConfiguration("scan_min_height")
    planner_tolerance = LaunchConfiguration("planner_tolerance")

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
    navigation_enabled = AndSubstitution(
        AndSubstitution(start_navigation, enable_localization),
        enable_lio,
    )
    defer_localization_until_initial_pose = LaunchConfiguration(
        "defer_localization_until_initial_pose"
    )
    localization_deferred = AndSubstitution(
        AndSubstitution(navigation_enabled, auto_initial_pose),
        defer_localization_until_initial_pose,
    )
    nav2_localization_autostart = PythonExpression([
        "'false' if '", localization_deferred, "' == 'true' else 'true'"
    ])
    navigation_disabled_without_lio = LogInfo(
        msg=(
            "Navigation was not started because enable_lio=false: "
            "the navigation scan source /cloud_deskewed is owned by Small Point-LIO."
        ),
        condition=IfCondition(
            AndSubstitution(start_navigation, NotSubstitution(enable_lio))
        ),
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
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_yaw": spawn_yaw,
            "scan_self_filter_radius": scan_self_filter_radius,
            "scan_min_height": scan_min_height,
            "cmd_vel_out_topic": cmd_vel_out_topic,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, "launch", "navigation.launch.py"])
        ]),
        condition=IfCondition(navigation_enabled),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_file,
            "params_file": nav2_params_file,
            "rviz": rviz,
            "use_fake_frame": use_fake_frame,
            "enable_gicp": enable_gicp,
            "enable_localization": enable_localization,
            "gicp_pcd_map": gicp_pcd_map,
            "cmd_vel_in_topic": cmd_vel_in_topic,
            "cmd_vel_out_topic": cmd_vel_out_topic,
            "planner_tolerance": planner_tolerance,
            "autostart": nav2_localization_autostart,
        }.items(),
    )

    initial_pose = Node(
        package="xxu_bringup",
        executable="auto_initial_pose.py",
        name="auto_initial_pose",
        output="both",
        condition=IfCondition(AndSubstitution(navigation_enabled, auto_initial_pose)),
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
            "defer_amcl_activation": localization_deferred,
        }],
    )

    odom_ready_gate = Node(
        package="xxu_bringup",
        executable="wait_for_odom.py",
        name="wait_for_odom",
        output="screen",
        condition=IfCondition(AndSubstitution(start_navigation, enable_lio)),
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
            "enable_localization",
            default_value="false",
            description="Re-enable paused AMCL/GICP localization and map navigation",
        ),
        DeclareLaunchArgument("amcl_particle_diagnostics", default_value="false"),
        DeclareLaunchArgument("amcl_diagnostic_reference_enabled", default_value="false"),
        DeclareLaunchArgument("amcl_diagnostic_reference_x", default_value="0.0"),
        DeclareLaunchArgument("amcl_diagnostic_reference_y", default_value="0.0"),
        DeclareLaunchArgument("amcl_diagnostic_reference_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "defer_localization_until_initial_pose",
            default_value="true",
            description="Configure map/AMCL first and activate AMCL only after initial-pose delivery",
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
            "planner_tolerance",
            default_value="1.0",
            description="Navfn endpoint tolerance in metres",
        ),
        DeclareLaunchArgument(
            "enable_lio",
            default_value="true",
            description="Start Small Point-LIO in the Gazebo launch",
        ),
        DeclareLaunchArgument(
            "enable_cmd_vel_odom",
            default_value="false",
            description=(
                "Publish odom by integrating /cmd_vel when LIO is disabled; "
                "this fallback has no /scan and cannot be used for navigation"
            ),
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
            default_value="1.5",
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
            "spawn_x", default_value="1.75",
            description="Gazebo robot spawn x in world coordinates",
        ),
        DeclareLaunchArgument(
            "spawn_y", default_value="0.0",
            description="Gazebo robot spawn y in world coordinates",
        ),
        DeclareLaunchArgument(
            "spawn_yaw", default_value="3.14159",
            description="Gazebo robot spawn yaw in world coordinates",
        ),
        DeclareLaunchArgument(
            "scan_self_filter_radius", default_value="0.42",
            description="Base-frame radius that removes chassis self-returns from the shared navigation scan",
        ),
        DeclareLaunchArgument(
            "scan_min_height", default_value="0.30",
            description="Minimum base-frame point height retained in the shared navigation scan",
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
        Node(
            package="xxu_bringup",
            executable="amcl_particle_diagnostic.py",
            name="amcl_particle_diagnostic",
            output="both",
            condition=IfCondition(AndSubstitution(navigation_enabled, amcl_particle_diagnostics)),
            parameters=[{
                "use_sim_time": use_sim_time,
                "reference_enabled": amcl_diagnostic_reference_enabled,
                "reference_x": amcl_diagnostic_reference_x,
                "reference_y": amcl_diagnostic_reference_y,
                "reference_yaw": amcl_diagnostic_reference_yaw,
            }],
        ),
        navigation_disabled_without_lio,
        # Start Nav2 only after the simulator has produced a real /odom message.
        # navigation.launch.py then gates lifecycle activation on map -> odom.
        odom_ready_gate,
        start_navigation_after_odom,
        initial_pose,
    ])
