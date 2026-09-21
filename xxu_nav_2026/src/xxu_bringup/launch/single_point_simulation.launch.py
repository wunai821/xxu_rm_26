"""Start simulation, load the saved map, and use RViz for navigation goals."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("xxu_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
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
    initial_pose_relocalize = LaunchConfiguration("initial_pose_relocalize")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_covariance_xy = LaunchConfiguration("initial_pose_covariance_xy")
    initial_pose_covariance_yaw = LaunchConfiguration("initial_pose_covariance_yaw")

    default_map = PathJoinSubstitution([bringup_share, "maps", "complex_map.yaml"])
    default_nav2_params = PathJoinSubstitution(
        [bringup_share, "config", "nav2_navigation.yaml"]
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([bringup_share, "launch", "simulation.launch.py"])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_navigation": "true",
            "enable_localization": LaunchConfiguration("enable_localization"),
            "map": map_file,
            "nav2_params_file": nav2_params_file,
            "rviz": rviz,
            "enable_lio": enable_lio,
            "enable_cmd_vel_odom": enable_cmd_vel_odom,
            "use_livox_native": use_livox_native,
            "use_fake_frame": use_fake_frame,
            "gyro_spin_rate": gyro_spin_rate,
            "gazebo_gui": gazebo_gui,
            "gimbal_mode": gimbal_mode,
            "world": world,
            "auto_initial_pose": auto_initial_pose,
            "initial_pose_relocalize": initial_pose_relocalize,
            "initial_pose_x": initial_pose_x,
            "initial_pose_y": initial_pose_y,
            "initial_pose_yaw": initial_pose_yaw,
            "initial_pose_covariance_xy": initial_pose_covariance_xy,
            "initial_pose_covariance_yaw": initial_pose_covariance_yaw,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_localization",
            default_value="false",
            description="Re-enable paused AMCL/GICP localization and map navigation",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Full path to the saved map yaml file",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=default_nav2_params,
            description="Full path to the Nav2 parameters file",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz from navigation.launch.py",
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
            default_value="1.5",
            description="Chassis gyro angular speed while translating (rad/s); 0 disables it",
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Start the Gazebo Sim GUI; disable it to reduce CPU load",
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
            description="Automatically publish the simulation AMCL initial pose",
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
        simulation,
    ])
