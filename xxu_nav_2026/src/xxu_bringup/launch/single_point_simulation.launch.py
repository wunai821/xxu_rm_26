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

    default_map = PathJoinSubstitution([bringup_share, "maps", "auto_map.yaml"])
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
            "map": map_file,
            "nav2_params_file": nav2_params_file,
            "rviz": rviz,
            "enable_lio": enable_lio,
            "enable_cmd_vel_odom": enable_cmd_vel_odom,
            "use_livox_native": use_livox_native,
        }.items(),
    )

    return LaunchDescription([
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
            default_value="false",
            description="Use the xxu_livox_sim Gazebo System plugin",
        ),
        simulation,
    ])
