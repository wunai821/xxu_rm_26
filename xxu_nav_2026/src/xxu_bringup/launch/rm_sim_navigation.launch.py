"""rm_sim_26 sentry and RMUC 2025 field, with existing LIO / AMCL / Nav2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = FindPackageShare('xxu_bringup')
    defaults = {
        'start_navigation': 'true', 'enable_localization': 'true',
        'rviz': 'true', 'gazebo_gui': 'true', 'enable_lio': 'true',
        'map': PathJoinSubstitution([share, 'maps', 'rmuc_2025.yaml']),
        'nav2_params_file': PathJoinSubstitution([share, 'config', 'nav2_navigation.yaml']),
        'spawn_x': '3.4', 'spawn_y': '9.5', 'spawn_yaw': '0.0',
        'initial_pose_x': LaunchConfiguration('spawn_x'),
        'initial_pose_y': LaunchConfiguration('spawn_y'),
        'initial_pose_yaw': LaunchConfiguration('spawn_yaw'),
        'initial_pose_relocalize': 'false',
        'initial_pose_covariance_xy': '0.0025', 'initial_pose_covariance_yaw': '0.0025',
        'use_fake_frame': 'true', 'gyro_spin_rate': '1.5',
        'gimbal_mode': 'hold',
        'scan_self_filter_radius': '0.42', 'scan_min_height': '0.10',
        'lidar_mode': 'rgl', 'rgl_plugin_dir': '',
        'field_model': '',
    }
    # Exact simulated spawn and the tilted GPU scan have different uncertainty
    # and beam coverage from the old field. Keep controller/safety parameters
    # in the existing file; specialize only this simulator's AMCL observation.
    localization_profile = RewrittenYaml(
        source_file=LaunchConfiguration('nav2_params_file'),
        param_rewrites={
            **{f'amcl.ros__parameters.alpha{i}': '0.02' for i in range(1, 6)},
            'amcl.ros__parameters.max_beams': '180',
            'amcl.ros__parameters.sigma_hit': '0.10',
            'amcl.ros__parameters.laser_likelihood_max_dist': '0.5',
            'amcl.ros__parameters.z_hit': '0.9',
            'amcl.ros__parameters.z_rand': '0.1',
            'amcl.ros__parameters.update_min_d': '0.05',
            'amcl.ros__parameters.update_min_a': '0.05',
        }, convert_types=True)
    return LaunchDescription([
        DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()
    ] + [IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([share, 'launch', 'simulation.launch.py'])),
        launch_arguments={**{key: LaunchConfiguration(key) for key in defaults},
                          'nav2_params_file': localization_profile,
                          'sim_backend': 'rm_sim_26', 'use_sim_time': 'true'}.items())])
