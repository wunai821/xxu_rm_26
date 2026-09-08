"""Launch the AMCL-seeded small_gicp localization node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('xxu_gicp_localization')
    params_file = PathJoinSubstitution([package_share, 'config', 'gicp_localization.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('pcd_map', default_value='', description='Map-frame PCD file'),
        DeclareLaunchArgument('params_file', default_value=params_file),
        Node(
            package='xxu_gicp_localization',
            executable='gicp_localization_node',
            name='gicp_localization',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'pcd_map': LaunchConfiguration('pcd_map'),
            }],
        ),
    ])
