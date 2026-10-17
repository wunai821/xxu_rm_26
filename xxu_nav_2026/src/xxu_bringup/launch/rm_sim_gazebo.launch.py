"""Use pinned rm_sim_26 geometry with the existing XXU estimation/safety chain."""

import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from xxu_description.launch_helpers import velocity_chain
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    RegisterEventHandler, SetEnvironmentVariable, TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def prepare_assets(share, directory, lidar_mode='gpu_lidar', field_model=''):
    """Adapt copies only; preserve the pinned upstream sources."""
    world = ET.parse(os.path.join(share, 'worlds', 'rmuc_2025_world.world'))
    for plugin in list(world.getroot().find('world').findall('plugin')):
        if lidar_mode == 'gpu_lidar' and 'RGL' in plugin.get('filename', ''):
            world.getroot().find('world').remove(plugin)
    if field_model:
        scene = world.getroot().find('world')
        field = ET.parse(field_model).getroot().find('model')
        if field is None or field.findtext('static') != 'true':
            raise ValueError('field_model must contain one static SDF model')
        # Resolve local mesh paths before embedding the field in a temporary world.
        for uri in field.findall('.//mesh/uri'):
            if not os.path.isabs(uri.text) and '://' not in uri.text:
                uri.text = os.path.abspath(os.path.join(os.path.dirname(field_model), uri.text))
        old_field = scene.find("model[@name='rmuc_2025']")
        scene.remove(old_field)
        scene.append(field)
        camera = scene.find("gui/plugin[@filename='MinimalScene']/camera_pose")
        if camera is not None:
            camera.text = '6.3 4.3 13 0 1.57 1.57'
    world_path = os.path.join(directory, 'rmuc_2025.world')
    world.write(world_path)

    model = ET.parse(os.path.join(share, 'models', 'sentry', 'model.sdf'))
    link = model.getroot().find("model/link[@name='livox_lidar']")
    rgl_sensor = link.find("sensor[@name='RGLLidar']")
    if lidar_mode == 'rgl':
        plugin = rgl_sensor.find('plugin')
        plugin.find('topic').text = '/rm_sim/lidar/points'
        plugin.find('range/min').text = '0.1'
        plugin.find('range/max').text = '40'
        plugin.remove(plugin.find('pattern_preset_path'))
        # Named presets carry the 40-frame alternation contract. The path
        # loader would cast the complete four-second file in every frame.
        ET.SubElement(plugin, 'pattern_preset').text = 'Livox Mid360'
        # The Livox casing is part of the parent gimbal visual. RGL ignores
        # geometry in the sensor's own link; attaching it to the empty IMU
        # link instead makes every ray stop on that casing below min range.
        # Keep the same physical sensor pose and ROS frame/extrinsics.
        link.remove(rgl_sensor)
        model.getroot().find("model/link[@name='gimbal_link']").append(rgl_sensor)
        sensor_pose = link.findtext('pose').split()
        ET.SubElement(rgl_sensor, 'pose').text = ' '.join(sensor_pose)
    else:
        link.remove(rgl_sensor)
    # One gpu_lidar frame is simultaneous. The shaper supplies identical point
    # times, never invented scan-time offsets, to the existing Livox adapter.
        link.append(ET.fromstring('''<sensor name="livox_lidar" type="gpu_lidar">
      <always_on>true</always_on><update_rate>10</update_rate>
      <topic>/rm_sim/lidar</topic><gz_frame_id>livox_lidar</gz_frame_id>
      <lidar><scan>
        <horizontal><samples>1875</samples><resolution>1</resolution>
          <min_angle>0</min_angle><max_angle>6.2831852</max_angle></horizontal>
        <vertical><samples>32</samples><resolution>1</resolution>
          <min_angle>-0.12217304764</min_angle><max_angle>0.90757121104</max_angle></vertical>
      </scan><range><min>0.1</min><max>40</max><resolution>0.01</resolution></range>
      <noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>
      </lidar></sensor>'''))
    model_path = os.path.join(directory, 'sentry.sdf')
    pose_plugin = model.getroot().find("model/plugin[@name='gz::sim::systems::PosePublisher']")
    ET.SubElement(pose_plugin, 'publish_model_pose').text = 'true'
    ET.SubElement(pose_plugin, 'use_pose_vector_msg').text = 'true'
    ET.SubElement(pose_plugin, 'update_frequency').text = '100'
    model.write(model_path)
    urdf = ET.parse(os.path.join(share, 'models', 'sentry', 'robot.urdf'))
    root = urdf.getroot()
    ET.SubElement(root, 'link', name='base_footprint')
    joint = ET.SubElement(root, 'joint', name='footprint_joint', type='fixed')
    ET.SubElement(joint, 'parent', link='base_footprint')
    ET.SubElement(joint, 'child', link='base_link')
    ET.SubElement(joint, 'origin', xyz='0 0 0.096', rpy='0 0 0')
    ET.SubElement(root, 'link', name='lio_base_sensor')
    joint = ET.SubElement(root, 'joint', name='lio_sensor_joint', type='fixed')
    ET.SubElement(joint, 'parent', link='base_link')
    ET.SubElement(joint, 'child', link='lio_base_sensor')
    ET.SubElement(joint, 'origin', xyz='0 0 0.0845', rpy='0 0 0')
    return world_path, model_path, ET.tostring(root, encoding='unicode')


def launch_setup(context):
    if LaunchConfiguration('gimbal_mode').perform(context) != 'hold':
        raise ValueError('rm_sim_26 currently supports gimbal_mode:=hold')
    share = get_package_share_directory('rm_sim_26')
    bringup = get_package_share_directory('xxu_bringup')
    lidar_mode = LaunchConfiguration('lidar_mode').perform(context)
    rgl_actions = []
    if lidar_mode == 'rgl':
        selected = LaunchConfiguration('rgl_plugin_dir').perform(context)
        candidates = [selected] if selected else [os.path.join(
            os.path.dirname(get_package_prefix('rm_sim_26')),
            'rgl_plugin', 'RGLServerPlugin')]
        if not selected:
            candidates.extend(filter(None, os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '').split(':')))
        required = ('libRobotecGPULidar.so', 'libRGLServerPluginInstance.so',
                    'libRGLServerPluginManager.so')
        plugin_dir = next((path for path in candidates if all(
            os.path.isfile(os.path.join(path, name)) for name in required)), None)
        if plugin_dir is None:
            raise RuntimeError('RGL libraries not found. Run tools/navigation/build_rgl.sh '
                               'or pass rgl_plugin_dir:=<RGLServerPlugin directory>; '
                               'lidar_mode:=gpu_lidar selects the fallback explicitly.')
        pattern_dir = os.path.join(share, 'plugin')
        if not os.path.isfile(os.path.join(pattern_dir, 'LivoxMid360.mat3x4f')):
            raise RuntimeError('Pinned LivoxMid360 pattern file is missing')
        rgl_actions = [
            SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', ':'.join(filter(None, [
                plugin_dir, os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')]))),
            SetEnvironmentVariable('RGL_PATTERNS_DIR', pattern_dir),
        ]
    temporary = tempfile.TemporaryDirectory(prefix='xxu_rm_sim_')
    world, model, robot = prepare_assets(
        share, temporary.name, lidar_mode, LaunchConfiguration('field_model').perform(context))
    lc = LaunchConfiguration
    sim = {'use_sim_time': True}

    def node(package, executable, name, parameters=None, **kwargs):
        return Node(package=package, executable=executable, name=name,
                    output='screen', parameters=[sim, parameters or {}], **kwargs)

    def bridge(name, arguments, remappings=None):
        return node('ros_gz_bridge', 'parameter_bridge', name,
                    arguments=arguments, remappings=remappings or [])

    sensor_geometry = {
        'target_frame': 'lio_base_sensor', 'joint_name': 'big_yaw_joint',
        'joint_axis': [0.0, 0.0, 1.0],
        'sensor_offset': [-0.1963, -0.0880, 0.2516],
        'sensor_rpy': [1.0472, 0.0, -1.3521],
        'max_joint_sample_gap': 0.05, 'time_reset_threshold': 0.5,
    }
    actions = [
        LogInfo(msg=f'rm_sim_26 lidar_mode={lidar_mode}; frame timestamps, no synthetic point-time spread'),
        *rgl_actions,
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda context: temporary.cleanup())])),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(filter(None, [
            os.path.join(share, 'models'), os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ('-r ' if lc('gazebo_gui').perform(context) == 'true'
                                         else '-r -s --headless-rendering ') + world,
                              'on_exit_shutdown': 'true'}.items()),
        node('robot_state_publisher', 'robot_state_publisher', 'robot_state_publisher',
             {'robot_description': robot, 'publish_frequency': 100.0}),
        TimerAction(period=3.0, actions=[node('ros_gz_sim', 'create', 'spawn_sentry',
            arguments=['-file', model, '-name', 'sentry_bot',
                       '-x', lc('spawn_x'), '-y', lc('spawn_y'), '-z', '0.3',
                       '-Y', lc('spawn_yaw')])]),
        bridge('rm_clock_bridge', ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        bridge('rm_sensor_bridge', [
            '/rm_sim/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/livox/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model']),
        # Diagnostics only. Ground truth never supplies odom or navigation TF.
        bridge('rm_truth_bridge', [
            '/model/sentry_bot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            remappings=[('/model/sentry_bot/pose', '/rm_sim/ground_truth')]),
        bridge('rm_command_bridge', [
            f'/{wheel}/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double'
            for wheel in ('wheel_fl', 'wheel_rl', 'wheel_rr', 'wheel_fr')
        ] + [f'/gimbal/{axis}/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'
             for axis in ('big_yaw', 'small_yaw', 'pitch')]),
        node('rm_sim_26', 'omni_drive.py', 'omni_drive'),
        node('xxu_bringup', 'rm_sim_adapter.py', 'rm_sim_adapter'),
        node('xxu_livox_sim', 'livox_pointcloud_shaper', 'rm_lidar_shaper', {
            'target_frame': 'livox_lidar', 'scan_period': 0.0, 'max_points': 12000},
            remappings=[('points_in', '/rm_sim/lidar/points'),
                        ('points_out', '/mid360/livox_points')]),
        node('xxu_description', 'pointcloud_frame_republisher.py', 'pointcloud_motion_compensator', {
            **sensor_geometry, 'expected_source_frame': 'livox_lidar',
            'tf_lookup_timeout': 0.0, 'use_point_timestamps': True,
            'use_joint_interpolation': True, 'pending_timeout': 0.25,
            'pending_queue_size': 3},
            remappings=[('points_in', '/mid360/livox_points'),
                        ('points_out', '/mid360/livox_points_compensated')]),
        node('xxu_description', 'gimbal_imu_compensator.py', 'gimbal_imu_compensator',
             sensor_geometry, remappings=[('imu_in', '/livox/imu'),
                                         ('imu_out', '/imu_lio_compensated')]),
        Node(package='small_point_lio', executable='small_point_lio_node',
             name='small_point_lio', output='screen', condition=IfCondition(lc('enable_lio')),
             parameters=[os.path.join(bringup, 'config', 'rm_sim_lio.yaml')]),
        node('pointcloud_to_laserscan', 'pointcloud_to_laserscan_node', 'pointcloud_to_laserscan', {
            'target_frame': 'base_footprint', 'transform_tolerance': 0.05,
            'min_height': ParameterValue(lc('scan_min_height'), value_type=float),
            'max_height': 0.50, 'angle_min': -3.1415926, 'angle_max': 3.1415926,
            'angle_increment': 0.0087266, 'scan_time': 0.1, 'range_min': 0.3,
            'range_max': 40.0,
            'robot_self_filter_radius': ParameterValue(lc('scan_self_filter_radius'), value_type=float),
            'use_inf': True, 'scan_qos_reliability': 'reliable',
            'publish_processed_cloud': True, 'processed_cloud_frame': 'base_footprint',
            'processed_cloud_project_to_2d': True, 'processed_cloud_z_value': 0.0},
            remappings=[('cloud_in', '/cloud_deskewed'), ('scan', '/scan'),
                        ('processed_cloud', '/mid360/points_navigation')]),
        *velocity_chain(
            use_sim_time=True, use_fake_frame=lc('use_fake_frame'),
            cmd_vel_topic=lc('cmd_vel_out_topic'),
            spin_speed=ParameterValue(lc('gyro_spin_rate'), value_type=float),
            velocity_window=ParameterValue(lc('nav_velocity_smoothing_window'), value_type=float),
            output_topic='/rm_sim/cmd_vel_safe',
        ),
    ]
    return actions


def generate_launch_description():
    defaults = {
        'gazebo_gui': 'true', 'enable_lio': 'true', 'use_fake_frame': 'true',
        'gimbal_mode': 'hold',
        'spawn_x': '3.4', 'spawn_y': '9.5', 'spawn_yaw': '0.0',
        'scan_min_height': '0.10', 'scan_self_filter_radius': '0.42',
        'gyro_spin_rate': '1.5', 'nav_velocity_smoothing_window': '0.3',
        'cmd_vel_out_topic': '/cmd_vel_collision',
        'rgl_plugin_dir': '', 'field_model': '',
    }
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value)
                              for key, value in defaults.items()] + [
        DeclareLaunchArgument('lidar_mode', default_value='rgl',
                              choices=['rgl', 'gpu_lidar']),
        OpaqueFunction(function=launch_setup)])
