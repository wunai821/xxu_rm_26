"""Resolve launch parameters without starting Gazebo or publishing commands."""

import importlib.util
import itertools
from pathlib import Path
import unittest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters


SRC = Path(__file__).resolve().parents[2]


def load_launch(relative_path, overrides=None):
    spec = importlib.util.spec_from_file_location('launch_under_test', SRC / relative_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    nodes = {}

    def record_node(**kwargs):
        node = Node(**kwargs)
        nodes[kwargs.get('name', kwargs.get('executable'))] = (node, kwargs)
        return node

    module.Node = record_node
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update(overrides or {})
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.name not in context.launch_configurations:
            context.launch_configurations[entity.name] = perform_substitutions(context, entity.default_value)
    return description, context, nodes


class LaunchConnections(unittest.TestCase):
    def test_entrypoints_agree_on_fake_frame(self):
        for path in ('xxu_bringup/launch/simulation.launch.py',
                     'xxu_bringup/launch/navigation.launch.py',
                     'xxu_slam_toolbox/launch/autonomous_mapping.launch.py',
                     'xxu_bringup/launch/real_robot.launch.py'):
            _, context, _ = load_launch(path)
            self.assertEqual(context.launch_configurations['use_fake_frame'], 'true', path)

    def test_gazebo_connection_matrix(self):
        for fake, lio, sim_time in itertools.product(('true', 'false'), repeat=3):
            with self.subTest(fake=fake, lio=lio, sim_time=sim_time):
                _, context, nodes = load_launch('xxu_description/launch/gazebo.launch.py', {
                    'use_fake_frame': fake, 'enable_lio': lio, 'use_sim_time': sim_time,
                    'enable_cmd_vel_odom': 'true' if lio == 'false' else 'false',
                    'cmd_vel_out_topic': '/custom_safe_velocity',
                })

                def params(name):
                    return evaluate_parameters(context, normalize_parameters(
                        [nodes[name][1]['parameters'][-1]]))[0]

                self.assertEqual(params('fake_vel_transform')['input_cmd_vel_topic'], '/custom_safe_velocity')
                watchdog = params('cmd_vel_watchdog')
                self.assertEqual(watchdog['input_topic'], '/cmd_vel_transformed' if fake == 'true' else '/custom_safe_velocity')
                self.assertTrue(watchdog['require_scan'])
                self.assertEqual(params('small_point_lio')['use_sim_time'], sim_time == 'true')
                scan, kwargs = nodes['pointcloud_to_laserscan']
                self.assertTrue(scan.condition is None or scan.condition.evaluate(context))
                source = dict(kwargs['remappings'])['cloud_in'].perform(context)
                self.assertEqual(source, '/cloud_deskewed' if lio == 'true' else '/mid360/livox_points_compensated')

    def test_simulation_forwards_custom_velocity_topic(self):
        description, context, _ = load_launch('xxu_bringup/launch/simulation.launch.py', {
            'cmd_vel_out_topic': '/custom_safe_velocity',
        })
        includes = [e for e in description.entities if isinstance(e, IncludeLaunchDescription)]
        forwarded = []
        for include in includes:
            args = dict(include.launch_arguments)
            if 'cmd_vel_out_topic' in args:
                forwarded.append(args['cmd_vel_out_topic'].perform(context))
        # Gazebo is direct; Nav2 is registered inside an OnProcessExit action.
        self.assertIn('/custom_safe_velocity', forwarded)


if __name__ == '__main__':
    unittest.main()
