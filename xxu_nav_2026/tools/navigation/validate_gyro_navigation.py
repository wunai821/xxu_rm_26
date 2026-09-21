#!/usr/bin/env python3
"""Isolated ROS regression: real MPPI + fake adapter + ideal planar chassis.

Does not launch hardware, Gazebo, AMCL, or publish /cmd_vel. Verifies the
navigation/controller interface; it does not certify physical tracking.
"""
import importlib.util
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

import yaml


def main():
    os.environ['ROS_DOMAIN_ID'] = '187'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from rclpy.action import ActionClient
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from geometry_msgs.msg import TwistStamped, TransformStamped, PoseStamped
    from nav_msgs.msg import Odometry, Path as NavPath
    from sensor_msgs.msg import LaserScan
    from nav2_msgs.action import FollowPath
    from lifecycle_msgs.srv import ChangeState
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
    from launch import LaunchContext
    from ament_index_python.packages import get_package_prefix

    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('navigation_launch', root / 'src/xxu_bringup/launch/navigation.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    nodes = []
    original_node = module.Node

    def capture(**kwargs):
        nodes.append(kwargs)
        return original_node(**kwargs)

    module.Node = capture
    module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations.update({
        'params_file': str(root / 'src/xxu_bringup/config/nav2_navigation.yaml'),
        'use_fake_frame': 'true', 'use_sim_time': 'false',
        'cmd_vel_in_topic': 'cmd_vel_smoothed', 'cmd_vel_out_topic': '/cmd_vel_collision',
    })
    selected = next(n for n in nodes if n.get('executable') == 'controller_server')['parameters'][0]
    params = yaml.safe_load(Path(selected.perform(context)).read_text())
    controller = params['controller_server']['ros__parameters']
    assert controller['odom_topic'] == '/odom_nav'
    assert controller['goal_checker']['plugin'] == 'nav2_controller::PositionGoalChecker'
    assert controller['FollowPath']['wz_max'] == 0.0
    context.launch_configurations['use_fake_frame'] = 'false'
    original = yaml.safe_load(Path(selected.perform(context)).read_text())
    assert original['controller_server']['ros__parameters']['FollowPath']['wz_max'] == 0.8
    assert original['controller_server']['ros__parameters']['goal_checker']['plugin'] == 'nav2_controller::SimpleGoalChecker'

    rclpy.init()
    node = rclpy.create_node('gyro_navigation_regression')
    processes = []
    logs = []
    temp = tempfile.TemporaryDirectory(prefix='gyro-navigation-')

    def start(package, executable, args):
        logfile = open(Path(temp.name) / (executable + '.log'), 'w+')
        logs.append(logfile)
        binary = Path(get_package_prefix(package)) / 'lib' / package / executable
        processes.append(subprocess.Popen([str(binary), '--ros-args', *args], stdout=logfile, stderr=subprocess.STDOUT))

    received = {}
    for topic, cls in [('/odom_nav', Odometry), ('/cmd_vel_transformed', TwistStamped), ('/cmd_vel_collision', TwistStamped)]:
        node.create_subscription(cls, topic, lambda msg, key=topic: received.__setitem__(key, msg), 20)
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    command_pub = node.create_publisher(TwistStamped, '/cmd_vel_collision', 10)
    scan_pub = node.create_publisher(LaserScan, '/scan', 10)
    tf = TransformBroadcaster(node)
    static_tf = StaticTransformBroadcaster(node)
    static = TransformStamped()
    static.header.stamp = node.get_clock().now().to_msg()
    static.header.frame_id = 'base_footprint'
    static.child_frame_id = 'base_link'
    static.transform.translation.z = 0.09519
    static.transform.rotation.w = 1.0
    static_tf.sendTransform(static)
    state = {'x': 0.0, 'y': 0.0, 'yaw': math.pi / 2, 'vx': 0.0, 'vy': -0.2, 'wz': 1.5}
    flags = {'plant': False, 'odom': True}
    last_tick = time.monotonic()

    def tick():
        nonlocal last_tick
        now = time.monotonic()
        dt = min(now - last_tick, 0.05)
        last_tick = now
        if flags['plant']:
            cmd = received.get('/cmd_vel_transformed', TwistStamped()).twist
            state.update(vx=cmd.linear.x, vy=cmd.linear.y, wz=cmd.angular.z)
            c, s = math.cos(state['yaw']), math.sin(state['yaw'])
            state['x'] += (c * state['vx'] - s * state['vy']) * dt
            state['y'] += (s * state['vx'] + c * state['vy']) * dt
            state['yaw'] += state['wz'] * dt
        if not flags['odom']:
            return
        odom = Odometry()
        odom.header.stamp = node.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = state['x'], state['y']
        odom.pose.pose.orientation.z = math.sin(state['yaw'] / 2)
        odom.pose.pose.orientation.w = math.cos(state['yaw'] / 2)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = state['vx'], state['vy']
        odom.twist.twist.angular.z = state['wz']
        odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x, transform.transform.translation.y = state['x'], state['y']
        transform.transform.rotation = odom.pose.pose.orientation
        tf.sendTransform(transform)
        scan = LaserScan()
        scan.header.stamp = odom.header.stamp
        scan.header.frame_id = 'base_footprint'
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi / 180
        scan.range_min, scan.range_max = 0.05, 8.0
        scan.ranges = [float('inf')] * 361
        scan_pub.publish(scan)

    node.create_timer(0.01, tick)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def wait(future, timeout=10):
        end = time.monotonic() + timeout
        report = time.monotonic() + 5
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
            if timeout >= 30 and time.monotonic() >= report:
                print('Tracking:', {k: round(v, 3) for k, v in state.items()}, flush=True)
                report += 5
        assert future.done(), 'ROS operation timed out'
        return future.result()

    def command(vx=0.0, vy=0.0):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'gimbal_yaw_fake'
        msg.twist.linear.x, msg.twist.linear.y = vx, vy
        command_pub.publish(msg)

    def spin_rate(rate):
        client = node.create_client(SetParameters, '/fake_vel_transform/set_parameters')
        assert client.wait_for_service(timeout_sec=5)
        req = SetParameters.Request()
        req.parameters = [Parameter(name='spin_speed', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=rate))]
        assert wait(client.call_async(req)).results[0].successful
        node.destroy_client(client)

    try:
        start('fake_vel_transform', 'fake_vel_transform_node', [
            '-p', 'odom_topic:=/odom', '-p', 'input_cmd_vel_topic:=/cmd_vel_collision',
            '-p', 'output_cmd_vel_topic:=/cmd_vel_transformed', '-p', 'spin_speed:=1.5'])
        spin(1.5)
        nav = received['/odom_nav']
        assert nav.child_frame_id == 'gimbal_yaw_fake'
        assert abs(nav.twist.twist.linear.x - 0.2) < 1e-6
        assert abs(nav.twist.twist.linear.y) < 1e-6
        assert nav.twist.twist.angular.z == 0.0 and nav.pose.pose.orientation.w == 1.0
        command(0.2)
        spin(0.1)
        out = received['/cmd_vel_transformed'].twist
        assert abs(out.linear.y + 0.2) < 1e-6 and out.angular.z == 1.5
        state['yaw'] = math.pi
        spin(0.1)  # No new navigation command: adapter must rotate the held command.
        assert abs(received['/cmd_vel_transformed'].twist.linear.x + 0.2) < 1e-6
        spin(0.3)
        assert received['/cmd_vel_transformed'].twist == TwistStamped().twist
        command(0.2)
        spin(0.05)
        command()
        spin(0.05)
        assert received['/cmd_vel_transformed'].twist == TwistStamped().twist
        spin_rate(0.0)
        command(0.2)
        spin(0.05)
        assert received['/cmd_vel_transformed'].twist.angular.z == 0.0
        spin_rate(1.5)
        flags['odom'] = False
        for _ in range(7):
            command(0.2)
            spin(0.1)
        assert received['/cmd_vel_transformed'].twist == TwistStamped().twist
        flags['odom'] = True
        command()
        state.update(x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, wz=0.0)
        flags['plant'] = True
        print('PASS: frame conversion, held-command rotation, runtime spin switch, zero command, stale command/odom', flush=True)

        params_file = Path(temp.name) / 'nav2.yaml'
        params_file.write_text(yaml.safe_dump(params))
        start('nav2_controller', 'controller_server', ['--params-file', str(params_file), '-r', 'cmd_vel:=cmd_vel_collision'])
        client = node.create_client(ChangeState, '/controller_server/change_state')
        assert client.wait_for_service(timeout_sec=10)
        for transition in [1, 3]:
            req = ChangeState.Request()
            req.transition.id = transition
            assert wait(client.call_async(req), 15).success, 'Controller lifecycle transition failed'
            spin(0.3)
        action = ActionClient(node, FollowPath, '/follow_path')
        assert action.wait_for_server(timeout_sec=10)
        for target, rate in [((1.0, 0.4), 0.0), ((0.2, 1.0), 1.5)]:
            spin_rate(rate)
            path = NavPath()
            path.header.frame_id = 'odom'
            path.header.stamp = node.get_clock().now().to_msg()
            initial = (state['x'], state['y'])
            for i in range(41):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = initial[0] + (target[0] - initial[0]) * i / 40
                pose.pose.position.y = initial[1] + (target[1] - initial[1]) * i / 40
                pose.pose.orientation.z = math.sin(1.2 / 2)
                pose.pose.orientation.w = math.cos(1.2 / 2)
                path.poses.append(pose)
            goal = FollowPath.Goal()
            goal.path, goal.controller_id, goal.goal_checker_id = path, 'FollowPath', 'goal_checker'
            handle = wait(action.send_goal_async(goal))
            assert handle.accepted
            result = wait(handle.get_result_async(), 30)
            distance = math.hypot(state['x'] - target[0], state['y'] - target[1])
            assert result.status == 4, str(result)
            assert distance < 0.15, distance
            assert abs(received['/odom_nav'].twist.twist.angular.z) < 1e-9
            assert abs(received['/cmd_vel_collision'].twist.angular.z) < 1e-9
            print(f'PASS: actual MPPI FollowPath spin={rate}, XY error={distance:.3f} m, arbitrary goal yaw accepted', flush=True)
        print('PASS: fake/non-fake launch configuration', flush=True)
    except BaseException:
        print('Final plant state:', state, flush=True)
        print('Last command:', received.get('/cmd_vel_collision'), flush=True)
        for log in logs:
            log.flush()
            log.seek(0)
            print(log.read()[-10000:])
        raise
    finally:
        for process in processes:
            process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        node.destroy_node()
        rclpy.shutdown()
        for log in logs:
            log.close()
        temp.cleanup()


if __name__ == '__main__':
    main()
