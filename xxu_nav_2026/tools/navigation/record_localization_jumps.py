#!/usr/bin/env python3
"""Read-only TF/AMCL/scan recorder. Never sends goals, poses or velocities."""
import argparse
import json
import math
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.msg import ParticleCloud
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=0)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'particles').mkdir()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('localization_jump_recorder', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    streams = {key: (args.output / (key + '.jsonl')).open('w')
               for key in ['tf', 'odom', 'amcl', 'scan', 'particles', 'initialpose', 'jumps']}
    latest = {}
    previous = {}
    publishers = {}
    jumps = []
    counts = {key: 0 for key in streams}
    stopped = False
    last_particles = -math.inf

    def now():
        return node.get_clock().now().nanoseconds / 1e9

    def write(key, record):
        record = {'receipt_ros': now(), 'wall_time': time.time(), **record}
        streams[key].write(json.dumps(record, allow_nan=False) + '\n')
        counts[key] += 1
        latest[key] = record
        return record

    def tf_callback(msg, info):
        gid_value = info.get('publisher_gid') if isinstance(info, dict) else getattr(info, 'publisher_gid', None)
        gid = bytes(gid_value).hex() if gid_value is not None else 'unavailable_in_rclpy_message_info'
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip('/')
            child = transform.child_frame_id.lstrip('/')
            key = parent + '->' + child
            if key not in ['map->odom', 'odom->base_footprint',
                           'base_footprint->gimbal_yaw_fake']:
                continue
            p = transform.transform.translation
            record = {'pair': key, 'stamp': stamp(transform),
                      'xy': [p.x, p.y], 'yaw': yaw(transform.transform.rotation),
                      'publisher_gid': gid}
            publishers.setdefault(key, set()).add(gid)
            write('tf', record)
            old = previous.get(key)
            if old:
                dt = record['stamp'] - old['stamp']
                distance = math.hypot(p.x - old['xy'][0], p.y - old['xy'][1])
                rotation = abs(angle(record['yaw'] - old['yaw']))
                suspicious = (key == 'map->odom' and 0 < dt <= 1.5 and
                              (distance > .3 or rotation > math.radians(15)))
                suspicious |= (key == 'odom->base_footprint' and 0 < dt <= .25 and
                               distance > .45)
                if suspicious:
                    event = write('jumps', {'pair': key, 'before': old, 'after': record,
                                           'dt': dt, 'translation_change_m': distance,
                                           'yaw_change_rad': rotation,
                                           'context': {k: latest.get(k) for k in
                                                       ['odom', 'amcl', 'scan', 'particles', 'initialpose']}})
                    jumps.append(event)
                    streams['jumps'].flush()
                    print(json.dumps({'jump': key, 'ros': now(), 'distance': distance,
                                      'rotation': rotation}), flush=True)
            if not old or record['stamp'] >= old['stamp']:
                previous[key] = record

    def odom_callback(msg):
        p = msg.pose.pose.position
        v = msg.twist.twist
        write('odom', {'stamp': stamp(msg), 'xy': [p.x, p.y],
                       'yaw': yaw(msg.pose.pose.orientation),
                       'velocity': [v.linear.x, v.linear.y, v.angular.z]})

    def pose_callback(msg, key):
        p = msg.pose.pose.position
        write(key, {'stamp': stamp(msg), 'xy': [p.x, p.y],
                    'yaw': yaw(msg.pose.pose.orientation),
                    'covariance': list(msg.pose.covariance)})

    def scan_callback(msg):
        ranges = np.asarray(msg.ranges)
        step = max(1, (len(ranges) - 1) // (60 - 1))
        selected = ranges[::step]
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges < msg.range_max)
        selected_valid = np.isfinite(selected) & (selected >= msg.range_min) & (selected < msg.range_max)
        write('scan', {'stamp': stamp(msg), 'frame': msg.header.frame_id,
                       'ray_count': len(ranges), 'finite_hits': int(valid.sum()),
                       'selected_60_hits': int(selected_valid.sum()),
                       'hits_under_06m': int((valid & (ranges < .6)).sum()),
                       'minimum': float(ranges[valid].min()) if valid.any() else None})

    def particles_callback(msg):
        nonlocal last_particles
        if stamp(msg) - last_particles < .2:
            return
        last_particles = stamp(msg)
        a = np.asarray([[p.pose.position.x, p.pose.position.y,
                         yaw(p.pose.orientation), p.weight] for p in msg.particles])
        if not len(a):
            return
        weights = a[:, 3]
        weights = weights / weights.sum() if weights.sum() > 0 else np.ones(len(a)) / len(a)
        mean = np.sum(a[:, :2] * weights[:, None], axis=0)
        variance = np.sum((a[:, :2] - mean) ** 2 * weights[:, None], axis=0)
        filename = f"{stamp(msg):.9f}.npz"
        np.savez_compressed(args.output / 'particles' / filename, xy_yaw_weight=a)
        write('particles', {'stamp': stamp(msg), 'count': len(a),
                            'mean_xy': mean.tolist(), 'std_xy': np.sqrt(variance).tolist(),
                            'file': filename})

    node.create_subscription(TFMessage, '/tf', tf_callback, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', odom_callback, qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                             lambda m: pose_callback(m, 'amcl'), qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                             lambda m: pose_callback(m, 'initialpose'), 10)
    node.create_subscription(LaserScan, '/scan', scan_callback, qos_profile_sensor_data)
    node.create_subscription(ParticleCloud, '/particle_cloud', particles_callback, qos_profile_sensor_data)

    def stop(*unused):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    start = flushed = time.monotonic()
    try:
        while rclpy.ok() and not stopped and (not args.duration or time.monotonic() - start < args.duration):
            rclpy.spin_once(node, timeout_sec=.05)
            if time.monotonic() - flushed > 1:
                for stream in streams.values():
                    stream.flush()
                health = {'wall_time': time.time(), 'ros_time': now(), 'counts': counts,
                          'last_tf': previous, 'latest': latest,
                          'publisher_gids': {k: sorted(v) for k, v in publishers.items()},
                          'jump_count': len(jumps)}
                (args.output / 'health.json').write_text(json.dumps(health, indent=2) + '\n')
                flushed = time.monotonic()
    finally:
        for stream in streams.values():
            stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
