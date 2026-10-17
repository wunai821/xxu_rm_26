#!/usr/bin/env python3
"""Read-only per-layer timestamp/receipt recorder. Does not command motion."""
import argparse
import csv
import json
import math
from pathlib import Path
import signal
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, JointState, PointCloud2, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('lio_input_timing_recorder', parameter_overrides=[Parameter('use_sim_time', value=True)])
    stream = (args.output/'samples.csv').open('x', newline='')
    writer = csv.writer(stream)
    writer.writerow(['wall_time','monotonic','receipt_ros','topic','stamp','stamp_gap','receipt_gap','header_latency','x','y','z','wx','wy','wz'])
    last = {}; stats = {}; stopped = False
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    def receive(m, topic):
        wall = time.time(); mono = time.monotonic(); ros = node.get_clock().now().nanoseconds/1e9
        t = m.header.stamp.sec+m.header.stamp.nanosec/1e9
        prior = last.get(topic); dt = t-prior[0] if prior else None; dr = mono-prior[1] if prior else None
        last[topic] = (t, mono)
        s = stats.setdefault(topic, {'count':0,'first_stamp':t,'max_stamp_gap':0.,'max_receipt_gap':0.,'max_header_latency':0.,'gaps_over_50ms':0,'gaps_over_250ms':0,'backwards':0})
        s['count'] += 1; s['last_stamp'] = t
        if dt is not None:
            s['max_stamp_gap'] = max(s['max_stamp_gap'],dt); s['max_receipt_gap'] = max(s['max_receipt_gap'],dr)
            s['gaps_over_50ms'] += int(dt > .05+1e-9); s['gaps_over_250ms'] += int(dt > .25+1e-9); s['backwards'] += int(dt < 0)
        s['max_header_latency'] = max(s['max_header_latency'],ros-t)
        values = [math.nan]*6
        if isinstance(m, Imu):
            values = [m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]
        elif isinstance(m, JointState) and 'gimbal_joint' in m.name:
            i=m.name.index('gimbal_joint'); values[:2]=[m.position[i],m.velocity[i]]
        elif isinstance(m, Odometry):
            p=m.pose.pose.position; v=m.twist.twist
            values=[p.x,p.y,p.z,v.linear.x,v.linear.y,v.angular.z]
        writer.writerow([wall,mono,ros,topic,t,dt,dr,ros-t,*values])
    for topic, cls in [('/imu_raw',Imu),('/imu',Imu),('/imu_lio_compensated',Imu),('/joint_states',JointState),('/mid360/livox_points',PointCloud2),('/mid360/livox_points_compensated',PointCloud2),('/odom',Odometry),('/odom_nav',Odometry),('/scan',LaserScan)]:
        node.create_subscription(cls,topic,lambda m,t=topic:receive(m,t),qos)
    fault=[]
    node.create_subscription(Bool,'/small_point_lio/imu_time_fault',lambda m:fault.append({'wall_time':time.time(),'fault':m.data}),QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def save():
        stream.flush()
        path=args.output/'health.json'; tmp=path.with_suffix('.tmp')
        tmp.write_text(json.dumps({'wall_time':time.time(),'read_only':True,'stats':stats,'fault_states':fault},indent=2)+'\n'); tmp.replace(path)
    node.create_timer(1.,save)
    def stop(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    try:
        while not stopped: rclpy.spin_once(node,timeout_sec=.05)
    finally:
        save(); stream.close(); node.destroy_node(); rclpy.shutdown()


if __name__=='__main__':main()
