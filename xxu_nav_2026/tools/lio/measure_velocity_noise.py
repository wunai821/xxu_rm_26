#!/usr/bin/env python3
"""Passively record unique odometry messages for a stationary noise test.

Uses simulation time for warmup and duration. Does not publish commands.
Gazebo truth should be recorded independently to establish actual stationarity.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--duration", type=float, default=60)
    parser.add_argument("--warmup", type=float, default=15)
    parser.add_argument("--wall-timeout", type=float, default=240)
    parser.add_argument("--state-topic", help="Also record an optional estimator-state velocity stream")
    args = parser.parse_args()
    if args.duration <= 0 or args.warmup < 0 or args.wall_timeout <= 0:
        parser.error("duration and wall timeout must be positive; warmup must be nonnegative")
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("odom_velocity_noise_recorder", parameter_overrides=[
        Parameter("use_sim_time", value=True)])
    columns = ["t", "receipt_t", "x", "y", "yaw", "vx", "vy", "wz",
               "frame", "child_frame", "cov_vx", "cov_vy", "cov_wz"]
    streams = {}
    writers = {}
    counts = {}
    last_stamp = {}
    rejected = {}
    first = None
    newest = None

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    odom_keys = ["odom", "odom_nav"] + (["odom_velocity_state"] if args.state_topic else [])
    for name in (*odom_keys, "joints", "commands"):
        streams[name] = (args.output / (name + ".csv")).open("w")
        writers[name] = csv.writer(streams[name])
        counts[name] = 0
    for key in odom_keys:
        writers[key].writerow(columns)
    writers["joints"].writerow(["t", "gimbal_wz", "wheel_abs_max"])
    writers["commands"].writerow(["t", "vx", "vy", "wz"])

    def odometry(msg, key):
        nonlocal first, newest
        t = seconds(msg.header.stamp)
        if t <= last_stamp.get(key, -math.inf):
            rejected[key] = rejected.get(key, 0) + 1
            return
        last_stamp[key] = t
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        writers[key].writerow([t, now(), p.x, p.y, yaw, v.linear.x, v.linear.y,
                              v.angular.z, msg.header.frame_id, msg.child_frame_id,
                              msg.twist.covariance[0], msg.twist.covariance[7],
                              msg.twist.covariance[35]])
        counts[key] += 1
        if key == "odom":
            if first is None:
                first = t
                print(f"Odometry ready at {t:.3f}; warmup={args.warmup}s, measurement={args.duration}s", flush=True)
            newest = t

    def joints(msg):
        values = dict(zip(msg.name, msg.velocity))
        wheels = [abs(v) for k, v in values.items() if k.startswith("wheel_")]
        writers["joints"].writerow([seconds(msg.header.stamp), values.get("gimbal_joint", math.nan),
                                    max(wheels) if wheels else math.nan])
        counts["joints"] += 1

    def command(msg):
        v = msg.twist
        writers["commands"].writerow([seconds(msg.header.stamp), v.linear.x, v.linear.y, v.angular.z])
        counts["commands"] += 1

    for key in odom_keys:
        topic = args.state_topic if key == "odom_velocity_state" else "/" + key
        node.create_subscription(Odometry, topic,
                                 lambda m, k=key: odometry(m, k), qos_profile_sensor_data)
    node.create_subscription(JointState, "/joint_states", joints, qos_profile_sensor_data)
    node.create_subscription(TwistStamped, "/cmd_vel", command, qos_profile_sensor_data)
    wall_start = time.monotonic()
    complete = False
    try:
        while rclpy.ok() and time.monotonic() - wall_start < args.wall_timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if first is not None and newest >= first + args.warmup + args.duration:
                complete = True
                break
        manifest = {"complete": complete, "first_odom_t": first, "last_odom_t": newest,
                    "measurement_start_t": first + args.warmup if first is not None else None,
                    "measurement_duration_s": args.duration, "warmup_s": args.warmup,
                    "wall_elapsed_s": time.monotonic() - wall_start,
                    "counts": counts, "duplicate_or_nonmonotonic_rejected": rejected,
                    "method": "One row per received message; odometry stamps strictly increasing; no command publication."}
        (args.output / "measurement.json").write_text(json.dumps(manifest, indent=2) + "\n")
        print(json.dumps(manifest, indent=2), flush=True)
        if not complete:
            raise RuntimeError("Simulation-time measurement did not finish before wall timeout")
        if args.state_topic and not counts["odom_velocity_state"]:
            raise RuntimeError("No estimator-state diagnostic messages received")
    finally:
        for stream in streams.values():
            stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
