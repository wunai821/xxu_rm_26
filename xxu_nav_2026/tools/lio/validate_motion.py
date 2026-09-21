#!/usr/bin/env python3
"""Validate odometry frame/velocity consistency against Gazebo motion truth.

The simulation launch must already be running.  Commands are injected at the
same point as Nav2's collision-monitor output, so this also exercises the
fake velocity frame and the final safety gate.
"""

import argparse
import json
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_xy_yaw(pose):
    q = pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return pose.position.x, pose.position.y, yaw


def relative_truth(first, current):
    dx = current[0] - first[0]
    dy = current[1] - first[1]
    c = math.cos(first[2])
    s = math.sin(first[2])
    return c * dx + s * dy, -s * dx + c * dy, wrap(current[2] - first[2])


class MotionValidator:
    def __init__(self):
        self.node = rclpy.create_node("lio_motion_validator")
        self.command_pub = self.node.create_publisher(
            TwistStamped, "/cmd_vel_collision", 10
        )
        self.odom = None
        self.truth = None
        self.truth_twist = None
        self.previous_truth = None
        self.joints = None
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(Odometry, "/odom", self._odom_callback, qos)
        self.node.create_subscription(PoseArray, "/world/complex_mapping/dynamic_pose/info", self._truth_callback, qos)
        self.node.create_subscription(JointState, "/joint_states", self._joint_callback, qos)

    def _odom_callback(self, message):
        self.odom = (stamp_seconds(message.header.stamp),) + pose_xy_yaw(message.pose.pose) + (
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.angular.z,
            message.header.frame_id,
            message.child_frame_id,
        )

    def _truth_callback(self, message):
        if message.poses:
            current = (stamp_seconds(message.header.stamp),) + pose_xy_yaw(message.poses[0])
            if self.previous_truth is not None:
                dt = current[0] - self.previous_truth[0]
                if 1.0e-6 < dt <= 1.0:
                    dx = current[1] - self.previous_truth[1]
                    dy = current[2] - self.previous_truth[2]
                    c = math.cos(current[3])
                    s = math.sin(current[3])
                    self.truth_twist = (
                        c * dx / dt + s * dy / dt,
                        -s * dx / dt + c * dy / dt,
                        wrap(current[3] - self.previous_truth[3]) / dt,
                    )
            self.previous_truth = current
            self.truth = current

    def _joint_callback(self, message):
        values = dict(zip(message.name, message.velocity))
        self.joints = {
            name: values.get(name, 0.0)
            for name in ("wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint", "gimbal_joint")
        }

    def spin(self, seconds=0.05):
        rclpy.spin_once(self.node, timeout_sec=seconds)

    def wait_ready(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin()
            if self.odom is not None and self.truth is not None and self.joints is not None:
                return True
        return False

    def publish_command(self, vx, vy, wz):
        message = TwistStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "gimbal_yaw_fake"
        message.twist.linear.x = float(vx)
        message.twist.linear.y = float(vy)
        message.twist.angular.z = float(wz)
        self.command_pub.publish(message)

    def collect(self, vx, vy, wz, duration, settle=0.5):
        self.publish_command(0.0, 0.0, 0.0)
        deadline = time.monotonic() + settle
        while time.monotonic() < deadline:
            self.spin()
        start_odom = self.odom
        start_truth = self.truth
        samples = []
        deadline = time.monotonic() + duration
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self.publish_command(vx, vy, wz)
                next_publish = now + 0.03
            self.spin(0.01)
            if self.odom is not None and self.truth is not None:
                odom = self.odom
                truth = self.truth
                truth_relative = relative_truth(start_truth[1:4], truth[1:4])
                samples.append((odom, truth_relative, dict(self.joints or {}), self.truth_twist))

        self.publish_command(0.0, 0.0, 0.0)
        for _ in range(20):
            self.spin(0.01)
        if not samples:
            raise RuntimeError("no synchronized odom/truth samples were received")

        last_odom = samples[-1][0]
        last_truth = samples[-1][1]
        odom_delta = (last_odom[1] - start_odom[1], last_odom[2] - start_odom[2], wrap(last_odom[3] - start_odom[3]))
        twist_x = [sample[0][4] for sample in samples]
        twist_y = [sample[0][5] for sample in samples]
        twist_w = [sample[0][6] for sample in samples]
        wheel_max = [
            max(abs(value) for name, value in sample[2].items() if "wheel_" in name)
            for sample in samples if sample[2]
        ]
        truth_vx = [sample[3][0] for sample in samples if sample[3] is not None]
        truth_vy = [sample[3][1] for sample in samples if sample[3] is not None]
        truth_wz = [sample[3][2] for sample in samples if sample[3] is not None]
        return {
            "command": {"vx": vx, "vy": vy, "wz": wz},
            "duration_s": duration,
            "samples": len(samples),
            "odom_frame": last_odom[7],
            "odom_child_frame": last_odom[8],
            "odom_delta": {"x": odom_delta[0], "y": odom_delta[1], "yaw": odom_delta[2]},
            "truth_delta": {"x": last_truth[0], "y": last_truth[1], "yaw": last_truth[2]},
            "final_pose_error": {
                "xy": math.hypot(odom_delta[0] - last_truth[0], odom_delta[1] - last_truth[1]),
                "yaw": abs(wrap(odom_delta[2] - last_truth[2])),
            },
            "odom_twist_median": {
                "x": statistics.median(twist_x),
                "y": statistics.median(twist_y),
                "wz": statistics.median(twist_w),
            },
            "truth_twist_median": {
                "x": statistics.median(truth_vx) if truth_vx else None,
                "y": statistics.median(truth_vy) if truth_vy else None,
                "wz": statistics.median(truth_wz) if truth_wz else None,
            },
            "wheel_speed_max_rad_s": max(wheel_max) if wheel_max else None,
            "gimbal_speed_median_rad_s": statistics.median([
                abs(sample[2].get("gimbal_joint", 0.0)) for sample in samples
            ]),
        }

    def close(self):
        self.node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", choices=("static", "translate", "rotate", "gyro_translate", "sweep"))
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--speed", type=float, default=0.2)
    parser.add_argument("--angular-speed", type=float, default=0.5)
    args = parser.parse_args()

    rclpy.init()
    validator = MotionValidator()
    try:
        if not validator.wait_ready():
            raise RuntimeError("odom, Gazebo truth, or joint_states did not become ready")
        if args.scenario == "static":
            result = validator.collect(0.0, 0.0, 0.0, args.duration)
        elif args.scenario in ("translate", "gyro_translate"):
            result = validator.collect(args.speed, 0.0, 0.0, args.duration)
        elif args.scenario == "rotate":
            result = validator.collect(0.0, 0.0, args.angular_speed, args.duration)
        else:
            speeds = (0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60)
            result = {
                "scenario": "gyro_translate",
                "sweep": [validator.collect(speed, 0.0, 0.0, args.duration, settle=0.35) for speed in speeds],
            }
        result["scenario"] = args.scenario
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        validator.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
