#!/usr/bin/env python3
"""Report AMCL initial-particle coverage and the first published particle cloud.

Nav2 AMCL does not expose the particle set immediately after ``pf_init``.
This diagnostic therefore records the exact Gaussian requested by
``/initialpose`` as the pre-scan support, and labels the first
``/particle_cloud`` as the first observable post-scan set.  It never treats
the latter as an exact copy of the former.
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import ParticleCloud
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def interval_probability(mean, sigma, lower, upper):
    if sigma <= 0.0:
        return 1.0 if lower <= mean <= upper else 0.0
    scale = math.sqrt(2.0) * sigma
    return 0.5 * (math.erf((upper - mean) / scale) - math.erf((lower - mean) / scale))


class AmclParticleDiagnostic(Node):
    def __init__(self):
        super().__init__("amcl_particle_diagnostic")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("particle_cloud_topic", "/particle_cloud")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("reference_enabled", False)
        self.declare_parameter("reference_x", 0.0)
        self.declare_parameter("reference_y", 0.0)
        self.declare_parameter("reference_yaw", 0.0)
        self.declare_parameter("reference_xy_radius", 0.5)
        self.declare_parameter("reference_yaw_radius", math.radians(15.0))
        self.declare_parameter("report_timeout", 20.0)

        self.reference_enabled = bool(self.get_parameter("reference_enabled").value)
        self.reference_x = float(self.get_parameter("reference_x").value)
        self.reference_y = float(self.get_parameter("reference_y").value)
        self.reference_yaw = float(self.get_parameter("reference_yaw").value)
        self.reference_xy_radius = float(self.get_parameter("reference_xy_radius").value)
        self.reference_yaw_radius = float(self.get_parameter("reference_yaw_radius").value)
        self.initial_pose = None
        self.scan_count_after_initial = 0
        self.first_scan_reported = False
        self.first_cloud_reported = False
        self.timeout_reported = False
        self.start_time = self.get_clock().now()

        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initial_pose_topic").value),
            self.initial_pose_callback,
            10,
        )
        self.create_subscription(
            ParticleCloud,
            str(self.get_parameter("particle_cloud_topic").value),
            self.particle_cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.timeout_timer = self.create_timer(1.0, self.timeout_callback)

    def initial_pose_callback(self, msg):
        self.initial_pose = msg
        covariance = msg.pose.covariance
        sigma_x = math.sqrt(max(0.0, float(covariance[0])))
        sigma_y = math.sqrt(max(0.0, float(covariance[7])))
        sigma_yaw = math.sqrt(max(0.0, float(covariance[35])))
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        stamp = stamp_seconds(msg.header.stamp)
        self.get_logger().info(
            "AMCL init pre-scan request: "
            f"msg_stamp={stamp:.9f}, x={msg.pose.pose.position.x:.3f}, "
            f"y={msg.pose.pose.position.y:.3f}, yaw={yaw:.3f}, "
            f"covariance=[{covariance[0]:.6f}, {covariance[7]:.6f}, {covariance[35]:.6f}], "
            f"sigma=[{sigma_x:.3f} m, {sigma_y:.3f} m, {math.degrees(sigma_yaw):.2f} deg]"
        )
        if self.reference_enabled:
            probability = self.reference_probability(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw,
                sigma_x,
                sigma_y,
                sigma_yaw,
            )
            self.get_logger().info(
                "AMCL init pre-scan Gaussian coverage near reference: "
                f"reference=({self.reference_x:.3f}, {self.reference_y:.3f}, "
                f"{self.reference_yaw:.3f}), "
                f"P(|dx|<={self.reference_xy_radius:.3f}, "
                f"|dy|<={self.reference_xy_radius:.3f}, "
                f"|dyaw|<={math.degrees(self.reference_yaw_radius):.1f}deg)="
                f"{probability:.4f}"
            )
        self.get_logger().info(
            "AMCL init pre-scan particle set is internal to Nav2; "
            "the values above are the exact requested Gaussian support, "
            "not a sampled-particle observation"
        )

    def reference_probability(self, x, y, yaw, sigma_x, sigma_y, sigma_yaw):
        if not self.reference_enabled:
            return float("nan")
        # The configured covariance is diagonal for these experiments. The
        # product is consequently a useful coverage diagnostic, not a claim
        # about the exact random sample count inside AMCL.
        px = interval_probability(
            x, sigma_x, self.reference_x - self.reference_xy_radius,
            self.reference_x + self.reference_xy_radius,
        )
        py = interval_probability(
            y, sigma_y, self.reference_y - self.reference_xy_radius,
            self.reference_y + self.reference_xy_radius,
        )
        yaw_error = normalize_angle(yaw - self.reference_yaw)
        pyaw = interval_probability(
            yaw_error, sigma_yaw, -self.reference_yaw_radius, self.reference_yaw_radius,
        )
        return px * py * pyaw

    def scan_callback(self, msg):
        if self.initial_pose is None:
            return
        self.scan_count_after_initial += 1
        if not self.first_scan_reported:
            self.first_scan_reported = True
            self.get_logger().info(
                "AMCL init observation: first scan after initial-pose request: "
                f"scan_stamp={stamp_seconds(msg.header.stamp):.9f}, "
                f"ranges={len(msg.ranges)}"
            )

    def particle_cloud_callback(self, msg):
        if self.initial_pose is None or self.first_cloud_reported:
            return
        self.first_cloud_reported = True
        particles = list(msg.particles)
        if not particles:
            self.get_logger().warn("AMCL first observable particle cloud is empty")
            return
        weights = [max(0.0, float(p.weight)) for p in particles]
        total_weight = sum(weights)
        if total_weight <= 0.0:
            weights = [1.0] * len(particles)
            total_weight = float(len(particles))
        xs = [float(p.pose.position.x) for p in particles]
        ys = [float(p.pose.position.y) for p in particles]
        yaws = [yaw_from_quaternion(p.pose.orientation) for p in particles]
        mean_x = sum(w * x for w, x in zip(weights, xs)) / total_weight
        mean_y = sum(w * y for w, y in zip(weights, ys)) / total_weight
        mean_yaw = math.atan2(
            sum(w * math.sin(yaw) for w, yaw in zip(weights, yaws)),
            sum(w * math.cos(yaw) for w, yaw in zip(weights, yaws)),
        )
        sigma_x = math.sqrt(sum(w * (x - mean_x) ** 2 for w, x in zip(weights, xs)) / total_weight)
        sigma_y = math.sqrt(sum(w * (y - mean_y) ** 2 for w, y in zip(weights, ys)) / total_weight)
        sigma_yaw = math.sqrt(
            sum(w * normalize_angle(yaw - mean_yaw) ** 2 for w, yaw in zip(weights, yaws))
            / total_weight
        )
        self.get_logger().info(
            "AMCL first observable particle cloud (post-scan): "
            f"cloud_stamp={stamp_seconds(msg.header.stamp):.9f}, count={len(particles)}, "
            f"weighted_mean=({mean_x:.3f}, {mean_y:.3f}, {mean_yaw:.3f}), "
            f"sigma=({sigma_x:.3f} m, {sigma_y:.3f} m, {math.degrees(sigma_yaw):.2f} deg)"
        )
        if self.reference_enabled:
            within = 0.0
            for weight, x, y, yaw in zip(weights, xs, ys, yaws):
                if math.hypot(x - self.reference_x, y - self.reference_y) <= self.reference_xy_radius and \
                        abs(normalize_angle(yaw - self.reference_yaw)) <= self.reference_yaw_radius:
                    within += weight
            self.get_logger().info(
                "AMCL first observable particle cloud coverage near reference: "
                f"fraction={within / total_weight:.4f} "
                f"(xy<={self.reference_xy_radius:.3f} m, "
                f"yaw<={math.degrees(self.reference_yaw_radius):.1f} deg)"
            )

    def timeout_callback(self):
        if self.timeout_reported or self.initial_pose is not None:
            return
        timeout = float(self.get_parameter("report_timeout").value)
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1.0e-9
        if elapsed >= timeout:
            self.timeout_reported = True
            self.get_logger().warn(
                f"No /initialpose received within {timeout:.1f} s; "
                "pre-scan particle coverage cannot be evaluated"
            )


def main():
    rclpy.init()
    node = AmclParticleDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
