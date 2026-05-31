#!/usr/bin/env python3
"""Estimate and publish the AMCL initial pose for simulation launches."""

from collections import deque
import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def parameter_bool(value):
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


def read_pgm(path):
    with open(path, "rb") as pgm:
        tokens = []
        while len(tokens) < 4:
            line = pgm.readline()
            if not line:
                raise RuntimeError(f"Invalid PGM header in {path}")
            line = line.split(b"#", 1)[0].strip()
            if line:
                tokens.extend(line.split())

        magic = tokens[0].decode("ascii")
        width = int(tokens[1])
        height = int(tokens[2])
        maxval = int(tokens[3])

        if magic == "P5":
            dtype = np.uint8 if maxval < 256 else np.dtype(">u2")
            data = np.frombuffer(pgm.read(width * height * np.dtype(dtype).itemsize), dtype=dtype)
        elif magic == "P2":
            data = np.fromstring(pgm.read().decode("ascii"), sep=" ", dtype=np.uint16)
        else:
            raise RuntimeError(f"Unsupported map image type {magic}")

    if data.size != width * height:
        raise RuntimeError(f"Map image size mismatch in {path}")
    return data.astype(np.uint8).reshape((height, width))


def compute_distance_cells(occupied):
    height, width = occupied.shape
    dist = np.full((height, width), np.inf, dtype=np.float32)
    queue = deque()

    occupied_y, occupied_x = np.nonzero(occupied)
    for py, px in zip(occupied_y, occupied_x):
        dist[py, px] = 0.0
        queue.append((py, px))

    neighbors = (
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, 1.4142),
        (-1, 1, 1.4142),
        (1, -1, 1.4142),
        (1, 1, 1.4142),
    )

    while queue:
        py, px = queue.popleft()
        base = float(dist[py, px])
        for dy, dx, cost in neighbors:
            ny = py + dy
            nx = px + dx
            if 0 <= nx < width and 0 <= ny < height:
                candidate = base + cost
                if candidate < dist[ny, nx]:
                    dist[ny, nx] = candidate
                    queue.append((ny, nx))
    return dist


class ScanMapMatcher:
    def __init__(
        self,
        map_yaml,
        max_range,
        obstacle_sigma,
        min_inside_ratio,
        coarse_xy_step,
        coarse_yaw_samples,
        top_candidates,
        refine_xy_radius,
        refine_xy_step,
        refine_yaw_radius,
        refine_yaw_step,
    ):
        self.max_range = max_range
        self.obstacle_sigma = obstacle_sigma
        self.min_inside_ratio = min_inside_ratio
        self.coarse_xy_step = coarse_xy_step
        self.coarse_yaw_samples = coarse_yaw_samples
        self.top_candidates = top_candidates
        self.refine_xy_radius = refine_xy_radius
        self.refine_xy_step = refine_xy_step
        self.refine_yaw_radius = refine_yaw_radius
        self.refine_yaw_step = refine_yaw_step

        yaml_path = Path(map_yaml).expanduser()
        with open(yaml_path, "r", encoding="utf-8") as stream:
            info = yaml.safe_load(stream)

        image_path = Path(info["image"])
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path

        self.image = read_pgm(image_path)
        self.resolution = float(info["resolution"])
        self.origin_x = float(info["origin"][0])
        self.origin_y = float(info["origin"][1])
        self.height, self.width = self.image.shape

        self.occupied = self.image < 50
        self.free = self.image > 180
        self.distance = compute_distance_cells(self.occupied)

    def estimate(self, scan_msg):
        ranges = np.asarray(scan_msg.ranges, dtype=np.float32)
        angles = scan_msg.angle_min + np.arange(ranges.size, dtype=np.float32) * scan_msg.angle_increment
        valid = np.isfinite(ranges)
        valid &= ranges > max(float(scan_msg.range_min), 0.15)
        valid &= ranges < min(float(scan_msg.range_max), self.max_range)
        ranges = ranges[valid]
        angles = angles[valid]
        if ranges.size < 40:
            raise RuntimeError("Not enough valid laser rays for relocalization")

        coarse_ranges, coarse_angles = self.downsample(ranges, angles, 180)
        refine_ranges, refine_angles = self.downsample(ranges, angles, 360)

        candidates = self.coarse_candidates(coarse_ranges, coarse_angles)
        if not candidates:
            raise RuntimeError("No valid relocalization candidates")

        best_pose = None
        best_score = -1.0
        for _, x, y, yaw in candidates:
            pose, score = self.refine_candidate(x, y, yaw, refine_ranges, refine_angles)
            if score > best_score:
                best_pose = pose
                best_score = score

        if best_pose is None:
            raise RuntimeError("Relocalization refinement failed")
        return (*best_pose, best_score)

    @staticmethod
    def downsample(ranges, angles, max_count):
        if ranges.size <= max_count:
            return ranges, angles
        indices = np.linspace(0, ranges.size - 1, max_count).astype(np.int32)
        return ranges[indices], angles[indices]

    def coarse_candidates(self, ranges, angles):
        free_y, free_x = np.nonzero(self.free[:: self.coarse_xy_step, :: self.coarse_xy_step])
        pxs = free_x * self.coarse_xy_step
        pys = free_y * self.coarse_xy_step
        yaws = np.linspace(-math.pi, math.pi, self.coarse_yaw_samples, endpoint=False)

        candidates = []
        for px, py in zip(pxs, pys):
            x = self.origin_x + (float(px) + 0.5) * self.resolution
            y = self.origin_y + (self.height - float(py) - 0.5) * self.resolution
            for yaw in yaws:
                score = self.score_pose(x, y, float(yaw), ranges, angles)
                if score > 0.0:
                    candidates.append((score, x, y, float(yaw)))

        candidates.sort(reverse=True, key=lambda item: item[0])
        return candidates[: self.top_candidates]

    def refine_candidate(self, x, y, yaw, ranges, angles):
        best_pose = (x, y, yaw)
        best_score = -1.0

        xy_offsets = np.arange(
            -self.refine_xy_radius,
            self.refine_xy_radius + self.refine_xy_step * 0.5,
            self.refine_xy_step,
        )
        yaw_offsets = np.arange(
            -self.refine_yaw_radius,
            self.refine_yaw_radius + self.refine_yaw_step * 0.5,
            self.refine_yaw_step,
        )

        for dx in xy_offsets:
            for dy in xy_offsets:
                rx = x + float(dx)
                ry = y + float(dy)
                if not self.is_free_world(rx, ry):
                    continue
                for dyaw in yaw_offsets:
                    ryaw = normalize_angle(yaw + float(dyaw))
                    score = self.score_pose(rx, ry, ryaw, ranges, angles)
                    if score > best_score:
                        best_score = score
                        best_pose = (rx, ry, ryaw)
        return best_pose, best_score

    def is_free_world(self, x, y):
        px, py = self.world_to_pixel_scalar(x, y)
        return 0 <= px < self.width and 0 <= py < self.height and bool(self.free[py, px])

    def score_pose(self, x, y, yaw, ranges, angles):
        endpoints_x = x + ranges * np.cos(yaw + angles)
        endpoints_y = y + ranges * np.sin(yaw + angles)
        px = ((endpoints_x - self.origin_x) / self.resolution).astype(np.int32)
        py = (self.height - 1 - ((endpoints_y - self.origin_y) / self.resolution)).astype(np.int32)
        inside = (px >= 0) & (px < self.width) & (py >= 0) & (py < self.height)
        inside_ratio = float(np.count_nonzero(inside)) / float(ranges.size)
        if inside_ratio < self.min_inside_ratio:
            return -1.0

        distances = self.distance[py[inside], px[inside]] * self.resolution
        distances = distances[np.isfinite(distances)]
        if distances.size < 20:
            return -1.0

        close = np.exp(-np.square(distances / self.obstacle_sigma))
        return float(np.mean(close) + 0.25 * np.mean(distances < 0.12) + 0.10 * inside_ratio)

    def world_to_pixel_scalar(self, x, y):
        px = int((x - self.origin_x) / self.resolution)
        py = int(self.height - 1 - ((y - self.origin_y) / self.resolution))
        return px, py


class AutoInitialPose(Node):
    def __init__(self):
        super().__init__("auto_initial_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", 0.02)
        self.declare_parameter("y", 0.03)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("publish_count", 6)
        self.declare_parameter("publish_period", 0.5)
        self.declare_parameter("covariance_x", 0.25)
        self.declare_parameter("covariance_y", 0.25)
        self.declare_parameter("covariance_yaw", 0.06853891945200942)
        self.declare_parameter("relocalize", True)
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_timeout", 8.0)
        self.declare_parameter("max_range", 6.0)
        self.declare_parameter("obstacle_sigma", 0.08)
        self.declare_parameter("min_inside_ratio", 0.55)
        self.declare_parameter("min_match_score", 0.18)
        self.declare_parameter("coarse_xy_step_cells", 3)
        self.declare_parameter("coarse_yaw_samples", 96)
        self.declare_parameter("top_candidates", 8)
        self.declare_parameter("refine_xy_radius", 0.18)
        self.declare_parameter("refine_xy_step", 0.03)
        self.declare_parameter("refine_yaw_radius", 0.22)
        self.declare_parameter("refine_yaw_step", 0.02)

        self.frame_id = self.get_parameter("frame_id").value
        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.yaw = float(self.get_parameter("yaw").value)
        self.publish_count = int(self.get_parameter("publish_count").value)
        self.publish_period = float(self.get_parameter("publish_period").value)
        self.relocalize = parameter_bool(self.get_parameter("relocalize").value)
        self.map_yaml = str(self.get_parameter("map_yaml").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.min_match_score = float(self.get_parameter("min_match_score").value)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.sent = 0
        self.timer = None
        self.scan_sub = None
        self.timeout_timer = None
        self.matching_started = False

        if self.relocalize and self.map_yaml:
            self.scan_sub = self.create_subscription(
                LaserScan, self.scan_topic, self.scan_callback, 10
            )
            timeout = float(self.get_parameter("scan_timeout").value)
            self.timeout_timer = self.create_timer(timeout, self.scan_timeout)
            self.get_logger().info(
                f"Waiting for {self.scan_topic} to relocalize against {self.map_yaml}"
            )
        else:
            self.start_publishing(self.x, self.y, self.yaw, "fixed")

    def scan_callback(self, scan_msg):
        if self.matching_started:
            return
        self.matching_started = True
        if self.timeout_timer is not None:
            self.timeout_timer.cancel()

        try:
            matcher = ScanMapMatcher(
                self.map_yaml,
                float(self.get_parameter("max_range").value),
                float(self.get_parameter("obstacle_sigma").value),
                float(self.get_parameter("min_inside_ratio").value),
                int(self.get_parameter("coarse_xy_step_cells").value),
                int(self.get_parameter("coarse_yaw_samples").value),
                int(self.get_parameter("top_candidates").value),
                float(self.get_parameter("refine_xy_radius").value),
                float(self.get_parameter("refine_xy_step").value),
                float(self.get_parameter("refine_yaw_radius").value),
                float(self.get_parameter("refine_yaw_step").value),
            )
            x, y, yaw, score = matcher.estimate(scan_msg)
            if score < self.min_match_score:
                raise RuntimeError(f"Low match score {score:.3f}")
            self.get_logger().info(
                f"Relocalized initial pose: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}, score={score:.3f}"
            )
            self.start_publishing(x, y, yaw, "relocalized")
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(
                f"Relocalization failed ({exc}); falling back to fixed initial pose"
            )
            self.start_publishing(self.x, self.y, self.yaw, "fixed fallback")

        if self.scan_sub is not None:
            self.destroy_subscription(self.scan_sub)
            self.scan_sub = None

    def scan_timeout(self):
        if self.matching_started:
            return
        self.matching_started = True
        self.get_logger().warn("Timed out waiting for scan; using fixed initial pose")
        self.start_publishing(self.x, self.y, self.yaw, "fixed timeout")

    def start_publishing(self, x, y, yaw, source):
        self.x = float(x)
        self.y = float(y)
        self.yaw = normalize_angle(float(yaw))
        self.sent = 0
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(self.publish_period, self.publish_pose)
        self.get_logger().info(
            f"Auto initial pose ({source}): frame={self.frame_id}, x={self.x:.3f}, "
            f"y={self.y:.3f}, yaw={self.yaw:.3f}"
        )

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.pose.covariance[0] = float(self.get_parameter("covariance_x").value)
        msg.pose.covariance[7] = float(self.get_parameter("covariance_y").value)
        msg.pose.covariance[35] = float(self.get_parameter("covariance_yaw").value)
        self.pub.publish(msg)
        self.sent += 1
        if self.sent >= self.publish_count:
            self.get_logger().info("Finished publishing initial pose")
            self.timer.cancel()


def main():
    rclpy.init()
    node = AutoInitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
