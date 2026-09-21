#!/usr/bin/env python3
"""Estimate and publish the AMCL initial pose for simulation launches."""

from collections import deque
import math
from pathlib import Path
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def parameter_bool(value):
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


def stamp_seconds(stamp):
    """Return a ROS message timestamp as seconds for concise diagnostics."""
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def planar_pose(transform_stamped):
    """Extract x, y, yaw from an odom -> base transform."""
    transform = transform_stamped.transform
    rotation = transform.rotation
    yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )
    return transform.translation.x, transform.translation.y, normalize_angle(yaw)


def advance_map_pose(map_pose, reference_odom_tf, latest_odom_tf):
    """Advance map -> base from one odom timestamp to another in SE(2)."""
    map_x, map_y, map_yaw = map_pose
    ref_x, ref_y, ref_yaw = planar_pose(reference_odom_tf)
    latest_x, latest_y, latest_yaw = planar_pose(latest_odom_tf)

    # T_map_base(latest) = T_map_base(reference) *
    #                      inverse(T_odom_base(reference)) *
    #                      T_odom_base(latest)
    dx_odom = latest_x - ref_x
    dy_odom = latest_y - ref_y
    cos_ref = math.cos(ref_yaw)
    sin_ref = math.sin(ref_yaw)
    dx_base = cos_ref * dx_odom + sin_ref * dy_odom
    dy_base = -sin_ref * dx_odom + cos_ref * dy_odom

    cos_map = math.cos(map_yaw)
    sin_map = math.sin(map_yaw)
    advanced_x = map_x + cos_map * dx_base - sin_map * dy_base
    advanced_y = map_y + sin_map * dx_base + cos_map * dy_base
    advanced_yaw = normalize_angle(map_yaw + latest_yaw - ref_yaw)
    return advanced_x, advanced_y, advanced_yaw


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
        min_range,
        max_range,
        min_robot_clearance,
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
        self.min_range = min_range
        self.max_range = max_range
        self.min_robot_clearance = min_robot_clearance
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
        # Reject returns from the chassis and sensor mount. Those short returns
        # are not present in the static map and otherwise make wall-adjacent
        # poses look deceptively good.
        valid &= ranges > max(float(scan_msg.range_min), self.min_range)
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
        sampled_free = self.free[:: self.coarse_xy_step, :: self.coarse_xy_step]
        sampled_clearance = (
            self.distance[:: self.coarse_xy_step, :: self.coarse_xy_step]
            * self.resolution
            >= self.min_robot_clearance
        )
        free_y, free_x = np.nonzero(sampled_free & sampled_clearance)
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
        return (
            0 <= px < self.width
            and 0 <= py < self.height
            and bool(self.free[py, px])
            and float(self.distance[py, px]) * self.resolution
            >= self.min_robot_clearance
        )

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
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("x", 0.02)
        self.declare_parameter("y", 0.03)
        self.declare_parameter("yaw", 0.0)
        # One message is enough and avoids repeatedly resetting a converging
        # particle set. Lifecycle/service gates below prove that AMCL exists;
        # a topic subscriber count would also include diagnostics.
        self.declare_parameter("publish_count", 1)
        self.declare_parameter("publish_period", 0.5)
        self.declare_parameter("covariance_x", 0.25)
        self.declare_parameter("covariance_y", 0.25)
        self.declare_parameter("covariance_yaw", 0.06853891945200942)
        self.declare_parameter("relocalize", True)
        self.declare_parameter("defer_amcl_activation", False)
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_timeout", 8.0)
        self.declare_parameter("scan_tf_timeout", 0.25)
        self.declare_parameter("scan_warmup_count", 20)
        self.declare_parameter("min_range", 0.6)
        self.declare_parameter("max_range", 6.0)
        self.declare_parameter("min_robot_clearance", 0.3)
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
        self.defer_amcl_activation = parameter_bool(
            self.get_parameter("defer_amcl_activation").value
        )
        self.map_yaml = str(self.get_parameter("map_yaml").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.scan_tf_timeout = float(self.get_parameter("scan_tf_timeout").value)
        self.scan_warmup_count = int(self.get_parameter("scan_warmup_count").value)
        self.min_match_score = float(self.get_parameter("min_match_score").value)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.amcl_state_client = self.create_client(GetState, "/amcl/get_state")
        self.amcl_state_future = None
        self.map_state_client = self.create_client(GetState, "/map_server/get_state")
        self.map_change_client = self.create_client(ChangeState, "/map_server/change_state")
        self.amcl_change_client = self.create_client(ChangeState, "/amcl/change_state")
        self.lifecycle_future = None
        self.lifecycle_stage = "wait_services"
        self.amcl_prepared = not self.defer_amcl_activation
        self.amcl_activation_timer = None
        self.sent = 0
        self.timer = None
        self.scan_sub = None
        self.timeout_timer = None
        self.matching_started = False
        self.scans_seen = 0
        self.publish_source = ""
        self.publish_odom_reference = None
        self.scan_callback_group = ReentrantCallbackGroup()
        self.prerequisites_ready = False
        # Poll frequently enough to catch the short interval after AMCL has
        # created its initial-pose subscription but before lifecycle activation.
        # Publishing in that interval lets AMCL retain the request and handle
        # it during activation, without racing a moving simulation clock.
        self.ready_timer = self.create_timer(0.05, self.wait_for_prerequisites)

    def wait_for_prerequisites(self):
        """Do not publish an initial pose until AMCL and odometry are usable."""
        if self.defer_amcl_activation and not self.amcl_prepared:
            if not self.prepare_amcl_before_pose():
                return

        # A diagnostic subscriber may also listen to /initialpose.  Subscriber
        # count alone therefore cannot prove that AMCL exists or is ready.
        if not self.amcl_state_client.service_is_ready():
            self.get_logger().info(
                "Waiting for AMCL lifecycle service", throttle_duration_sec=5.0
            )
            return

        if self.amcl_state_future is None:
            self.amcl_state_future = self.amcl_state_client.call_async(GetState.Request())
            return
        if not self.amcl_state_future.done():
            return
        try:
            amcl_state = self.amcl_state_future.result().current_state.id
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(f"AMCL lifecycle state query failed: {exc}")
            self.amcl_state_future = None
            return
        self.amcl_state_future = None
        if amcl_state not in (2, 3):  # inactive or active
            self.get_logger().info(
                f"Waiting for AMCL to reach inactive/active state (state={amcl_state})",
                throttle_duration_sec=5.0,
            )
            return

        now = self.get_clock().now()
        if now.nanoseconds == 0 or not self.tf_buffer.can_transform(
            self.odom_frame, self.base_frame, now, timeout=Duration(seconds=0.25)
        ):
            self.get_logger().info(
                f"Waiting for current odometry transform {self.odom_frame} -> {self.base_frame}",
                throttle_duration_sec=5.0,
            )
            return

        self.prerequisites_ready = True
        self.ready_timer.cancel()
        if self.relocalize and self.map_yaml:
            self.scan_sub = self.create_subscription(
                LaserScan,
                self.scan_topic,
                self.scan_callback,
                qos_profile_sensor_data,
                callback_group=self.scan_callback_group,
            )
            timeout = float(self.get_parameter("scan_timeout").value)
            self.timeout_timer = self.create_timer(timeout, self.scan_timeout)
            self.get_logger().info(
                f"Prerequisites ready; warming up {self.scan_warmup_count} scans "
                f"from {self.scan_topic} before relocalizing"
            )
        else:
            self.start_publishing(self.x, self.y, self.yaw, "fixed")

    def prepare_amcl_before_pose(self):
        """Configure AMCL while inactive, then activate it after /initialpose.

        This is used by simulation tests to remove the AMCL callback-time race:
        a topic message received while AMCL is inactive is retained by Nav2 and
        applied during activation, so AMCL does not have to extrapolate a
        moving odometry transform while handling the message.
        """
        if self.lifecycle_stage == "wait_services":
            clients = (
                self.map_state_client,
                self.map_change_client,
                self.amcl_state_client,
                self.amcl_change_client,
            )
            if not all(client.service_is_ready() for client in clients):
                self.get_logger().info(
                    "Waiting for map_server/AMCL lifecycle services before initialization",
                    throttle_duration_sec=5.0,
                )
                return False
            self.lifecycle_stage = "query_map"

        if self.lifecycle_future is not None:
            if not self.lifecycle_future.done():
                return False
            previous_stage = self.lifecycle_stage
            try:
                response = self.lifecycle_future.result()
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().warn(f"Lifecycle request failed: {exc}")
                self.lifecycle_future = None
                return False
            self.lifecycle_future = None

            if previous_stage in (
                "query_map", "query_map_after_configure", "query_amcl",
                "query_amcl_after_configure",
            ):
                state = response.current_state.id
                if previous_stage == "query_map":
                    if state == 1:
                        self.lifecycle_stage = "configure_map"
                    elif state == 2:
                        self.lifecycle_stage = "activate_map"
                    elif state == 3:
                        self.lifecycle_stage = "query_amcl"
                    else:
                        self.lifecycle_stage = "query_map"
                elif previous_stage == "query_map_after_configure":
                    if state == 2:
                        self.lifecycle_stage = "activate_map"
                    elif state == 3:
                        self.lifecycle_stage = "query_amcl"
                    else:
                        self.lifecycle_stage = "query_map_after_configure"
                elif previous_stage == "query_amcl":
                    if state == 1:
                        self.lifecycle_stage = "configure_amcl"
                    elif state in (2, 3):
                        self.amcl_prepared = True
                        self.lifecycle_stage = "ready"
                    else:
                        self.lifecycle_stage = "query_amcl"
                else:  # query_amcl_after_configure
                    if state in (2, 3):
                        self.amcl_prepared = True
                        self.lifecycle_stage = "ready"
                    else:
                        self.lifecycle_stage = "query_amcl_after_configure"
            else:
                if not response.success:
                    self.get_logger().warn(
                        f"Lifecycle transition rejected at stage {previous_stage}"
                    )
                    self.lifecycle_stage = "wait_services"
                    return False
                if previous_stage == "configure_map":
                    self.lifecycle_stage = "query_map_after_configure"
                elif previous_stage == "activate_map":
                    self.lifecycle_stage = "query_amcl"
                elif previous_stage == "configure_amcl":
                    self.lifecycle_stage = "query_amcl_after_configure"

        if self.lifecycle_stage == "ready":
            return True
        if self.lifecycle_stage == "query_map":
            self.lifecycle_future = self.map_state_client.call_async(GetState.Request())
            return False
        if self.lifecycle_stage == "query_map_after_configure":
            self.lifecycle_future = self.map_state_client.call_async(GetState.Request())
            return False
        if self.lifecycle_stage == "query_amcl":
            self.lifecycle_future = self.amcl_state_client.call_async(GetState.Request())
            return False
        if self.lifecycle_stage == "query_amcl_after_configure":
            self.lifecycle_future = self.amcl_state_client.call_async(GetState.Request())
            return False
        if self.lifecycle_stage == "configure_map":
            request = ChangeState.Request()
            request.transition.id = 1  # TRANSITION_CONFIGURE
            self.lifecycle_future = self.map_change_client.call_async(request)
            return False
        if self.lifecycle_stage == "activate_map":
            request = ChangeState.Request()
            request.transition.id = 3  # TRANSITION_ACTIVATE
            self.lifecycle_future = self.map_change_client.call_async(request)
            return False
        if self.lifecycle_stage == "configure_amcl":
            request = ChangeState.Request()
            request.transition.id = 1  # TRANSITION_CONFIGURE
            self.lifecycle_future = self.amcl_change_client.call_async(request)
            return False
        return False

    def scan_callback(self, scan_msg):
        if self.matching_started:
            return
        self.scans_seen += 1
        if self.scans_seen <= self.scan_warmup_count:
            if self.scans_seen == self.scan_warmup_count:
                self.get_logger().info(
                    "Scan warmup complete; the next stable scan will be matched"
                )
            return
        self.matching_started = True
        if self.timeout_timer is not None:
            self.timeout_timer.cancel()

        try:
            scan_time = Time.from_msg(scan_msg.header.stamp)
            if scan_time.nanoseconds == 0:
                raise RuntimeError("Scan timestamp is zero")
            scan_odom_tf = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                scan_time,
                timeout=Duration(seconds=self.scan_tf_timeout),
            )
            self.get_logger().info(
                "Captured scan-time odometry: "
                f"scan_stamp={stamp_seconds(scan_msg.header.stamp):.9f}, "
                f"odom_stamp={stamp_seconds(scan_odom_tf.header.stamp):.9f}, "
                f"offset_ms={(stamp_seconds(scan_odom_tf.header.stamp) - stamp_seconds(scan_msg.header.stamp)) * 1000.0:.1f}"
            )
            matching_started = time.monotonic()
            matcher = ScanMapMatcher(
                self.map_yaml,
                float(self.get_parameter("min_range").value),
                float(self.get_parameter("max_range").value),
                float(self.get_parameter("min_robot_clearance").value),
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
            matching_elapsed_ms = (time.monotonic() - matching_started) * 1000.0
            if score < self.min_match_score:
                raise RuntimeError(f"Low match score {score:.3f}")
            self.get_logger().info(
                "Relocalized scan pose: "
                f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}, score={score:.3f}"
            )
            self.get_logger().info(
                "Relocalization timing: "
                f"scan_stamp={stamp_seconds(scan_msg.header.stamp):.9f}, "
                f"scan_odom_stamp={stamp_seconds(scan_odom_tf.header.stamp):.9f}, "
                f"search_ms={matching_elapsed_ms:.1f}"
            )
            self.start_publishing(
                x,
                y,
                yaw,
                "relocalized",
                odom_reference=scan_odom_tf,
            )
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

    def start_publishing(self, x, y, yaw, source, odom_reference=None):
        self.x = float(x)
        self.y = float(y)
        self.yaw = normalize_angle(float(yaw))
        self.publish_source = source
        self.publish_odom_reference = odom_reference
        self.sent = 0
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        self.get_logger().info(
            f"Auto initial pose ({source}): frame={self.frame_id}, x={self.x:.3f}, "
            f"y={self.y:.3f}, yaw={self.yaw:.3f}"
        )
        self.publish_pose()
        if self.sent < self.publish_count:
            self.timer = self.create_timer(self.publish_period, self.publish_pose)

    def publish_pose(self):
        try:
            # AMCL integrates odometry from the message stamp to its current
            # clock time. Require a real transform at that same time before
            # publishing; a historical/latest TF is not a valid substitute.
            current_time = self.get_clock().now()
            if current_time.nanoseconds == 0:
                raise RuntimeError("simulation time is not available")
            current_odom_tf = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                current_time,
                timeout=Duration(seconds=0.25),
            )
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(
                f"Latest odometry transform is unavailable; delaying initial-pose publish ({exc})"
            )
            return

        x = self.x
        y = self.y
        yaw = self.yaw
        if self.publish_odom_reference is not None:
            if (
                stamp_seconds(current_odom_tf.header.stamp)
                < stamp_seconds(self.publish_odom_reference.header.stamp)
            ):
                self.get_logger().warn(
                    "Latest odometry transform predates the matched scan; "
                    "delaying initial-pose publish"
                )
                return
            x, y, yaw = advance_map_pose(
                (self.x, self.y, self.yaw),
                self.publish_odom_reference,
                current_odom_tf,
            )
        msg = PoseWithCovarianceStamped()
        # The transform was successfully queried at this same time. Keep the
        # message stamp at the query time so AMCL's now-to-stamp integration
        # has a valid TF interval.
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = float(self.get_parameter("covariance_x").value)
        msg.pose.covariance[7] = float(self.get_parameter("covariance_y").value)
        msg.pose.covariance[35] = float(self.get_parameter("covariance_yaw").value)
        self.pub.publish(msg)
        self.sent += 1
        timing_details = ""
        if self.publish_odom_reference is not None:
            reference_stamp = stamp_seconds(self.publish_odom_reference.header.stamp)
            publish_stamp = stamp_seconds(msg.header.stamp)
            timing_details = (
                f", scan_odom_stamp={reference_stamp:.9f}, "
                f"scan_to_publish_ms={(publish_stamp - reference_stamp) * 1000.0:.1f}"
            )
        self.get_logger().info(
            "Published initial pose: "
            f"source={self.publish_source}, count={self.sent}/{self.publish_count}, "
            f"stamp={stamp_seconds(msg.header.stamp):.9f}, "
            f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}{timing_details}"
        )
        if self.sent >= self.publish_count:
            self.get_logger().info("Finished publishing initial pose")
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
            if self.defer_amcl_activation and self.amcl_activation_timer is None:
                # Leave enough time for the DDS sample to reach AMCL while it
                # is inactive. The transition is then explicit and observable.
                self.amcl_activation_timer = self.create_timer(
                    0.5, self.activate_deferred_amcl
                )

    def activate_deferred_amcl(self):
        if self.amcl_activation_timer is not None:
            self.amcl_activation_timer.cancel()
            self.amcl_activation_timer = None
        if not self.amcl_change_client.service_is_ready():
            self.get_logger().warn("AMCL change_state service disappeared; keeping AMCL inactive")
            return
        request = ChangeState.Request()
        request.transition.id = 3  # TRANSITION_ACTIVATE
        future = self.amcl_change_client.call_async(request)

        def activation_done(result):
            try:
                response = result.result()
                if response.success:
                    self.get_logger().info("AMCL activated after initial-pose delivery")
                else:
                    self.get_logger().error("AMCL activation was rejected")
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().error(f"AMCL activation failed: {exc}")

        future.add_done_callback(activation_done)


def main():
    rclpy.init()
    node = AutoInitialPose()
    # Matching is CPU-intensive. Keep a second executor thread free so the
    # TF listener can continue receiving LIO transforms while the scan search
    # is running.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
