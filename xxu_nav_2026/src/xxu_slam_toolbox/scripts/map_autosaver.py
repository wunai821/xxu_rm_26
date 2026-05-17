#!/usr/bin/env python3
"""Cache /map and write it to disk when the mapping launch exits."""

import math
import signal
import sys
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MapAutosaver(Node):
    def __init__(self):
        super().__init__("map_autosaver")
        self.declare_parameter("map_path", "auto_map")
        self.declare_parameter("occupied_thresh", 0.65)
        self.declare_parameter("free_thresh", 0.196)

        self.map_path = Path(self.get_parameter("map_path").value).expanduser()
        self.occupied_thresh = float(self.get_parameter("occupied_thresh").value)
        self.free_thresh = float(self.get_parameter("free_thresh").value)
        self.latest_map = None
        self.saved = False

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, qos)
        self.get_logger().info(f"Auto-saving /map to {self.map_path} on shutdown")

    def map_callback(self, msg):
        self.latest_map = msg

    def save(self):
        if self.saved:
            return True
        if self.latest_map is None:
            self.get_logger().warn("No /map message received; skipping automatic map save")
            return False

        map_path = self.map_path
        yaml_path = map_path.with_suffix(".yaml")
        pgm_path = map_path.with_suffix(".pgm")
        yaml_path.parent.mkdir(parents=True, exist_ok=True)

        grid = self.latest_map
        width = grid.info.width
        height = grid.info.height
        if width == 0 or height == 0 or len(grid.data) != width * height:
            self.get_logger().error("Invalid occupancy grid; skipping automatic map save")
            return False

        with pgm_path.open("wb") as pgm:
            pgm.write(f"P5\n# CREATOR: xxu_slam_toolbox map_autosaver\n{width} {height}\n255\n".encode())
            for y in range(height - 1, -1, -1):
                row_offset = y * width
                for x in range(width):
                    value = grid.data[row_offset + x]
                    if value < 0:
                        pixel = 205
                    elif value >= int(self.occupied_thresh * 100.0):
                        pixel = 0
                    elif value <= int(self.free_thresh * 100.0):
                        pixel = 254
                    else:
                        pixel = 205
                    pgm.write(bytes([pixel]))

        origin = grid.info.origin
        yaw = yaw_from_quaternion(origin.orientation)
        yaml_path.write_text(
            "\n".join([
                f"image: {pgm_path.name}",
                "mode: trinary",
                f"resolution: {grid.info.resolution:.3f}",
                (
                    "origin: "
                    f"[{origin.position.x:.3f}, {origin.position.y:.3f}, {yaw:.6f}]"
                ),
                "negate: 0",
                f"occupied_thresh: {self.occupied_thresh}",
                f"free_thresh: {self.free_thresh}",
                "",
            ]),
            encoding="utf-8",
        )

        self.saved = True
        self.get_logger().info(f"Saved map to {yaml_path} and {pgm_path}")
        return True


def main():
    rclpy.init()
    node = MapAutosaver()
    stopping = False

    def handle_shutdown(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.save()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.save()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
