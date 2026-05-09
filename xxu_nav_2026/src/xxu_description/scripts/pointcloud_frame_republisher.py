#!/usr/bin/env python3
"""Republish PointCloud2 with corrected frame_id."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class PointCloudFrameRepublisher(Node):
    def __init__(self):
        super().__init__("pointcloud_frame_republisher")
        self.declare_parameter("target_frame", "radar_link")
        self.target_frame = self.get_parameter("target_frame").value

        self.sub = self.create_subscription(PointCloud2, "points_in", self.callback, 10)
        self.pub = self.create_publisher(PointCloud2, "points_out", 10)
        self.get_logger().info(
            f"Republishing points_in -> points_out with frame_id={self.target_frame}"
        )

    def callback(self, msg: PointCloud2):
        msg.header.frame_id = self.target_frame
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PointCloudFrameRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
