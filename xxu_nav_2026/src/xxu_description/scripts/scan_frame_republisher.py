#!/usr/bin/env python3
"""Republish LaserScan with corrected frame_id."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanFrameRepublisher(Node):
    def __init__(self):
        super().__init__("scan_frame_republisher")
        self.declare_parameter("target_frame", "radar_link")
        self.target_frame = self.get_parameter("target_frame").value

        self.sub = self.create_subscription(LaserScan, "scan_in", self.callback, 10)
        self.pub = self.create_publisher(LaserScan, "scan_out", 10)
        self.get_logger().info(f"Republishing scan_in -> scan_out with frame_id={self.target_frame}")

    def callback(self, msg: LaserScan):
        msg.header.frame_id = self.target_frame
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanFrameRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
