#!/usr/bin/env python3
"""Republish Imu with corrected frame_id."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuFrameRepublisher(Node):
    def __init__(self):
        super().__init__("imu_frame_republisher")
        self.declare_parameter("target_frame", "mid360_link")
        self.target_frame = self.get_parameter("target_frame").value

        self.sub = self.create_subscription(Imu, "imu_in", self.callback, 10)
        self.pub = self.create_publisher(Imu, "imu_out", 10)
        self.get_logger().info(
            f"Republishing imu_in -> imu_out with frame_id={self.target_frame}"
        )

    def callback(self, msg: Imu):
        msg.header.frame_id = self.target_frame
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuFrameRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
