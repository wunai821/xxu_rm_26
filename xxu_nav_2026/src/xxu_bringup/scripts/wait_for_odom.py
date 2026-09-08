#!/usr/bin/env python3
"""Exit after the first odometry message is received on the configured topic."""

import sys

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class OdomReadyGate(Node):
    """Provide a launch event when odometry is actually available."""

    def __init__(self):
        super().__init__('wait_for_odom')
        self.declare_parameter('topic', '/odom')
        self.topic = str(self.get_parameter('topic').value)
        self.report_timer = self.create_timer(5.0, self.report_waiting)
        self.subscription = self.create_subscription(
            Odometry,
            self.topic,
            self.odom_callback,
            10,
        )
        self.get_logger().info(
            f'Waiting for the first Odometry message on {self.topic} before starting Nav2'
        )

    def report_waiting(self):
        self.get_logger().info(
            f'Still waiting for an Odometry message on {self.topic}',
            throttle_duration_sec=15.0,
        )

    def odom_callback(self, _msg):
        self.get_logger().info(
            f'Received the first Odometry message on {self.topic}; starting Nav2'
        )
        self.report_timer.cancel()
        self.destroy_subscription(self.subscription)
        rclpy.shutdown()


def main():
    rclpy.init()
    node = OdomReadyGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
