#!/usr/bin/env python3
"""Republish cmd_vel and stop the robot when commands time out."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelWatchdog(Node):
    def __init__(self):
        super().__init__("cmd_vel_watchdog")

        self.declare_parameter("input_topic", "/cmd_vel_keyboard")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("timeout", 0.3)
        self.declare_parameter("publish_rate", 20.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.timeout = float(self.get_parameter("timeout").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.last_cmd = Twist()
        self.last_cmd_time = None

        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.subscription = self.create_subscription(
            Twist,
            input_topic,
            self.cmd_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_cmd)

    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def publish_cmd(self):
        cmd = Twist()
        if self.last_cmd_time is not None:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if age <= self.timeout:
                cmd = self.last_cmd

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
