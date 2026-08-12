#!/usr/bin/env python3
"""Start the Nav2 navigation lifecycle only after localization is usable."""

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class NavigationStartup(Node):
    """Gate Nav2 navigation activation on the AMCL map-to-odom transform."""

    def __init__(self):
        super().__init__("nav2_navigation_startup")
        self.declare_parameter("manager_service", "/lifecycle_manager_navigation/manage_nodes")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("check_period", 0.5)

        self.manager_service = self.get_parameter("manager_service").value
        self.global_frame = self.get_parameter("global_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = self.create_client(ManageLifecycleNodes, self.manager_service)
        self.request_sent = False
        self.timer = self.create_timer(
            float(self.get_parameter("check_period").value), self.try_start
        )

    def try_start(self):
        if self.request_sent:
            return
        if not self.client.service_is_ready():
            self.get_logger().info(
                f"Waiting for navigation lifecycle manager at {self.manager_service}",
                throttle_duration_sec=5.0,
            )
            return
        if not self.tf_buffer.can_transform(
            self.global_frame, self.odom_frame, Time(), timeout=Duration(seconds=0.0)
        ):
            self.get_logger().info(
                f"Waiting for localization transform {self.global_frame} -> {self.odom_frame}",
                throttle_duration_sec=5.0,
            )
            return

        self.request_sent = True
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = self.client.call_async(request)
        future.add_done_callback(self.startup_complete)
        self.get_logger().info("Localization is ready; starting Nav2 navigation lifecycle")

    def startup_complete(self, future):
        try:
            response = future.result()
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Navigation lifecycle startup failed: {exc}")
            self.request_sent = False
            return

        if response.success:
            self.get_logger().info("Nav2 navigation lifecycle is active")
            self.timer.cancel()
        else:
            self.get_logger().error("Navigation lifecycle manager rejected startup; retrying")
            self.request_sent = False


def main():
    rclpy.init()
    node = NavigationStartup()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
