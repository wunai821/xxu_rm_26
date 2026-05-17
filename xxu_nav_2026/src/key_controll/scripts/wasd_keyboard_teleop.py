#!/usr/bin/env python3
"""Keyboard teleop for the XXU Gazebo robot."""

import select
import sys
import termios
import tty
import os

os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node


HELP_TEXT = """
XXU keyboard teleop
------------------
W/S : forward / backward
A/D : strafe left / right
Q/E : rotate left / right; choose gyro direction when gyro mode is enabled
G : toggle gyro mode, rotate while driving
Space : stop
Ctrl-C : quit
"""


class WasdKeyboardTeleop(Node):
    def __init__(self):
        super().__init__("wasd_keyboard_teleop")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel_keyboard")
        self.declare_parameter("linear_speed", 0.5)
        self.declare_parameter("angular_speed", 1.0)
        self.declare_parameter("linear_acceleration", 5.0)
        self.declare_parameter("angular_acceleration", 10.0)
        self.declare_parameter("gyro_angular_speed", 1.5)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("key_timeout", 0.35)

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.linear_acceleration = float(self.get_parameter("linear_acceleration").value)
        self.angular_acceleration = float(self.get_parameter("angular_acceleration").value)
        self.gyro_angular_speed = float(self.get_parameter("gyro_angular_speed").value)
        self.key_timeout = float(self.get_parameter("key_timeout").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        if publish_rate <= 0.0:
            self.get_logger().warn("publish_rate must be positive; using 50.0")
            publish_rate = 50.0
        if self.linear_acceleration <= 0.0:
            self.get_logger().warn("linear_acceleration must be positive; using 5.0")
            self.linear_acceleration = 5.0
        if self.angular_acceleration <= 0.0:
            self.get_logger().warn("angular_acceleration must be positive; using 10.0")
            self.angular_acceleration = 10.0
        if self.key_timeout <= 0.0:
            self.get_logger().warn("key_timeout must be positive; using 0.35")
            self.key_timeout = 0.35

        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.active_until = {}
        self.last_publish_time = self.get_clock().now()
        self.output_cmd = Twist()
        self.gyro_enabled = False
        self.gyro_direction = 1.0
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_cmd)

        self.get_logger().info(
            f"Publishing keyboard commands to {cmd_vel_topic}: "
            f"linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}, "
            f"linear_accel={self.linear_acceleration:.2f}, "
            f"angular_accel={self.angular_acceleration:.2f}, "
            f"gyro={self.gyro_angular_speed:.2f}"
        )

    def handle_key(self, key):
        key = key.lower()
        now = self.get_clock().now()

        if key in ("w", "s", "a", "d", "q", "e"):
            self.active_until[key] = now + Duration(seconds=self.key_timeout)
            if key == "q" and self.gyro_enabled:
                self.gyro_direction = 1.0
            elif key == "e" and self.gyro_enabled:
                self.gyro_direction = -1.0
        elif key == "g":
            self.gyro_enabled = not self.gyro_enabled
            self.get_logger().info(
                f"Gyro mode {'enabled' if self.gyro_enabled else 'disabled'}"
            )
        elif key == " ":
            self.gyro_enabled = False
            self.active_until.clear()
            self.output_cmd = Twist()
            self.publisher.publish(self.output_cmd)
        else:
            return

    def approach(self, current, target, max_delta):
        if current < target:
            return min(current + max_delta, target)
        if current > target:
            return max(current - max_delta, target)
        return current

    def gyro_cmd(self):
        return self.gyro_direction * abs(self.gyro_angular_speed)

    def key_active(self, key, now):
        return key in self.active_until and self.active_until[key] > now

    def prune_inactive_keys(self, now):
        expired = [key for key, end_time in self.active_until.items() if end_time <= now]
        for key in expired:
            del self.active_until[key]

    def build_target_cmd(self, now):
        target = Twist()
        x_axis = float(self.key_active("w", now)) - float(self.key_active("s", now))
        y_axis = float(self.key_active("a", now)) - float(self.key_active("d", now))

        target.linear.x = x_axis * self.linear_speed
        target.linear.y = y_axis * self.linear_speed

        if self.gyro_enabled:
            target.angular.z = self.gyro_cmd()
        else:
            angular_axis = float(self.key_active("q", now)) - float(self.key_active("e", now))
            target.angular.z = angular_axis * self.angular_speed

        return target

    def publish_cmd(self):
        now = self.get_clock().now()
        dt = max((now - self.last_publish_time).nanoseconds * 1e-9, 0.0)
        self.last_publish_time = now
        self.prune_inactive_keys(now)
        target_cmd = self.build_target_cmd(now)

        linear_step = self.linear_acceleration * dt
        angular_step = self.angular_acceleration * dt
        self.output_cmd.linear.x = self.approach(
            self.output_cmd.linear.x, target_cmd.linear.x, linear_step
        )
        self.output_cmd.linear.y = self.approach(
            self.output_cmd.linear.y, target_cmd.linear.y, linear_step
        )
        self.output_cmd.angular.z = self.approach(
            self.output_cmd.angular.z, target_cmd.angular.z, angular_step
        )

        self.publisher.publish(self.output_cmd)

    def stop(self):
        self.gyro_enabled = False
        self.active_until.clear()
        self.output_cmd = Twist()
        self.publisher.publish(self.output_cmd)


def read_key(timeout):
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    if not readable:
        return None
    return sys.stdin.read(1)


def main(args=None):
    rclpy.init(args=args)
    old_settings = termios.tcgetattr(sys.stdin)
    node = None

    print(HELP_TEXT)
    try:
        node = WasdKeyboardTeleop()
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key(0.005)
            if key is not None:
                node.handle_key(key)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        if "This member is not been selected" in str(exc):
            print(
                "\nFailed to create a ROS node. The current FastDDS/FastCDR "
                "runtime is failing before this teleop node starts.\n"
                "Install CycloneDDS and run with:\n"
                "  sudo apt-get install ros-jazzy-rmw-cyclonedds-cpp\n"
                "  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n",
                file=sys.stderr,
            )
        raise
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        if node is not None:
            node.stop()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
