#!/usr/bin/env python3
"""Deterministic hardware-interface mock for bringup and safety smoke tests.

The mock speaks the same ROS interfaces as the real adapters: Livox raw
PointCloud2/Imu, gimbal JointState, chassis Odometry/LaserScan, and the
Float64MultiArray wheel command sink.  It can deliberately stop one health
stream or introduce a front obstacle to exercise the fail-safe path.
"""

import math
import struct
import time

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, Float64MultiArray
from tf2_ros import TransformBroadcaster


class MockRobotIO(Node):
    def __init__(self):
        super().__init__("mock_robot_io")

        self.declare_parameter("lidar_topic", "/livox/lidar")
        self.declare_parameter("imu_topic", "/livox/imu")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_frame_id", "base_footprint")
        self.declare_parameter(
            "wheel_command_topic", "/wheel_velocity_controller/commands"
        )
        self.declare_parameter("command_topic", "/cmd_vel_collision")
        self.declare_parameter("command_frame_id", "gimbal_yaw_fake")
        self.declare_parameter("publish_test_command", False)
        self.declare_parameter("test_command_rate", 10.0)
        self.declare_parameter("test_command_linear_x", 0.30)
        self.declare_parameter("test_command_linear_y", 0.0)
        self.declare_parameter("test_command_angular_z", 0.0)
        self.declare_parameter("stop_test_command_after", -1.0)
        self.declare_parameter("sensor_rate", 20.0)
        self.declare_parameter("obstacle_range", -1.0)
        self.declare_parameter("obstacle_after", -1.0)
        self.declare_parameter("drop_odom_after", -1.0)
        self.declare_parameter("drop_scan_after", -1.0)
        self.declare_parameter("drop_joint_states_after", -1.0)
        self.declare_parameter("drop_tf_after", -1.0)
        self.declare_parameter("test_duration", -1.0)
        self.declare_parameter("failure_grace", 1.0)
        self.declare_parameter("assert_safety", False)

        self.lidar_topic = str(self.get_parameter("lidar_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.scan_frame_id = str(self.get_parameter("scan_frame_id").value)
        self.wheel_command_topic = str(
            self.get_parameter("wheel_command_topic").value
        )
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.command_frame_id = str(self.get_parameter("command_frame_id").value)
        self.publish_test_command = bool(
            self.get_parameter("publish_test_command").value
        )
        self.test_command_rate = self._positive_parameter("test_command_rate", 10.0)
        self.stop_test_command_after = float(
            self.get_parameter("stop_test_command_after").value
        )
        self.sensor_rate = self._positive_parameter("sensor_rate", 20.0)
        self.obstacle_range = float(self.get_parameter("obstacle_range").value)
        self.obstacle_after = float(self.get_parameter("obstacle_after").value)
        self.drop_times = {
            "odom": float(self.get_parameter("drop_odom_after").value),
            "scan": float(self.get_parameter("drop_scan_after").value),
            "joint_states": float(
                self.get_parameter("drop_joint_states_after").value
            ),
            "tf": float(self.get_parameter("drop_tf_after").value),
        }
        self.test_duration = float(self.get_parameter("test_duration").value)
        self.failure_grace = max(
            0.0, float(self.get_parameter("failure_grace").value)
        )
        self.assert_safety = bool(self.get_parameter("assert_safety").value)

        self.started_at = time.monotonic()
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.wheel_commands = []
        self.final_commands = []
        self.status_messages = []
        self.contract_errors = []

        self.lidar_publisher = self.create_publisher(
            PointCloud2, self.lidar_topic, qos_profile_sensor_data
        )
        self.imu_publisher = self.create_publisher(
            Imu, self.imu_topic, qos_profile_sensor_data
        )
        self.joint_publisher = self.create_publisher(
            # robot_state_publisher uses a reliable subscription for
            # JointState; keep the mock compatible with that hardware edge.
            JointState, self.joint_states_topic, 10
        )
        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.scan_publisher = self.create_publisher(
            LaserScan, self.scan_topic, qos_profile_sensor_data
        )
        self.command_publisher = self.create_publisher(
            TwistStamped, self.command_topic, 10
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Float64MultiArray,
            self.wheel_command_topic,
            self._wheel_command_callback,
            10,
        )
        self.create_subscription(TwistStamped, "/cmd_vel", self._final_command_callback, 10)
        self.create_subscription(
            Bool,
            "/cmd_vel_watchdog/healthy",
            lambda message: self.status_messages.append((time.monotonic(), message.data)),
            10,
        )
        # Subscribe to the mock's own outputs so the validation is a real DDS
        # type/frame/flow probe, not just a publisher-side object check.
        self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self._lidar_contract_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            self.imu_topic,
            self._imu_contract_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            self.joint_states_topic,
            self._joint_contract_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_contract_callback,
            10,
        )
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_contract_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            self.command_topic,
            self._command_contract_callback,
            10,
        )

        self.sensor_timer = self.create_timer(1.0 / self.sensor_rate, self.publish_sensors)
        if self.publish_test_command:
            self.command_timer = self.create_timer(
                1.0 / self.test_command_rate, self.publish_test_command_message
            )
        if self.test_duration > 0.0:
            self.test_timer = self.create_timer(
                0.05, self.finish_test, clock=self.steady_clock
            )

        self.get_logger().info(
            f"Mock interfaces active: raw radar {self.lidar_topic}/{self.imu_topic}, "
            f"gimbal {self.joint_states_topic}, chassis health {self.odom_topic}/"
            f"{self.scan_topic}, wheel sink {self.wheel_command_topic}"
        )

    def _positive_parameter(self, name, fallback):
        value = float(self.get_parameter(name).value)
        return value if value > 0.0 else fallback

    def elapsed(self):
        return time.monotonic() - self.started_at

    def active(self, stream):
        drop_time = self.drop_times[stream]
        return drop_time < 0.0 or self.elapsed() < drop_time

    def stamp(self):
        return self.get_clock().now().to_msg()

    def publish_sensors(self):
        stamp = self.stamp()
        elapsed = self.elapsed()
        if self.active("joint_states"):
            joint = JointState()
            joint.header.stamp = stamp
            joint.name = ["gimbal_joint"]
            joint.position = [0.0]
            joint.velocity = [0.0]
            joint.effort = [0.0]
            self.joint_publisher.publish(joint)

        if self.active("odom"):
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"
            odom.pose.pose.orientation.w = 1.0
            self.odom_publisher.publish(odom)

        if self.active("tf"):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "odom"
            transform.child_frame_id = "base_footprint"
            transform.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(transform)

        if self.active("scan"):
            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = self.scan_frame_id
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.angle_increment = 2.0 * math.pi / 360.0
            scan.range_min = 0.05
            scan.range_max = 20.0
            ranges = [float("inf")] * 360
            obstacle = self.obstacle_range
            if self.obstacle_after >= 0.0 and elapsed >= self.obstacle_after:
                obstacle = self.obstacle_range if self.obstacle_range > 0.0 else 0.15
            if obstacle > 0.0:
                # angle_min=-pi, so angle zero (straight ahead) is around
                # index 180, not index zero.
                for index in range(177, 184):
                    ranges[index] = obstacle
            scan.ranges = ranges
            self.scan_publisher.publish(scan)

        self.lidar_publisher.publish(self._make_lidar_message(stamp))
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "livox_frame"
        # The real launch converts Livox g units to SI in the compensator.
        imu.linear_acceleration.z = -1.0
        imu.orientation.w = 1.0
        self.imu_publisher.publish(imu)

    def _make_lidar_message(self, stamp):
        message = PointCloud2()
        message.header.stamp = stamp
        message.header.frame_id = "livox_frame"
        message.height = 1
        message.width = 1
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="tag", offset=12, datatype=PointField.UINT8, count=1),
            PointField(name="timestamp", offset=16, datatype=PointField.FLOAT64, count=1),
        ]
        message.point_step = 24
        message.row_step = message.point_step
        message.is_bigendian = False
        message.is_dense = True
        payload = bytearray(message.point_step)
        timestamp_ns = float(stamp.sec) * 1.0e9 + float(stamp.nanosec)
        struct.pack_into("<fffB3xd", payload, 0, 2.0, 0.0, 0.2, 0, timestamp_ns)
        message.data = payload
        return message

    def publish_test_command_message(self):
        if (
            self.stop_test_command_after >= 0.0
            and self.elapsed() >= self.stop_test_command_after
        ):
            return
        message = TwistStamped()
        message.header.stamp = self.stamp()
        message.header.frame_id = self.command_frame_id
        message.twist.linear.x = float(self.get_parameter("test_command_linear_x").value)
        message.twist.linear.y = float(self.get_parameter("test_command_linear_y").value)
        message.twist.angular.z = float(self.get_parameter("test_command_angular_z").value)
        self.command_publisher.publish(message)

    def _contract_error(self, message):
        if message not in self.contract_errors:
            self.contract_errors.append(message)

    def _lidar_contract_callback(self, message):
        field_types = {
            field.name: field.datatype for field in message.fields
        }
        expected = {
            "x": PointField.FLOAT32,
            "y": PointField.FLOAT32,
            "z": PointField.FLOAT32,
            "tag": PointField.UINT8,
            "timestamp": PointField.FLOAT64,
        }
        if message.header.frame_id != "livox_frame":
            self._contract_error("PointCloud2 frame_id must be livox_frame")
        if field_types != expected or message.point_step != 24:
            self._contract_error(
                "PointCloud2 must expose x/y/z float32, tag uint8, timestamp float64"
            )

    def _imu_contract_callback(self, message):
        if message.header.frame_id != "livox_frame":
            self._contract_error("Imu frame_id must be livox_frame")

    def _joint_contract_callback(self, message):
        if "gimbal_joint" not in message.name:
            self._contract_error("JointState must contain gimbal_joint")

    def _odom_contract_callback(self, message):
        if (
            message.header.frame_id != "odom"
            or message.child_frame_id != "base_footprint"
        ):
            self._contract_error(
                "Odometry must use header odom and child_frame_id base_footprint"
            )

    def _scan_contract_callback(self, message):
        if message.header.frame_id != self.scan_frame_id:
            self._contract_error(
                f"LaserScan frame_id must be {self.scan_frame_id}"
            )

    def _command_contract_callback(self, message):
        if message.header.frame_id != self.command_frame_id:
            self._contract_error(
                f"input TwistStamped frame_id must be {self.command_frame_id}"
            )

    def _final_command_callback(self, message):
        if message.header.frame_id != "base_link":
            self._contract_error("final TwistStamped frame_id must be base_link")
        self.final_commands.append(
            (time.monotonic(), abs(message.twist.linear.x) + abs(message.twist.linear.y) + abs(message.twist.angular.z))
        )

    def _wheel_command_callback(self, message):
        if len(message.data) != 4 or not all(math.isfinite(value) for value in message.data):
            self.get_logger().error(
                "Wheel command contract violation: expected four finite rad/s values"
            )
            return
        self.wheel_commands.append((time.monotonic(), max(abs(value) for value in message.data)))

    def finish_test(self):
        if self.elapsed() < self.test_duration:
            return
        if not self.assert_safety:
            self.test_timer.cancel()
            return

        failure_times = [value for value in self.drop_times.values() if value >= 0.0]
        if self.obstacle_after >= 0.0:
            failure_times.append(self.obstacle_after)
        if self.stop_test_command_after >= 0.0:
            failure_times.append(self.stop_test_command_after)
        failure_time = min(failure_times) if failure_times else self.test_duration
        healthy_commands = [value for stamp, value in self.wheel_commands if stamp - self.started_at < failure_time]
        after_failure = [
            value
            for stamp, value in self.wheel_commands
            if stamp - self.started_at >= failure_time + self.failure_grace
        ]
        self.get_logger().info(
            "Validation metrics: wheel_samples=%d, healthy_samples=%d, "
            "after_fault_samples=%d, after_fault_max=%.6g, "
            "failure_time=%.2fs, failure_grace=%.2fs"
            % (
                len(self.wheel_commands),
                len(healthy_commands),
                len(after_failure),
                max(after_failure) if after_failure else float("nan"),
                failure_time,
                self.failure_grace,
            )
        )
        if self.contract_errors:
            self.get_logger().error(
                "Interface contract violations: " + "; ".join(self.contract_errors)
            )
            self.test_timer.cancel()
            rclpy.shutdown()
            return
        if healthy_commands and after_failure and max(after_failure) < 1.0e-6:
            self.get_logger().info(
                "PASS: mock hardware observed motion before the fault and zero wheel commands after it"
            )
            self.test_timer.cancel()
            rclpy.shutdown()
            return

        self.get_logger().error(
            "FAIL: wheel commands were not proven zero after the injected fault"
        )
        self.test_timer.cancel()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MockRobotIO()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
