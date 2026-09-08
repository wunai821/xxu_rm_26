#!/usr/bin/env python3
"""Control the simulated radar gimbal in continuous-spin or world-yaw-hold mode."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return ROS ZYX yaw from a normalized quaternion."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GimbalStabilizer(Node):
    """Drive the radar gimbal while keeping the joint TF physically truthful."""

    def __init__(self) -> None:
        super().__init__('gimbal_stabilizer')

        self.declare_parameter('imu_topic', '/imu_raw')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter(
            'command_topic', '/gimbal_velocity_controller/commands'
        )
        self.declare_parameter('joint_name', 'gimbal_joint')
        self.declare_parameter('mode', 'spin')
        self.declare_parameter('spin_rate', 6.283185307179586)
        self.declare_parameter('spin_start_delay', 1.0)
        self.declare_parameter('wait_for_odom', True)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('command_rate', 50.0)
        self.declare_parameter('hold_gain', 4.0)
        self.declare_parameter('max_hold_velocity', 2.0)

        self._joint_name = str(self.get_parameter('joint_name').value)
        self._mode = str(self.get_parameter('mode').value).lower()
        self._spin_rate = float(self.get_parameter('spin_rate').value)
        self._spin_start_delay = float(
            self.get_parameter('spin_start_delay').value
        )
        self._wait_for_odom = bool(self.get_parameter('wait_for_odom').value)
        command_rate = float(self.get_parameter('command_rate').value)
        self._hold_gain = float(self.get_parameter('hold_gain').value)
        self._max_hold_velocity = abs(float(
            self.get_parameter('max_hold_velocity').value
        ))
        if self._mode not in ('spin', 'hold'):
            raise ValueError("mode must be either 'spin' or 'hold'")
        if command_rate <= 0.0:
            raise ValueError('command_rate must be greater than zero')

        self._joint_position = None
        self._hold_yaw = None
        self._spin_start_time = None
        self._odom_ready_time = None

        self._command_publisher = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter('command_topic').value),
            10,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_states_topic').value),
            self._joint_state_callback,
            20,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter('imu_topic').value),
            self._imu_callback,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            10,
        )
        self.create_timer(1.0 / command_rate, self._command_timer_callback)

        if self._mode == 'spin':
            period = math.inf
            if abs(self._spin_rate) > 1.0e-9:
                period = 2.0 * math.pi / abs(self._spin_rate)
            self.get_logger().info(
                '雷达云台连续旋转: '
                f'{self._spin_rate:.3f} rad/s '
                f'({math.degrees(self._spin_rate):.1f} deg/s), '
                f'一圈约 {period:.1f} s；'
                + ('等待LIO输出/odom后' if self._wait_for_odom else '启动后')
                + f'静止 {self._spin_start_delay:.1f} s 再旋转'
            )
        else:
            self.get_logger().info('雷达云台使用世界航向保持模式')

    def _joint_state_callback(self, message: JointState) -> None:
        try:
            index = message.name.index(self._joint_name)
        except ValueError:
            return
        if index < len(message.position) and math.isfinite(message.position[index]):
            self._joint_position = message.position[index]

    def _imu_callback(self, message: Imu) -> None:
        if self._mode != 'hold':
            return
        if self._joint_position is None:
            return

        orientation = message.orientation
        norm = math.sqrt(
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )
        if norm < 1.0e-6:
            return

        world_yaw = quaternion_yaw(
            orientation.x / norm,
            orientation.y / norm,
            orientation.z / norm,
            orientation.w / norm,
        )
        if self._hold_yaw is None:
            self._hold_yaw = world_yaw
            self.get_logger().info(
                f'保持雷达初始世界航向 {math.degrees(world_yaw):.1f} deg'
            )

        correction = wrap_angle(self._hold_yaw - world_yaw)
        velocity = self._hold_gain * correction
        velocity = max(-self._max_hold_velocity, min(self._max_hold_velocity, velocity))
        command = Float64MultiArray()
        command.data = [velocity]
        self._command_publisher.publish(command)

    def _odom_callback(self, _message: Odometry) -> None:
        if self._odom_ready_time is None:
            self._odom_ready_time = self.get_clock().now()
            self.get_logger().info('LIO已输出/odom，云台初始化等待开始')

    def _command_timer_callback(self) -> None:
        if self._mode != 'spin' or self._joint_position is None:
            return

        now = self.get_clock().now()
        if self._spin_start_time is None:
            self._spin_start_time = now

        if self._wait_for_odom and self._odom_ready_time is None:
            self._publish_velocity(0.0)
            return

        delay_start_time = (
            self._odom_ready_time if self._wait_for_odom else self._spin_start_time
        )
        startup_elapsed = (now - delay_start_time).nanoseconds * 1.0e-9
        if startup_elapsed < self._spin_start_delay:
            self._publish_velocity(0.0)
            return

        self._publish_velocity(self._spin_rate)

    def _publish_velocity(self, velocity: float) -> None:
        command = Float64MultiArray()
        command.data = [velocity]
        self._command_publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GimbalStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
