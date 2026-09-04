#!/usr/bin/env python3
"""Convert a gimbal-mounted IMU into a virtual IMU at the gimbal axis."""

import bisect
from collections import deque
import copy
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState


def _rotation_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _rotation_from_axis_angle(axis, angle):
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _rotate_covariance(values, rotation, scale=1.0):
    if len(values) != 9 or values[0] < 0.0:
        return list(values)
    covariance = np.asarray(values, dtype=float).reshape(3, 3)
    return (
        scale * scale * (rotation @ covariance @ rotation.T)
    ).reshape(-1).tolist()


class GimbalImuCompensator(Node):
    """Remove known joint motion at a virtual frame on the gimbal axis."""

    def __init__(self):
        super().__init__('gimbal_imu_compensator')
        self.declare_parameter('joint_name', 'gimbal_joint')
        self.declare_parameter('target_frame', 'lio_base_sensor')
        self.declare_parameter('joint_axis', [0.0, 0.0, 1.0])
        self.declare_parameter('sensor_offset', [0.0, 0.08637, 0.0])
        self.declare_parameter('sensor_rpy', [-0.2967059728, 0.0, 0.0])
        self.declare_parameter('joint_position_offset', 0.0)
        self.declare_parameter('input_acceleration_scale', 1.0)
        self.declare_parameter('angular_acceleration_time_constant', 0.05)
        self.declare_parameter('joint_acceleration_time_constant', 0.05)
        self.declare_parameter('max_joint_acceleration', 100.0)
        self.declare_parameter('max_joint_sample_gap', 0.05)
        self.declare_parameter('time_reset_threshold', 0.5)

        self.joint_name = str(self.get_parameter('joint_name').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.joint_axis = np.asarray(
            self.get_parameter('joint_axis').value, dtype=float
        )
        if self.joint_axis.shape != (3,) or not np.all(np.isfinite(self.joint_axis)):
            raise ValueError('joint_axis must contain three finite values')
        axis_norm = np.linalg.norm(self.joint_axis)
        if axis_norm <= np.finfo(float).eps:
            raise ValueError('joint_axis must be non-zero')
        self.joint_axis /= axis_norm
        self.sensor_offset = np.asarray(
            self.get_parameter('sensor_offset').value, dtype=float
        )
        if self.sensor_offset.shape != (3,) or not np.all(
            np.isfinite(self.sensor_offset)
        ):
            raise ValueError('sensor_offset must contain three finite values')
        sensor_rpy = np.asarray(
            self.get_parameter('sensor_rpy').value, dtype=float
        )
        if sensor_rpy.shape != (3,) or not np.all(np.isfinite(sensor_rpy)):
            raise ValueError('sensor_rpy must contain three finite values')
        self.sensor_rotation = _rotation_from_rpy(*sensor_rpy)
        self.joint_position_offset = float(
            self.get_parameter('joint_position_offset').value
        )
        if not math.isfinite(self.joint_position_offset):
            raise ValueError('joint_position_offset must be finite')
        self.input_acceleration_scale = float(
            self.get_parameter('input_acceleration_scale').value
        )
        if (
            not math.isfinite(self.input_acceleration_scale)
            or self.input_acceleration_scale <= 0.0
        ):
            raise ValueError('input_acceleration_scale must be finite and positive')
        self.derivative_tau = self._nonnegative_seconds_parameter(
            'angular_acceleration_time_constant'
        )
        self.joint_acceleration_tau = self._nonnegative_seconds_parameter(
            'joint_acceleration_time_constant'
        )
        self.max_joint_acceleration = float(
            self.get_parameter('max_joint_acceleration').value
        )
        if (
            not math.isfinite(self.max_joint_acceleration)
            or self.max_joint_acceleration <= 0.0
        ):
            raise ValueError('max_joint_acceleration must be finite and positive')
        self.max_joint_sample_gap = self._positive_seconds_parameter(
            'max_joint_sample_gap'
        )
        self.time_reset_threshold = self._positive_seconds_parameter(
            'time_reset_threshold'
        )

        self.joint_samples = deque(maxlen=2048)
        self.pending_imus = deque()
        self.pending_imu_limit = 256
        self.last_raw_position = None
        self.last_unwrapped_position = None
        self.previous_base_omega = None
        self.previous_imu_time = None
        self.filtered_base_alpha = np.zeros(3)
        self.previous_joint_velocity = None
        self.previous_joint_time = None
        self.filtered_joint_acceleration = 0.0
        self.missing_joint_velocity_warned = False
        self.nonfinite_imu_warned = False
        self.dropped_nonfinite_imus = 0

        self.publisher = self.create_publisher(Imu, 'imu_out', qos_profile_sensor_data)
        self.create_subscription(
            JointState, 'joint_states', self._joint_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, 'imu_in', self._imu_callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            f'Compensating gimbal IMU into virtual frame {self.target_frame} '
            f'(acceleration scale={self.input_acceleration_scale:g})'
        )

    def _joint_callback(self, message):
        try:
            index = message.name.index(self.joint_name)
        except ValueError:
            return
        if index >= len(message.position):
            return
        if index >= len(message.velocity):
            if not self.missing_joint_velocity_warned:
                self.get_logger().error(
                    f'JointState {self.joint_name} has no velocity; '
                    'IMU compensation requires measured joint velocity'
                )
                self.missing_joint_velocity_warned = True
            return
        position = float(message.position[index])
        velocity = float(message.velocity[index])
        timestamp = _stamp_seconds(message.header.stamp)
        if not all(math.isfinite(value) for value in (timestamp, position, velocity)):
            return

        if self.joint_samples and timestamp <= self.joint_samples[-1][0]:
            backwards = self.joint_samples[-1][0] - timestamp
            if backwards < self.time_reset_threshold:
                return
            self._reset_temporal_state(clear_joint_samples=True)
            self.get_logger().warning(
                'Joint timestamp moved backwards; reset IMU compensation state'
            )
        elif (
            self.joint_samples
            and timestamp - self.joint_samples[-1][0]
            > self.max_joint_sample_gap
        ):
            gap = timestamp - self.joint_samples[-1][0]
            self._reset_temporal_state(clear_joint_samples=True)
            self.get_logger().warning(
                f'Joint samples had a {gap:.3f}s gap; reset IMU compensation state'
            )

        if self.last_raw_position is None:
            unwrapped = position
        else:
            delta = math.atan2(
                math.sin(position - self.last_raw_position),
                math.cos(position - self.last_raw_position),
            )
            unwrapped = self.last_unwrapped_position + delta
        self.last_raw_position = position
        self.last_unwrapped_position = unwrapped

        joint_acceleration = 0.0
        if (
            self.previous_joint_velocity is not None
            and self.previous_joint_time is not None
        ):
            dt = timestamp - self.previous_joint_time
            raw_acceleration = (velocity - self.previous_joint_velocity) / dt
            raw_acceleration = max(
                -self.max_joint_acceleration,
                min(self.max_joint_acceleration, raw_acceleration),
            )
            gain = 1.0 if self.joint_acceleration_tau == 0.0 else dt / (
                self.joint_acceleration_tau + dt
            )
            self.filtered_joint_acceleration += gain * (
                raw_acceleration - self.filtered_joint_acceleration
            )
            joint_acceleration = self.filtered_joint_acceleration
        self.previous_joint_velocity = velocity
        self.previous_joint_time = timestamp

        self.joint_samples.append(
            (timestamp, unwrapped, velocity, joint_acceleration)
        )
        self._process_pending()

    def _imu_callback(self, message):
        timestamp = _stamp_seconds(message.header.stamp)
        measurement = (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        )
        if not math.isfinite(timestamp) or not all(map(math.isfinite, measurement)):
            self.dropped_nonfinite_imus += 1
            if not self.nonfinite_imu_warned:
                self.get_logger().error(
                    'Dropping IMU with non-finite timestamp, gyro, or acceleration'
                )
                self.nonfinite_imu_warned = True
            return
        latest_timestamp = self.previous_imu_time
        if self.pending_imus:
            latest_timestamp = _stamp_seconds(self.pending_imus[-1].header.stamp)
        if latest_timestamp is not None and timestamp <= latest_timestamp:
            backwards = latest_timestamp - timestamp
            if backwards < self.time_reset_threshold:
                return
            self._reset_temporal_state(clear_joint_samples=False)
            self.get_logger().warning(
                'IMU timestamp moved backwards; reset IMU derivative state'
            )
        if len(self.pending_imus) >= self.pending_imu_limit:
            self.pending_imus.popleft()
        self.pending_imus.append(copy.deepcopy(message))
        self._process_pending()

    def _interpolate_joint(self, timestamp):
        if len(self.joint_samples) < 2:
            return None
        times = [sample[0] for sample in self.joint_samples]
        epsilon = 1.0e-9
        if timestamp < times[0] - epsilon or timestamp > times[-1] + epsilon:
            return None
        upper = bisect.bisect_left(times, timestamp)
        if upper < len(times) and abs(times[upper] - timestamp) <= epsilon:
            sample = self.joint_samples[upper]
            if upper == 0:
                neighbor = self.joint_samples[1]
                interval = neighbor[0] - sample[0]
                if interval <= 0.0 or interval > self.max_joint_sample_gap:
                    return None
            else:
                neighbor = self.joint_samples[upper - 1]
                interval = sample[0] - neighbor[0]
                if interval <= 0.0 or interval > self.max_joint_sample_gap:
                    return None
            return sample[1], sample[2], sample[3]
        if upper == 0 or upper >= len(times):
            return None
        before = self.joint_samples[upper - 1]
        after = self.joint_samples[upper]
        interval = after[0] - before[0]
        if interval <= 0.0 or interval > self.max_joint_sample_gap:
            return None
        ratio = (timestamp - before[0]) / interval
        position = before[1] + ratio * (after[1] - before[1])
        velocity = before[2] + ratio * (after[2] - before[2])
        acceleration = before[3] + ratio * (after[3] - before[3])
        return position, velocity, acceleration

    def _process_pending(self):
        while self.pending_imus:
            message = self.pending_imus[0]
            timestamp = _stamp_seconds(message.header.stamp)
            joint = self._interpolate_joint(timestamp)
            if joint is None:
                if self.joint_samples and timestamp < self.joint_samples[0][0]:
                    self.pending_imus.popleft()
                    continue
                if self.joint_samples and timestamp <= self.joint_samples[-1][0]:
                    # Timestamp is covered, but only across a missing/late joint
                    # interval. Never extrapolate or block newer valid IMU data.
                    self.pending_imus.popleft()
                    continue
                break
            self.pending_imus.popleft()
            self._publish_compensated(message, timestamp, *joint)

    def _publish_compensated(
        self, message, timestamp, joint_position, joint_velocity,
        joint_acceleration
    ):
        joint_rotation = _rotation_from_axis_angle(
            self.joint_axis, joint_position + self.joint_position_offset
        )
        base_from_sensor = joint_rotation @ self.sensor_rotation
        lever_arm = joint_rotation @ self.sensor_offset

        sensor_omega = np.array([
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        ])
        total_omega = base_from_sensor @ sensor_omega
        relative_omega = self.joint_axis * joint_velocity
        base_omega = total_omega - relative_omega

        base_alpha = np.zeros(3)
        if (
            self.previous_base_omega is not None
            and self.previous_imu_time is not None
            and timestamp > self.previous_imu_time
        ):
            dt = timestamp - self.previous_imu_time
            raw_alpha = (base_omega - self.previous_base_omega) / dt
            gain = 1.0 if self.derivative_tau == 0.0 else dt / (
                self.derivative_tau + dt
            )
            self.filtered_base_alpha += gain * (
                raw_alpha - self.filtered_base_alpha
            )
            base_alpha = self.filtered_base_alpha
        self.previous_base_omega = base_omega
        self.previous_imu_time = timestamp

        relative_alpha = self.joint_axis * joint_acceleration
        relative_velocity = np.cross(relative_omega, lever_arm)
        sensor_origin_acceleration = (
            np.cross(base_alpha, lever_arm)
            + np.cross(base_omega, np.cross(base_omega, lever_arm))
            + 2.0 * np.cross(base_omega, relative_velocity)
            + np.cross(relative_alpha, lever_arm)
            + np.cross(relative_omega, np.cross(relative_omega, lever_arm))
        )
        sensor_specific_force = np.array([
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        ]) * self.input_acceleration_scale
        base_specific_force = (
            base_from_sensor @ sensor_specific_force
            - sensor_origin_acceleration
        )

        output = copy.deepcopy(message)
        output.header.frame_id = self.target_frame
        output.angular_velocity.x = float(base_omega[0])
        output.angular_velocity.y = float(base_omega[1])
        output.angular_velocity.z = float(base_omega[2])
        output.linear_acceleration.x = float(base_specific_force[0])
        output.linear_acceleration.y = float(base_specific_force[1])
        output.linear_acceleration.z = float(base_specific_force[2])

        # Small Point-LIO uses only angular velocity and specific force. Mark
        # orientation unavailable instead of relabelling sensor orientation as
        # the virtual frame.
        output.orientation.x = 0.0
        output.orientation.y = 0.0
        output.orientation.z = 0.0
        output.orientation.w = 1.0
        output.orientation_covariance = [-1.0] + [0.0] * 8
        output.angular_velocity_covariance = _rotate_covariance(
            message.angular_velocity_covariance, base_from_sensor
        )
        output.linear_acceleration_covariance = _rotate_covariance(
            message.linear_acceleration_covariance,
            base_from_sensor,
            self.input_acceleration_scale,
        )
        self.publisher.publish(output)

    def _reset_temporal_state(self, clear_joint_samples):
        if clear_joint_samples:
            self.joint_samples.clear()
            self.last_raw_position = None
            self.last_unwrapped_position = None
        self.pending_imus.clear()
        self.previous_base_omega = None
        self.previous_imu_time = None
        self.filtered_base_alpha = np.zeros(3)
        self.previous_joint_velocity = None
        self.previous_joint_time = None
        self.filtered_joint_acceleration = 0.0

    def _positive_seconds_parameter(self, parameter_name):
        value = float(self.get_parameter(parameter_name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{parameter_name} must be finite and positive')
        return value

    def _nonnegative_seconds_parameter(self, parameter_name):
        value = float(self.get_parameter(parameter_name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f'{parameter_name} must be finite and non-negative')
        return value


def main():
    rclpy.init()
    node = GimbalImuCompensator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
