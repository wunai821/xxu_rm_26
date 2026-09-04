#!/usr/bin/env python3
"""Transform PointCloud2 with TF or vectorized gimbal-joint interpolation."""

from collections import deque
import copy
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


def _rotation_from_rpy(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _rotate_vectors_about_axis(vectors, axis, angles):
    """Apply Rodrigues rotations to corresponding rows of ``vectors``."""
    angles = np.asarray(angles, dtype=float).reshape(-1, 1)
    cosine = np.cos(angles)
    sine = np.sin(angles)
    axis_dot_vector = vectors @ axis
    return (
        vectors * cosine
        + np.cross(axis, vectors) * sine
        + axis * axis_dot_vector[:, None] * (1.0 - cosine)
    )


class PointCloudFrameRepublisher(Node):
    def __init__(self):
        super().__init__("pointcloud_frame_republisher")
        self.declare_parameter("target_frame", "mid360_link")
        self.declare_parameter("expected_source_frame", "")
        self.declare_parameter("tf_lookup_timeout", 0.1)
        self.declare_parameter("use_point_timestamps", False)
        self.declare_parameter("use_joint_interpolation", False)
        self.declare_parameter("joint_name", "gimbal_joint")
        self.declare_parameter("joint_axis", [0.0, 0.0, 1.0])
        self.declare_parameter("sensor_offset", [0.0, 0.08637, 0.0])
        self.declare_parameter("sensor_rpy", [-0.2967059728, 0.0, 0.0])
        self.declare_parameter("joint_position_offset", 0.0)
        self.declare_parameter("max_joint_sample_gap", 0.05)
        self.declare_parameter("time_reset_threshold", 0.5)
        self.declare_parameter("max_tf_timestamp_groups", 256)
        self.declare_parameter("pending_timeout", 0.25)
        self.declare_parameter("pending_queue_size", 3)
        self.target_frame = self.get_parameter("target_frame").value
        self.expected_source_frame = str(
            self.get_parameter("expected_source_frame").value
        ).strip()
        self.tf_lookup_timeout = max(
            0.0, float(self.get_parameter("tf_lookup_timeout").value)
        )
        self.use_point_timestamps = bool(
            self.get_parameter("use_point_timestamps").value
        )
        self.use_joint_interpolation = bool(
            self.get_parameter("use_joint_interpolation").value
        )
        if self.use_joint_interpolation and not self.use_point_timestamps:
            raise ValueError(
                "use_joint_interpolation requires use_point_timestamps=true"
            )
        self.joint_name = str(self.get_parameter("joint_name").value)
        self.joint_axis = np.asarray(
            self.get_parameter("joint_axis").value, dtype=float
        )
        if self.joint_axis.shape != (3,) or not np.all(np.isfinite(self.joint_axis)):
            raise ValueError("joint_axis must contain three finite values")
        axis_norm = np.linalg.norm(self.joint_axis)
        if axis_norm <= np.finfo(float).eps:
            raise ValueError("joint_axis must be non-zero")
        self.joint_axis /= axis_norm
        self.sensor_offset = np.asarray(
            self.get_parameter("sensor_offset").value, dtype=float
        )
        if self.sensor_offset.shape != (3,) or not np.all(
            np.isfinite(self.sensor_offset)
        ):
            raise ValueError("sensor_offset must contain three finite values")
        sensor_rpy = np.asarray(
            self.get_parameter("sensor_rpy").value, dtype=float
        )
        if sensor_rpy.shape != (3,) or not np.all(np.isfinite(sensor_rpy)):
            raise ValueError("sensor_rpy must contain three finite values")
        self.sensor_rotation = _rotation_from_rpy(*sensor_rpy)
        self.joint_position_offset = float(
            self.get_parameter("joint_position_offset").value
        )
        if not np.isfinite(self.joint_position_offset):
            raise ValueError("joint_position_offset must be finite")
        self.max_joint_sample_gap_ns = self._positive_seconds_as_ns(
            "max_joint_sample_gap"
        )
        self.time_reset_threshold_ns = self._positive_seconds_as_ns(
            "time_reset_threshold"
        )
        self.pending_timeout = max(
            0.0, float(self.get_parameter("pending_timeout").value)
        )
        self.pending_queue_size = max(
            1, int(self.get_parameter("pending_queue_size").value)
        )
        self.max_tf_timestamp_groups = max(
            1, int(self.get_parameter("max_tf_timestamp_groups").value)
        )
        self.tf_buffer = None
        self.tf_listener = None
        if not self.use_joint_interpolation:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_tf_warning_ns = 0
        self.transformed_messages = 0
        self.dropped_messages = 0
        self.pending_messages = deque()
        self.joint_samples = deque(maxlen=4096)
        self.last_raw_joint_position = None
        self.last_unwrapped_joint_position = None

        self.sub = self.create_subscription(
            PointCloud2, "points_in", self.callback, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(
            PointCloud2, "points_out", qos_profile_sensor_data
        )
        if self.use_joint_interpolation:
            self.create_subscription(
                JointState,
                "joint_states",
                self._joint_callback,
                qos_profile_sensor_data,
            )
        if self.use_point_timestamps:
            self.create_timer(0.01, self._process_pending_messages)
        self.get_logger().info(
            "Transforming points_in -> points_out "
            f"to frame={self.target_frame} "
            f"(timeout={self.tf_lookup_timeout:.3f}s, "
            f"point_timestamps={self.use_point_timestamps}, "
            f"joint_interpolation={self.use_joint_interpolation})"
        )

    def _joint_callback(self, message: JointState):
        try:
            index = message.name.index(self.joint_name)
        except ValueError:
            return
        if index >= len(message.position):
            return
        position = float(message.position[index])
        timestamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if not np.isfinite(position):
            return
        if self.joint_samples and timestamp_ns <= self.joint_samples[-1][0]:
            backwards_ns = self.joint_samples[-1][0] - timestamp_ns
            if backwards_ns < self.time_reset_threshold_ns:
                return
            discarded = len(self.pending_messages)
            self.dropped_messages += discarded
            self.pending_messages.clear()
            self.joint_samples.clear()
            self.last_raw_joint_position = None
            self.last_unwrapped_joint_position = None
            self.get_logger().warning(
                "Joint timestamp moved backwards; reset interpolation state "
                f"and discarded {discarded} pending clouds"
            )
        elif (
            self.joint_samples
            and timestamp_ns - self.joint_samples[-1][0]
            > self.max_joint_sample_gap_ns
        ):
            gap_ns = timestamp_ns - self.joint_samples[-1][0]
            discarded = len(self.pending_messages)
            self.dropped_messages += discarded
            self.pending_messages.clear()
            self.joint_samples.clear()
            self.last_raw_joint_position = None
            self.last_unwrapped_joint_position = None
            self.get_logger().warning(
                f"Joint samples had a {gap_ns * 1.0e-9:.3f}s gap; reset "
                f"interpolation state and discarded {discarded} pending clouds"
            )

        if self.last_raw_joint_position is None:
            unwrapped = position
        else:
            delta = np.arctan2(
                np.sin(position - self.last_raw_joint_position),
                np.cos(position - self.last_raw_joint_position),
            )
            unwrapped = self.last_unwrapped_joint_position + float(delta)
        self.last_raw_joint_position = position
        self.last_unwrapped_joint_position = unwrapped
        self.joint_samples.append((timestamp_ns, unwrapped))
        self._process_pending_messages()

    def callback(self, msg: PointCloud2):
        source_frame = msg.header.frame_id.strip()
        if not source_frame:
            self._warn_throttled("Dropping PointCloud2 with an empty source frame_id")
            return
        if (
            self.expected_source_frame
            and source_frame != self.expected_source_frame
        ):
            self._warn_throttled(
                f"Dropping cloud from unexpected frame {source_frame}; expected "
                f"{self.expected_source_frame}"
            )
            return

        if source_frame == self.target_frame and not self.use_joint_interpolation:
            self.pub.publish(msg)
            return

        if self.use_point_timestamps:
            if len(self.pending_messages) >= self.pending_queue_size:
                self.pending_messages.popleft()
                self.dropped_messages += 1
                self._warn_throttled(
                    "Dropping oldest pending timestamped cloud because its queue is full"
                )
            self.pending_messages.append((msg, time.monotonic()))
            self._process_pending_messages()
            return

        try:
            # lookup_transform(target, source, stamp) returns target <- source.
            # The timestamp is never replaced with the latest available TF.
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                msg.header.stamp,
                timeout=Duration(seconds=self.tf_lookup_timeout),
            )
            output = self._transform_cloud(msg, transform)
        except TransformException as exc:
            self._warn_throttled(
                f"Dropping cloud at {self._stamp_text(msg)}: "
                f"cannot transform {source_frame} -> {self.target_frame}: {exc}"
            )
            return
        except (AssertionError, ValueError) as exc:
            self._warn_throttled(
                f"Dropping malformed cloud from {source_frame}: {exc}"
            )
            return

        self.transformed_messages += 1
        if self.transformed_messages == 1:
            self.get_logger().info(
                f"Applied TF2 transform {source_frame} -> {self.target_frame} "
                f"at {self._stamp_text(msg)}"
            )
        self.pub.publish(output)

    def _process_pending_messages(self):
        """Publish queued clouds only after every point-time TF is available."""
        while self.pending_messages:
            msg, queued_at = self.pending_messages[0]
            source_frame = msg.header.frame_id.strip()
            try:
                if self.use_joint_interpolation:
                    output = self._transform_cloud_by_joint_time(msg)
                else:
                    output = self._transform_cloud_by_point_time(msg, source_frame)
            except TransformException as exc:
                if time.monotonic() - queued_at <= self.pending_timeout:
                    return
                self.pending_messages.popleft()
                self.dropped_messages += 1
                self._warn_throttled(
                    f"Dropping deferred cloud at {self._stamp_text(msg)} after "
                    f"{self.pending_timeout:.3f}s without complete motion data: {exc}"
                )
                continue
            except (AssertionError, ValueError) as exc:
                self.pending_messages.popleft()
                self.dropped_messages += 1
                self._warn_throttled(
                    f"Dropping malformed deferred cloud from {source_frame}: {exc}"
                )
                continue
            self.pending_messages.popleft()
            if output is None:
                self.dropped_messages += 1
                continue
            self.transformed_messages += 1
            self.pub.publish(output)

    @staticmethod
    def _transform_cloud(msg: PointCloud2, transform) -> PointCloud2:
        """
        Apply target <- source to xyz while preserving every other field.

        ``tf2_sensor_msgs.do_transform_cloud`` assumes fields are tightly
        packed. Gazebo's PointCloudPacked commonly contains padding between
        fields (for example x/y/z at 0/4/8 and ring at 24), so transform the
        three FLOAT32 fields in-place on a copied message instead.
        """
        fields = {field.name: field for field in msg.fields}
        if not all(name in fields for name in ("x", "y", "z")):
            raise AssertionError("PointCloud2 x/y/z fields are required")
        if any(fields[name].datatype != PointField.FLOAT32 for name in ("x", "y", "z")):
            raise ValueError("PointCloud2 x/y/z fields must be FLOAT32")

        output = copy.deepcopy(msg)
        if output.width == 0 or output.height == 0:
            output.header.frame_id = transform.header.frame_id
            return output

        endian = ">" if msg.is_bigendian else "<"
        dtype = np.dtype(f"{endian}f4")
        shape = (msg.height, msg.width)
        strides = (msg.row_step, msg.point_step)

        xyz = {}
        for name in ("x", "y", "z"):
            xyz[name] = np.ndarray(
                shape=shape,
                dtype=dtype,
                buffer=output.data,
                offset=fields[name].offset,
                strides=strides,
            )

        x = xyz["x"].copy()
        y = xyz["y"].copy()
        z = xyz["z"].copy()
        q = transform.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
        if norm <= np.finfo(float).eps:
            raise ValueError("TF contains a zero-length quaternion")
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r02 = 2.0 * (qx * qz + qy * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qx * qw)
        r20 = 2.0 * (qx * qz - qy * qw)
        r21 = 2.0 * (qy * qz + qx * qw)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
        t = transform.transform.translation

        # Gazebo encodes misses as NaN/Inf. Preserve those values without
        # flooding the console with NumPy warnings; downstream filters can
        # remove them as before.
        with np.errstate(invalid="ignore", over="ignore"):
            xyz["x"][:] = r00 * x + r01 * y + r02 * z + t.x
            xyz["y"][:] = r10 * x + r11 * y + r12 * z + t.y
            xyz["z"][:] = r20 * x + r21 * y + r22 * z + t.z
        output.header.frame_id = transform.header.frame_id
        return output

    def _transform_cloud_by_point_time(self, msg: PointCloud2, source_frame):
        """
        Transform a Livox cloud in temporal groups instead of at the header.

        The native MID-360 simulation has a FLOAT64 ``timestamp`` field and
        only a small number of distinct acquisition times (one per raycast
        batch). Looking up one TF per distinct time keeps this tractable while
        preserving the real scan trajectory. The complete cloud stays queued
        until every exact transform is available.
        """
        output, xyz, timestamp_ns = self._copy_timestamped_xyz(msg)
        if output is None:
            return None
        if timestamp_ns.size == 0:
            output.header.frame_id = self.target_frame
            return output

        flat_xyz = {name: values.reshape(-1) for name, values in xyz.items()}
        unique_timestamp_ns = np.unique(timestamp_ns)
        if unique_timestamp_ns.size > self.max_tf_timestamp_groups:
            raise ValueError(
                f"cloud has {unique_timestamp_ns.size} timestamp groups; exact TF "
                "mode is intentionally limited, use joint interpolation"
            )
        for stamp_ns in unique_timestamp_ns:
            indices = np.flatnonzero(timestamp_ns == stamp_ns)
            transform = self._lookup_point_transform(
                source_frame, int(stamp_ns)
            )

            self._apply_transform_to_indices(flat_xyz, indices, transform)

        output.header.frame_id = self.target_frame
        return output

    def _transform_cloud_by_joint_time(self, msg: PointCloud2):
        """
        Transform every point using vectorized interpolated joint position.

        Unlike exact TF lookup per acquisition time, this remains O(N) when a
        Livox driver gives every point a distinct timestamp.
        """
        fields, timestamp_ns = self._read_timestamp_ns(msg)
        if fields is None:
            return None
        if timestamp_ns.size == 0:
            output, _, _ = self._copy_timestamped_xyz(
                msg, fields, timestamp_ns
            )
            output.header.frame_id = self.target_frame
            return output
        if len(self.joint_samples) < 2:
            raise TransformException("fewer than two joint samples are available")

        joint_times = np.fromiter(
            (sample[0] for sample in self.joint_samples), dtype=np.int64
        )
        if timestamp_ns.min() < joint_times[0] or timestamp_ns.max() > joint_times[-1]:
            raise TransformException(
                "point timestamps are not bracketed by joint samples"
            )
        joint_positions = np.fromiter(
            (sample[1] for sample in self.joint_samples), dtype=np.float64
        )
        relative_joint_times = joint_times - joint_times[0]
        relative_point_times = timestamp_ns - joint_times[0]
        upper_indices = np.searchsorted(joint_times, timestamp_ns, side="left")
        upper_indices = np.minimum(upper_indices, joint_times.size - 1)
        lower_indices = np.maximum(upper_indices - 1, 0)
        interpolation_gaps = joint_times[upper_indices] - joint_times[lower_indices]
        interpolation_gaps[timestamp_ns == joint_times[upper_indices]] = 0
        if np.any(interpolation_gaps > self.max_joint_sample_gap_ns):
            largest_gap = int(interpolation_gaps.max())
            raise ValueError(
                "joint samples contain an interpolation gap of "
                f"{largest_gap * 1.0e-9:.3f}s"
            )
        point_positions = np.interp(
            relative_point_times, relative_joint_times, joint_positions
        ) + self.joint_position_offset

        # Delay the expensive message copy until complete joint coverage is
        # available. While waiting, the retry timer only scans timestamps.
        output, xyz, _ = self._copy_timestamped_xyz(
            msg, fields, timestamp_ns
        )
        points = np.column_stack(
            (xyz["x"].reshape(-1), xyz["y"].reshape(-1), xyz["z"].reshape(-1))
        ).astype(np.float64, copy=True)
        with np.errstate(invalid="ignore", over="ignore"):
            vectors_at_zero = points @ self.sensor_rotation.T + self.sensor_offset
            transformed = _rotate_vectors_about_axis(
                vectors_at_zero, self.joint_axis, point_positions
            )
            xyz["x"][:] = transformed[:, 0].reshape(xyz["x"].shape)
            xyz["y"][:] = transformed[:, 1].reshape(xyz["y"].shape)
            xyz["z"][:] = transformed[:, 2].reshape(xyz["z"].shape)
        output.header.frame_id = self.target_frame
        return output

    def _read_timestamp_ns(self, msg: PointCloud2):
        fields = {field.name: field for field in msg.fields}
        if not all(name in fields for name in ("x", "y", "z", "timestamp")):
            self._warn_throttled(
                "Dropping timestamped cloud: x/y/z/timestamp fields are required"
            )
            return None, None
        if any(
            fields[name].datatype != PointField.FLOAT32 for name in ("x", "y", "z")
        ) or fields["timestamp"].datatype != PointField.FLOAT64:
            self._warn_throttled(
                "Dropping timestamped cloud: xyz must be FLOAT32 and "
                "timestamp must be FLOAT64"
            )
            return None, None
        if msg.width == 0 or msg.height == 0:
            return fields, np.empty(0, dtype=np.int64)

        endian = ">" if msg.is_bigendian else "<"
        shape = (msg.height, msg.width)
        strides = (msg.row_step, msg.point_step)
        timestamps = np.ndarray(
            shape=shape,
            dtype=np.dtype(f"{endian}f8"),
            buffer=msg.data,
            offset=fields["timestamp"].offset,
            strides=strides,
        ).reshape(-1)
        if not np.all(np.isfinite(timestamps)):
            self._warn_throttled(
                "Dropping timestamped cloud containing non-finite timestamps"
            )
            return None, None
        # livox_ros_driver2 stores absolute nanoseconds in FLOAT64. Precision
        # at Unix-epoch magnitude is about 256 ns, still far below the joint
        # sampling interval and MID360 timing accuracy.
        return fields, np.rint(timestamps).astype(np.int64)

    def _copy_timestamped_xyz(
        self, msg: PointCloud2, fields=None, timestamp_ns=None
    ):
        if fields is None or timestamp_ns is None:
            fields, timestamp_ns = self._read_timestamp_ns(msg)
            if fields is None:
                return None, None, None
        output = copy.deepcopy(msg)
        if msg.width == 0 or msg.height == 0:
            return output, {}, timestamp_ns
        endian = ">" if msg.is_bigendian else "<"
        shape = (msg.height, msg.width)
        strides = (msg.row_step, msg.point_step)
        xyz = {
            name: np.ndarray(
                shape=shape,
                dtype=np.dtype(f"{endian}f4"),
                buffer=output.data,
                offset=fields[name].offset,
                strides=strides,
            )
            for name in ("x", "y", "z")
        }
        return output, xyz, timestamp_ns

    def _lookup_point_transform(self, source_frame, stamp_ns):
        """Look up target <- source at one exact point acquisition time."""
        return self.tf_buffer.lookup_transform(
            self.target_frame,
            source_frame,
            Time(nanoseconds=stamp_ns),
            timeout=Duration(seconds=self.tf_lookup_timeout),
        )

    @staticmethod
    def _apply_transform_to_indices(xyz, indices, transform):
        q = transform.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
        if norm <= np.finfo(float).eps:
            raise ValueError("TF contains a zero-length quaternion")
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r02 = 2.0 * (qx * qz + qy * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qx * qw)
        r20 = 2.0 * (qx * qz - qy * qw)
        r21 = 2.0 * (qy * qz + qx * qw)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
        t = transform.transform.translation

        x = xyz["x"][indices].copy()
        y = xyz["y"][indices].copy()
        z = xyz["z"][indices].copy()
        with np.errstate(invalid="ignore", over="ignore"):
            xyz["x"][indices] = r00 * x + r01 * y + r02 * z + t.x
            xyz["y"][indices] = r10 * x + r11 * y + r12 * z + t.y
            xyz["z"][indices] = r20 * x + r21 * y + r22 * z + t.z

    @staticmethod
    def _stamp_text(msg: PointCloud2) -> str:
        return f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"

    def _warn_throttled(self, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_tf_warning_ns >= 1_000_000_000:
            self.get_logger().warning(message)
            self.last_tf_warning_ns = now_ns

    def _positive_seconds_as_ns(self, parameter_name):
        seconds = float(self.get_parameter(parameter_name).value)
        if not np.isfinite(seconds) or seconds <= 0.0:
            raise ValueError(f"{parameter_name} must be finite and positive")
        return int(round(seconds * 1_000_000_000))


def main():
    rclpy.init()
    node = PointCloudFrameRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
