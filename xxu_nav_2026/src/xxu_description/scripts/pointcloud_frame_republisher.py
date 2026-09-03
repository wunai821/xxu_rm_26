#!/usr/bin/env python3
"""Transform PointCloud2 with exact message or per-point timestamp TF."""

import copy
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


class PointCloudFrameRepublisher(Node):
    def __init__(self):
        super().__init__("pointcloud_frame_republisher")
        self.declare_parameter("target_frame", "mid360_link")
        self.declare_parameter("tf_lookup_timeout", 0.1)
        self.declare_parameter("use_point_timestamps", False)
        self.declare_parameter("pending_timeout", 0.25)
        self.declare_parameter("pending_queue_size", 3)
        self.target_frame = self.get_parameter("target_frame").value
        self.tf_lookup_timeout = max(
            0.0, float(self.get_parameter("tf_lookup_timeout").value)
        )
        self.use_point_timestamps = bool(
            self.get_parameter("use_point_timestamps").value
        )
        self.pending_timeout = max(
            0.0, float(self.get_parameter("pending_timeout").value)
        )
        self.pending_queue_size = max(
            1, int(self.get_parameter("pending_queue_size").value)
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_tf_warning_ns = 0
        self.transformed_messages = 0
        self.dropped_messages = 0
        self.pending_messages = deque()

        self.sub = self.create_subscription(
            PointCloud2, "points_in", self.callback, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(
            PointCloud2, "points_out", qos_profile_sensor_data
        )
        if self.use_point_timestamps:
            self.create_timer(0.002, self._process_pending_messages)
        self.get_logger().info(
            "Transforming points_in -> points_out "
            f"to frame={self.target_frame} "
            f"(timeout={self.tf_lookup_timeout:.3f}s, "
            f"point_timestamps={self.use_point_timestamps})"
        )

    def callback(self, msg: PointCloud2):
        source_frame = msg.header.frame_id.strip()
        if not source_frame:
            self._warn_throttled("Dropping PointCloud2 with an empty source frame_id")
            return

        if source_frame == self.target_frame:
            self.pub.publish(msg)
            return

        if self.use_point_timestamps:
            if len(self.pending_messages) >= self.pending_queue_size:
                self.pending_messages.popleft()
                self.dropped_messages += 1
                self._warn_throttled(
                    "Dropping oldest pending cloud because the exact-TF queue is full"
                )
            self.pending_messages.append((copy.deepcopy(msg), time.monotonic()))
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
                output = self._transform_cloud_by_point_time(msg, source_frame)
            except TransformException as exc:
                if time.monotonic() - queued_at <= self.pending_timeout:
                    return
                self.pending_messages.popleft()
                self.dropped_messages += 1
                self._warn_throttled(
                    f"Dropping deferred cloud at {self._stamp_text(msg)} after "
                    f"{self.pending_timeout:.3f}s without exact TF: {exc}"
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
        """Apply target <- source to xyz while preserving every other field.

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
        """Transform a Livox cloud in temporal groups instead of at the header.

        The native MID-360 simulation has a FLOAT64 ``timestamp`` field and
        only a small number of distinct acquisition times (one per raycast
        batch). Looking up one TF per distinct time keeps this tractable while
        preserving the real scan trajectory. The complete cloud stays queued
        until every exact transform is available.
        """
        fields = {field.name: field for field in msg.fields}
        if not all(name in fields for name in ("x", "y", "z", "timestamp")):
            self._warn_throttled(
                "Dropping timestamped cloud: x/y/z/timestamp fields are required"
            )
            return None
        if any(
            fields[name].datatype != PointField.FLOAT32
            for name in ("x", "y", "z")
        ) or fields["timestamp"].datatype != PointField.FLOAT64:
            self._warn_throttled(
                "Dropping timestamped cloud: xyz must be FLOAT32 and "
                "timestamp must be FLOAT64"
            )
            return None

        output = copy.deepcopy(msg)
        if output.width == 0 or output.height == 0:
            output.header.frame_id = self.target_frame
            return output

        endian = ">" if msg.is_bigendian else "<"
        xyz_dtype = np.dtype(f"{endian}f4")
        timestamp_dtype = np.dtype(f"{endian}f8")
        shape = (msg.height, msg.width)
        strides = (msg.row_step, msg.point_step)

        xyz = {
            name: np.ndarray(
                shape=shape,
                dtype=xyz_dtype,
                buffer=output.data,
                offset=fields[name].offset,
                strides=strides,
            )
            for name in ("x", "y", "z")
        }
        timestamps = np.ndarray(
            shape=shape,
            dtype=timestamp_dtype,
            buffer=msg.data,
            offset=fields["timestamp"].offset,
            strides=strides,
        ).reshape(-1).copy()
        if not np.all(np.isfinite(timestamps)):
            self._warn_throttled(
                "Dropping timestamped cloud containing non-finite timestamps"
            )
            return None
        # Livox PointCloud2 stores this field as absolute nanoseconds. The
        # LIO adapter converts it with *1e-9, so do not scale it again here.
        timestamp_ns = np.rint(timestamps).astype(np.int64)

        flat_xyz = {name: values.reshape(-1) for name, values in xyz.items()}
        unique_timestamp_ns = np.unique(timestamp_ns)
        for stamp_ns in unique_timestamp_ns:
            indices = np.flatnonzero(timestamp_ns == stamp_ns)
            transform = self._lookup_point_transform(
                source_frame, int(stamp_ns)
            )

            self._apply_transform_to_indices(flat_xyz, indices, transform)

        output.header.frame_id = self.target_frame
        return output

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
