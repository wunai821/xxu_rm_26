#!/usr/bin/env python3
"""Inspect timing, point validity, gimbal motion and odometry in a ROS 2 bag."""

import argparse
import bisect
import json
import math
import statistics
import struct
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointField


NUMERIC_FIELDS = {
    PointField.INT8: ("b", 1),
    PointField.UINT8: ("B", 1),
    PointField.INT16: ("h", 2),
    PointField.UINT16: ("H", 2),
    PointField.INT32: ("i", 4),
    PointField.UINT32: ("I", 4),
    PointField.FLOAT32: ("f", 4),
    PointField.FLOAT64: ("d", 8),
}


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def message_time(message, fallback_ns):
    if hasattr(message, "header"):
        value = stamp_seconds(message.header.stamp)
        if value > 0.0:
            return value
    return fallback_ns * 1.0e-9


def rate_summary(times):
    intervals = [after - before for before, after in zip(times, times[1:])]
    positive_intervals = [interval for interval in intervals if interval > 0.0]
    timing = {
        "non_monotonic_timestamps": sum(interval < 0.0 for interval in intervals),
        "duplicate_timestamps": sum(interval == 0.0 for interval in intervals),
        "median_interval_s": (
            statistics.median(positive_intervals) if positive_intervals else None
        ),
        "max_interval_s": max(positive_intervals) if positive_intervals else None,
    }
    if len(times) < 2:
        result = {
            "messages": len(times),
            "duration_s": 0.0,
            "rate_hz": None,
            "start_time_s": times[0] if times else None,
            "end_time_s": times[0] if times else None,
        }
        result.update(timing)
        return result
    duration = max(times) - min(times)
    rate = (len(times) - 1) / duration if duration > 0.0 else None
    result = {
        "messages": len(times),
        "duration_s": duration,
        "rate_hz": rate,
        "start_time_s": min(times),
        "end_time_s": max(times),
    }
    result.update(timing)
    return result


def percentile(values, ratio):
    if not values:
        return None
    ordered = sorted(values)
    index = ratio * (len(ordered) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def scalar_summary(values):
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return None
    return {
        "min": min(finite),
        "median": statistics.median(finite),
        "p95": percentile(finite, 0.95),
        "max": max(finite),
    }


def field_reader(message, field):
    if field.datatype not in NUMERIC_FIELDS or field.count != 1:
        return None
    format_code, size = NUMERIC_FIELDS[field.datatype]
    endian = ">" if message.is_bigendian else "<"
    unpack = struct.Struct(endian + format_code).unpack_from

    def read(point_offset):
        offset = point_offset + field.offset
        if offset < 0 or offset + size > len(message.data):
            raise ValueError("PointCloud2 field exceeds data buffer")
        return unpack(message.data, offset)[0]

    return read


def point_offsets(message):
    for row in range(message.height):
        row_offset = row * message.row_step
        for column in range(message.width):
            yield row_offset + column * message.point_step


def infer_point_times(raw_values, header_time, field_name):
    """Return seconds, scale and mode using the header as the absolute-time anchor."""
    if not raw_values:
        return [], None, "missing"
    raw_median = statistics.median(raw_values)
    scales = (1.0, 1.0e-3, 1.0e-6, 1.0e-9)
    absolute = min(scales, key=lambda scale: abs(raw_median * scale - header_time))
    absolute_error = abs(raw_median * absolute - header_time)
    if absolute_error <= 5.0:
        return [value * absolute for value in raw_values], absolute, "absolute"

    raw_span = max(raw_values) - min(raw_values)
    plausible = [scale for scale in scales if 1.0e-5 <= raw_span * scale <= 1.0]
    scale = min(
        plausible or scales,
        key=lambda candidate: abs(raw_span * candidate - 0.1),
    )
    base = header_time if field_name in ("offset_time", "time", "t") else 0.0
    return [base + value * scale for value in raw_values], scale, "offset"


class CloudStats:
    def __init__(self):
        self.times = []
        self.fields = set()
        self.frames = set()
        self.cloud_points = []
        self.total_points = 0
        self.finite_xyz = 0
        self.time_field = None
        self.time_datatypes = set()
        self.scan_spans = []
        self.time_groups = []
        self.time_scales = []
        self.midpoint_header_errors = []
        self.point_time_min = None
        self.point_time_max = None
        self.scan_time_ranges = []
        self.previous_timestamp_groups = None
        self.shared_timestamp_groups = []

    def add(self, message, bag_time_ns):
        header_time = message_time(message, bag_time_ns)
        self.times.append(header_time)
        self.frames.add(message.header.frame_id)
        fields = {field.name: field for field in message.fields}
        self.fields.update(fields)
        count = int(message.width) * int(message.height)
        self.cloud_points.append(count)
        self.total_points += count

        xyz_readers = [
            field_reader(message, fields.get(name))
            for name in ("x", "y", "z")
            if name in fields
        ]
        timestamp_name = next(
            (
                name
                for name in ("timestamp", "offset_time", "time", "t")
                if name in fields
            ),
            None,
        )
        timestamp_reader = (
            field_reader(message, fields[timestamp_name]) if timestamp_name else None
        )
        raw_times = []
        for offset in point_offsets(message):
            if len(xyz_readers) == 3:
                xyz = [reader(offset) for reader in xyz_readers]
                if all(math.isfinite(float(value)) for value in xyz):
                    self.finite_xyz += 1
            if timestamp_reader is not None:
                value = float(timestamp_reader(offset))
                if math.isfinite(value):
                    raw_times.append(value)

        if raw_times:
            self.time_field = timestamp_name
            self.time_datatypes.add(int(fields[timestamp_name].datatype))
            point_times, scale, mode = infer_point_times(
                raw_times, header_time, timestamp_name
            )
            timestamp_groups = set(raw_times)
            self.scan_spans.append(max(point_times) - min(point_times))
            self.time_groups.append(len(timestamp_groups))
            self.time_scales.append({"scale": scale, "mode": mode})
            cloud_min = min(point_times)
            cloud_max = max(point_times)
            self.scan_time_ranges.append((cloud_min, cloud_max))
            if self.previous_timestamp_groups is not None:
                self.shared_timestamp_groups.append(
                    len(timestamp_groups & self.previous_timestamp_groups)
                )
            self.previous_timestamp_groups = timestamp_groups
            self.point_time_min = (
                cloud_min
                if self.point_time_min is None
                else min(self.point_time_min, cloud_min)
            )
            self.point_time_max = (
                cloud_max
                if self.point_time_max is None
                else max(self.point_time_max, cloud_max)
            )
            if mode == "absolute":
                midpoint = 0.5 * (cloud_min + cloud_max)
                self.midpoint_header_errors.append(midpoint - header_time)
        else:
            self.scan_time_ranges.append(None)
            self.previous_timestamp_groups = None

    def report(self):
        result = rate_summary(self.times)
        scan_boundary_gaps = [
            after[0] - before[1]
            for before, after in zip(
                self.scan_time_ranges, self.scan_time_ranges[1:]
            )
            if before is not None and after is not None
        ]
        result.update(
            {
                "frames": sorted(self.frames),
                "fields": sorted(self.fields),
                "points_per_cloud": scalar_summary(self.cloud_points),
                "total_points": self.total_points,
                "finite_xyz_ratio": (
                    self.finite_xyz / self.total_points if self.total_points else None
                ),
                "point_time_field": self.time_field,
                "point_time_datatypes": sorted(self.time_datatypes),
                "scan_span_s": scalar_summary(self.scan_spans),
                "time_groups_per_cloud": scalar_summary(self.time_groups),
                "point_time_interpretations": [
                    {"mode": mode, "scale": scale}
                    for mode, scale in sorted(
                        {(item["mode"], item["scale"]) for item in self.time_scales}
                    )
                ],
                "point_time_start_s": self.point_time_min,
                "point_time_end_s": self.point_time_max,
                "point_time_midpoint_minus_header_s": scalar_summary(
                    self.midpoint_header_errors
                ),
                "scan_boundary_gap_s": scalar_summary(scan_boundary_gaps),
                "overlapping_scan_boundaries": sum(
                    gap < -1.0e-6 for gap in scan_boundary_gaps
                ),
                "touching_scan_boundaries": sum(
                    abs(gap) <= 1.0e-6 for gap in scan_boundary_gaps
                ),
                "shared_timestamp_groups_between_clouds": scalar_summary(
                    self.shared_timestamp_groups
                ),
            }
        )
        return result


class ImuStats:
    def __init__(self):
        self.times = []
        self.angular_norms = []

    def add(self, message, bag_time_ns):
        self.times.append(message_time(message, bag_time_ns))
        angular = message.angular_velocity
        self.angular_norms.append(
            math.sqrt(
                angular.x * angular.x + angular.y * angular.y + angular.z * angular.z
            )
        )

    def report(self):
        result = rate_summary(self.times)
        result["angular_speed_rad_s"] = scalar_summary(self.angular_norms)
        return result


class JointStats:
    def __init__(self, joint_name):
        self.joint_name = joint_name
        self.times = []
        self.positions = []
        self.velocities = []
        self.messages = 0
        self.missing_joint_messages = 0

    def add(self, message, bag_time_ns):
        self.messages += 1
        try:
            index = list(message.name).index(self.joint_name)
        except ValueError:
            self.missing_joint_messages += 1
            return
        if index >= len(message.position) or index >= len(message.velocity):
            self.missing_joint_messages += 1
            return
        self.times.append(message_time(message, bag_time_ns))
        self.positions.append(float(message.position[index]))
        self.velocities.append(float(message.velocity[index]))

    def report(self):
        result = rate_summary(self.times)
        result.update(
            {
                "joint_name": self.joint_name,
                "source_messages": self.messages,
                "messages_without_joint": self.missing_joint_messages,
                "position_rad": scalar_summary(self.positions),
                "abs_velocity_rad_s": scalar_summary(
                    abs(value) for value in self.velocities
                ),
            }
        )
        return result


class OdomStats:
    def __init__(self):
        self.times = []
        self.positions = []
        self.frames = set()

    def add(self, message, bag_time_ns):
        self.times.append(message_time(message, bag_time_ns))
        pose = message.pose.pose.position
        self.positions.append((float(pose.x), float(pose.y), float(pose.z)))
        self.frames.add((message.header.frame_id, message.child_frame_id))

    def report(self):
        result = rate_summary(self.times)
        path_3d = 0.0
        path_xy = 0.0
        max_from_start = 0.0
        if self.positions:
            start = self.positions[0]
            for before, after in zip(self.positions, self.positions[1:]):
                path_3d += math.dist(before, after)
                path_xy += math.hypot(after[0] - before[0], after[1] - before[1])
            max_from_start = max(
                math.dist(start, position) for position in self.positions
            )
            net_3d = math.dist(start, self.positions[-1])
            net_xy = math.hypot(
                self.positions[-1][0] - start[0], self.positions[-1][1] - start[1]
            )
        else:
            net_3d = 0.0
            net_xy = 0.0
        result.update(
            {
                "frames": [list(frame_pair) for frame_pair in sorted(self.frames)],
                "path_length_m": path_3d,
                "path_length_xy_m": path_xy,
                "net_displacement_m": net_3d,
                "net_displacement_xy_m": net_xy,
                "max_displacement_from_start_m": max_from_start,
                "max_displacement_xy_from_start_m": (
                    max(
                        math.hypot(position[0] - start[0], position[1] - start[1])
                        for position in self.positions
                    )
                    if self.positions
                    else 0.0
                ),
            }
        )
        return result


def correlation(left, right):
    if len(left) < 3 or len(left) != len(right):
        return None
    left_mean = statistics.mean(left)
    right_mean = statistics.mean(right)
    numerator = sum((a - left_mean) * (b - right_mean) for a, b in zip(left, right))
    left_energy = sum((value - left_mean) ** 2 for value in left)
    right_energy = sum((value - right_mean) ** 2 for value in right)
    denominator = math.sqrt(left_energy * right_energy)
    return numerator / denominator if denominator > 0.0 else None


def compare_imu_joint(imu, joint):
    if len(joint.times) < 2:
        return None
    joint_samples = sorted(zip(joint.times, joint.velocities))
    joint_times = [sample[0] for sample in joint_samples]
    joint_velocities = [sample[1] for sample in joint_samples]
    joint_speed = []
    imu_speed = []
    for timestamp, angular_norm in zip(imu.times, imu.angular_norms):
        upper = bisect.bisect_left(joint_times, timestamp)
        if upper == 0 or upper >= len(joint_times):
            continue
        before = upper - 1
        interval = joint_times[upper] - joint_times[before]
        if interval <= 0.0 or interval > 0.1:
            continue
        ratio = (timestamp - joint_times[before]) / interval
        velocity = joint_velocities[before] + ratio * (
            joint_velocities[upper] - joint_velocities[before]
        )
        joint_speed.append(abs(velocity))
        imu_speed.append(abs(angular_norm))
    if not joint_speed:
        return None
    errors = [
        abs(imu_value - joint_value)
        for imu_value, joint_value in zip(imu_speed, joint_speed)
    ]
    return {
        "samples": len(errors),
        "abs_speed_correlation": correlation(imu_speed, joint_speed),
        "abs_speed_error_rad_s": scalar_summary(errors),
    }


def inspect_bag(args):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    available = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected = {
        args.lidar_topic: CloudStats(),
        args.imu_topic: ImuStats(),
        args.joint_topic: JointStats(args.joint_name),
        args.lio_odom_topic: OdomStats(),
    }
    if args.reference_odom:
        selected[args.reference_odom] = OdomStats()
    expected_types = {
        args.lidar_topic: "sensor_msgs/msg/PointCloud2",
        args.imu_topic: "sensor_msgs/msg/Imu",
        args.joint_topic: "sensor_msgs/msg/JointState",
        args.lio_odom_topic: "nav_msgs/msg/Odometry",
    }
    if args.reference_odom:
        expected_types[args.reference_odom] = "nav_msgs/msg/Odometry"
    message_types = {
        topic: get_message(available[topic])
        for topic in selected
        if topic in available and available[topic] == expected_types[topic]
    }
    bag_times = []
    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        bag_times.append(bag_time_ns * 1.0e-9)
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        selected[topic].add(message, bag_time_ns)

    reports = {}
    for topic, stats in selected.items():
        if topic not in available:
            reports[topic] = {"missing": True, "expected_type": expected_types[topic]}
        elif available[topic] != expected_types[topic]:
            reports[topic] = {
                "type_mismatch": True,
                "expected_type": expected_types[topic],
                "actual_type": available[topic],
            }
        else:
            reports[topic] = stats.report()
    imu_joint = (
        compare_imu_joint(selected[args.imu_topic], selected[args.joint_topic])
        if args.imu_topic in message_types and args.joint_topic in message_types
        else None
    )
    duration = max(bag_times) - min(bag_times) if len(bag_times) > 1 else 0.0
    return {
        "bag": str(args.bag),
        "duration_s": duration,
        "available_topics": available,
        "topics": reports,
        "imu_joint_comparison": imu_joint,
        "warnings": make_warnings(args, reports),
    }


def make_warnings(args, reports):
    warnings = []
    required_topics = [
        args.lidar_topic,
        args.imu_topic,
        args.joint_topic,
        args.lio_odom_topic,
    ]
    if args.reference_odom:
        required_topics.append(args.reference_odom)
    for topic in required_topics:
        topic_report = reports.get(topic, {})
        if topic_report.get("missing"):
            warnings.append(f"missing required topic {topic}")
        elif topic_report.get("type_mismatch"):
            warnings.append(
                f"required topic {topic} has type {topic_report['actual_type']}, "
                f"expected {topic_report['expected_type']}"
            )
        elif topic_report.get("messages", 0) == 0:
            warnings.append(f"required topic {topic} contains no usable messages")

    cloud = reports.get(args.lidar_topic, {})
    if not cloud.get("missing") and not cloud.get("type_mismatch"):
        if cloud.get("total_points", 0) == 0:
            warnings.append("LiDAR topic contains no points")
        ratio = cloud.get("finite_xyz_ratio")
        if ratio is not None and ratio < 0.999:
            warnings.append(f"only {ratio:.2%} of LiDAR xyz samples are finite")
        if cloud.get("point_time_field") != "timestamp" or cloud.get(
            "point_time_datatypes"
        ) != [PointField.FLOAT64]:
            warnings.append(
                "LiDAR does not contain the FLOAT64 timestamp field expected by the Livox adapter"
            )
        interpretations = cloud.get("point_time_interpretations", [])
        if any(
            item.get("mode") != "absolute"
            or not math.isclose(
                item.get("scale", 0.0),
                1.0e-9,
                rel_tol=0.0,
                abs_tol=1.0e-15,
            )
            for item in interpretations
        ):
            warnings.append(
                "LiDAR timestamp is not absolute nanoseconds; the Livox adapter always multiplies it by 1e-9"
            )
        span = (cloud.get("scan_span_s") or {}).get("median")
        if span is not None and not 0.05 <= span <= 0.15:
            warnings.append(
                f"median LiDAR scan span is {span:.6f}s, expected about 0.1s"
            )
        groups = (cloud.get("time_groups_per_cloud") or {}).get("median")
        if groups is not None and groups < 2:
            warnings.append(
                "LiDAR point timestamps have fewer than two acquisition groups"
            )
        header_error = (cloud.get("point_time_midpoint_minus_header_s") or {}).get(
            "median"
        )
        if header_error is not None and abs(header_error) > 0.2:
            warnings.append(
                f"LiDAR point-time domain differs from the cloud header by {header_error:.3f}s"
            )
        overlaps = cloud.get("overlapping_scan_boundaries", 0)
        if overlaps:
            boundary_gap = cloud.get("scan_boundary_gap_s") or {}
            warnings.append(
                f"LiDAR has {overlaps} overlapping scan boundaries; minimum "
                f"next-start minus previous-end is "
                f"{boundary_gap.get('min', 0.0):.6f}s"
            )
        shared_groups = cloud.get("shared_timestamp_groups_between_clouds") or {}
        if shared_groups.get("max", 0) > 0:
            warnings.append(
                "adjacent LiDAR clouds share timestamp groups; inspect whether "
                "they contain duplicate or distinct rays"
            )

    imu = reports.get(args.imu_topic, {})
    if (
        not cloud.get("missing")
        and not imu.get("missing")
        and cloud.get("point_time_start_s") is not None
        and imu.get("start_time_s") is not None
    ):
        overlap = min(cloud["point_time_end_s"], imu["end_time_s"]) - max(
            cloud["point_time_start_s"], imu["start_time_s"]
        )
        if overlap < 0.0:
            warnings.append("LiDAR point timestamps and IMU timestamps do not overlap")

    imu_rate = reports.get(args.imu_topic, {}).get("rate_hz")
    if imu_rate is not None and imu_rate < 80.0:
        warnings.append(f"IMU rate is only {imu_rate:.1f}Hz")
    joint_rate = reports.get(args.joint_topic, {}).get("rate_hz")
    if joint_rate is not None and joint_rate < 50.0:
        warnings.append(f"joint-state rate is only {joint_rate:.1f}Hz")
    joint = reports.get(args.joint_topic, {})
    source_messages = joint.get("source_messages", 0)
    missing_joint = joint.get("messages_without_joint", 0)
    if source_messages and missing_joint:
        warnings.append(
            f"gimbal joint is missing or incomplete in {missing_joint}/{source_messages} JointState messages"
        )

    odom_topics = [args.lio_odom_topic]
    if args.reference_odom:
        odom_topics.append(args.reference_odom)
    for topic in odom_topics:
        if reports.get(topic, {}).get("messages", 0) == 1:
            warnings.append(f"odometry topic {topic} has only one usable message")
    for topic in required_topics:
        topic_report = reports.get(topic, {})
        non_monotonic = topic_report.get("non_monotonic_timestamps", 0)
        duplicates = topic_report.get("duplicate_timestamps", 0)
        if non_monotonic:
            warnings.append(
                f"topic {topic} has {non_monotonic} non-monotonic header timestamps"
            )
        if duplicates:
            warnings.append(
                f"topic {topic} has {duplicates} duplicate header timestamps"
            )
    return warnings


def format_number(value, digits=3):
    return "n/a" if value is None else f"{value:.{digits}f}"


def print_report(report, args):
    print(f"Bag: {report['bag']}")
    print(f"Duration: {report['duration_s']:.3f} s")
    for topic, values in report["topics"].items():
        print(f"\n{topic}")
        if values.get("missing"):
            print("  MISSING")
            continue
        if values.get("type_mismatch"):
            print(
                f"  TYPE MISMATCH: {values['actual_type']} "
                f"(expected {values['expected_type']})"
            )
            continue
        print(
            f"  messages={values['messages']} rate={format_number(values['rate_hz'])} Hz"
        )
        if topic == args.lidar_topic:
            points = values.get("points_per_cloud") or {}
            spans = values.get("scan_span_s") or {}
            groups = values.get("time_groups_per_cloud") or {}
            print(
                "  points/cloud median={} finite_xyz={} frame={}".format(
                    format_number(points.get("median"), 0),
                    format_number(values.get("finite_xyz_ratio"), 5),
                    ",".join(values.get("frames", [])),
                )
            )
            print(
                "  point_time={} span_median={} s groups_median={} interpretation={}".format(
                    values.get("point_time_field") or "missing",
                    format_number(spans.get("median"), 6),
                    format_number(groups.get("median"), 0),
                    values.get("point_time_interpretations") or "missing",
                )
            )
            boundary = values.get("scan_boundary_gap_s") or {}
            shared = values.get("shared_timestamp_groups_between_clouds") or {}
            print(
                "  boundary_gap_median={} s overlaps={} touching={} shared_groups_max={}".format(
                    format_number(boundary.get("median"), 6),
                    values.get("overlapping_scan_boundaries", 0),
                    values.get("touching_scan_boundaries", 0),
                    format_number(shared.get("max"), 0),
                )
            )
        elif topic == args.imu_topic:
            angular = values.get("angular_speed_rad_s") or {}
            print(
                f"  angular_speed_median={format_number(angular.get('median'))} rad/s"
            )
        elif topic == args.joint_topic:
            speed = values.get("abs_velocity_rad_s") or {}
            print(
                f"  joint={args.joint_name} abs_speed_median={format_number(speed.get('median'))} rad/s"
            )
        else:
            print(
                "  net_xy={} m path_xy={} m max_xy_from_start={} m".format(
                    format_number(values.get("net_displacement_xy_m"), 4),
                    format_number(values.get("path_length_xy_m"), 4),
                    format_number(values.get("max_displacement_xy_from_start_m"), 4),
                )
            )
    comparison = report.get("imu_joint_comparison")
    if comparison:
        error = comparison.get("abs_speed_error_rad_s") or {}
        print("\nIMU vs gimbal joint")
        print(
            "  samples={} abs_speed_correlation={} median_abs_error={} rad/s".format(
                comparison["samples"],
                format_number(comparison.get("abs_speed_correlation")),
                format_number(error.get("median")),
            )
        )
    if report["warnings"]:
        print("\nWarnings:")
        for warning in report["warnings"]:
            print(f"  - {warning}")
    else:
        print("\nNo timing/format warnings detected.")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory or MCAP file URI")
    parser.add_argument("--lidar-topic", default="/livox/lidar")
    parser.add_argument("--imu-topic", default="/livox/imu")
    parser.add_argument("--joint-topic", default="/joint_states")
    parser.add_argument("--joint-name", default="gimbal_joint")
    parser.add_argument("--lio-odom-topic", default="/odom")
    parser.add_argument("--reference-odom", default="")
    parser.add_argument(
        "--json", action="store_true", help="emit machine-readable JSON"
    )
    args = parser.parse_args()
    if args.reference_odom and args.reference_odom == args.lio_odom_topic:
        parser.error("--reference-odom must differ from --lio-odom-topic")
    return args


def main():
    args = parse_args()
    try:
        report = inspect_bag(args)
    except Exception as exc:
        print(f"Failed to inspect bag: {exc}", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(report, indent=2, ensure_ascii=False, sort_keys=True))
    else:
        print_report(report, args)
    return 0 if not report["warnings"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
