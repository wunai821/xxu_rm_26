#!/usr/bin/env python3
"""Check whether raw MID-360 clouds move consistently with Gazebo pose truth.

The input bag stores /mid360/livox_points in the sensor frame.  This tool
uses the dynamic-pose JSONL recorded from Gazebo to build the sensor world
pose (xxu model -> gimbal_link -> MID-360 sensor_pose), then compares two
short cloud windows before and during a chassis translation.  A static scene
should have better nearest-neighbour and voxel overlap after truth alignment
than in the raw sensor frame.
"""

import argparse
import bisect
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def vector(value):
    return np.array([value.get("x", 0.0), value.get("y", 0.0), value.get("z", 0.0)], dtype=float)


def quaternion_matrix(value):
    x = float(value.get("x", 0.0))
    y = float(value.get("y", 0.0))
    z = float(value.get("z", 0.0))
    w = float(value.get("w", 1.0))
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)


def load_truth(path):
    samples = []
    for line in Path(path).read_text().splitlines():
        message = json.loads(line)
        poses = {pose["name"]: pose for pose in message.get("pose", [])}
        if "xxu" not in poses or "gimbal_link" not in poses:
            continue
        stamp = message["header"]["stamp"]
        # Gazebo omits the zero-valued nanosecond field in some JSON records.
        samples.append((
            float(stamp.get("sec", 0.0))
            + float(stamp.get("nsec", stamp.get("nanosec", 0.0))) * 1e-9,
            vector(poses["xxu"].get("position", {})),
            quaternion_matrix(poses["xxu"].get("orientation", {})),
            vector(poses["gimbal_link"].get("position", {})),
            quaternion_matrix(poses["gimbal_link"].get("orientation", {})),
        ))
    if len(samples) < 2:
        raise RuntimeError("truth JSONL lacks xxu/gimbal_link poses")
    return samples


def interpolate_truth(samples, timestamp):
    times = [item[0] for item in samples]
    index = bisect.bisect_left(times, timestamp)
    index = min(max(index, 1), len(samples) - 1)
    before, after = samples[index - 1], samples[index]
    ratio = min(1.0, max(0.0, (timestamp - before[0]) / (after[0] - before[0])))
    # Pose samples are 55 Hz and the base attitude changes slowly here; matrix
    # interpolation followed by orthogonalization is enough for this input test.
    def rotation(a, b):
        matrix = (1.0 - ratio) * a + ratio * b
        u, _, vt = np.linalg.svd(matrix)
        return u @ vt
    return (
        (1.0 - ratio) * before[1] + ratio * after[1],
        rotation(before[2], after[2]),
        (1.0 - ratio) * before[3] + ratio * after[3],
        rotation(before[4], after[4]),
    )


def cloud_points(message):
    fields = {field.name: field for field in message.fields}
    required = [fields.get(name) for name in ("x", "y", "z")]
    if any(field is None or field.datatype != 7 or field.count != 1 for field in required):
        raise RuntimeError("PointCloud2 must contain float32 x/y/z fields")
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4", "<f4", "<f4"],
        "offsets": [field.offset for field in required],
        "itemsize": message.point_step,
    })
    raw = np.frombuffer(message.data, dtype=dtype, count=message.width * message.height)
    points = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(float, copy=False)
    finite = np.isfinite(points).all(axis=1)
    ranges = np.linalg.norm(points, axis=1)
    return points[finite & (ranges >= 1.0) & (ranges <= 25.0)]


def read_clouds(bag, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if types.get(topic) != "sensor_msgs/msg/PointCloud2":
        raise RuntimeError(f"missing PointCloud2 topic {topic}")
    message_type = get_message(types[topic])
    clouds = []
    while reader.has_next():
        message_topic, data, bag_time = reader.read_next()
        if message_topic != topic:
            continue
        message = deserialize_message(data, message_type)
        clouds.append((stamp_seconds(message.header.stamp) or bag_time * 1e-9, cloud_points(message)))
    return clouds


def voxel_set(points, leaf):
    return {tuple(row) for row in np.floor(points / leaf).astype(np.int64)}


def compare(reference, candidate, leaf):
    tree = cKDTree(reference)
    distances, _ = tree.query(candidate, k=1, workers=-1)
    first = voxel_set(reference, leaf)
    second = voxel_set(candidate, leaf)
    intersection = len(first & second)
    union = len(first | second)
    return {
        "nn_median_m": float(np.median(distances)),
        "nn_p90_m": float(np.quantile(distances, 0.90)),
        "voxel_jaccard": intersection / union if union else 0.0,
        "reference_voxels": len(first),
        "candidate_voxels": len(second),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("truth_jsonl")
    parser.add_argument("--topic", default="/mid360/livox_points")
    parser.add_argument("--window-s", type=float, default=2.0)
    parser.add_argument("--later-offset-s", type=float, default=10.0)
    parser.add_argument("--voxel-m", type=float, default=0.30)
    args = parser.parse_args()

    truth = load_truth(args.truth_jsonl)
    clouds = read_clouds(args.bag, args.topic)
    if len(clouds) < 2:
        raise RuntimeError("not enough clouds")
    start = clouds[0][0]
    early = [(t, p) for t, p in clouds if start <= t < start + args.window_s]
    later_start = start + args.later_offset_s
    late = [(t, p) for t, p in clouds if later_start <= t < later_start + args.window_s]
    if not early or not late:
        raise RuntimeError("selected cloud windows are empty")

    sensor_offset = np.array([0.0, 0.08637, 0.0])
    sensor_rotation = rpy_matrix(-0.2967059728, 0.0, 0.0)

    def world_cloud(item):
        timestamp, points = item
        p_model, r_model, p_gimbal, r_gimbal = interpolate_truth(truth, timestamp)
        r_sensor = r_model @ r_gimbal @ sensor_rotation
        p_sensor = p_model + r_model @ p_gimbal + r_model @ r_gimbal @ sensor_offset
        return points @ r_sensor.T + p_sensor

    early_raw = np.concatenate([points for _, points in early])
    late_raw = np.concatenate([points for _, points in late])
    early_world = np.concatenate([world_cloud(item) for item in early])
    late_world = np.concatenate([world_cloud(item) for item in late])
    first_pose = interpolate_truth(truth, early[0][0])
    last_pose = interpolate_truth(truth, late[0][0])
    sensor_shift = (last_pose[0] + last_pose[1] @ last_pose[2] @ sensor_offset) - (first_pose[0] + first_pose[1] @ first_pose[2] @ sensor_offset)
    print(json.dumps({
        "clouds": {"early": len(early), "late": len(late), "early_points": len(early_raw), "late_points": len(late_raw)},
        "windows": {"early_start": early[0][0], "late_start": late[0][0], "true_sensor_shift_world_m": sensor_shift.tolist()},
        "raw_sensor_frame": compare(early_raw, late_raw, args.voxel_m),
        "truth_aligned_world": compare(early_world, late_world, args.voxel_m),
        "interpretation": "Truth-aligned world overlap should improve over raw sensor-frame overlap for a static scene.",
    }, indent=2))


if __name__ == "__main__":
    main()
