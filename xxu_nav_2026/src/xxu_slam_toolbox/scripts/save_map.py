#!/usr/bin/env python3
"""Save the current slam_toolbox occupancy map."""

import argparse
import sys
import time
from pathlib import Path

import rclpy
from slam_toolbox.srv import SaveMap


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-path", required=True, help="Output path without .yaml/.pgm suffix")
    parser.add_argument("--service", default="/slam_toolbox/save_map")
    parser.add_argument("--timeout", type=float, default=8.0)
    return parser.parse_args()


def main():
    args = parse_args()
    map_path = Path(args.map_path).expanduser()
    map_path.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node("xxu_save_map")
    try:
        client = node.create_client(SaveMap, args.service)
        deadline = time.monotonic() + args.timeout
        while not client.wait_for_service(timeout_sec=0.25):
            if time.monotonic() >= deadline:
                node.get_logger().error(f"Timed out waiting for {args.service}")
                return 1

        request = SaveMap.Request()
        request.name.data = str(map_path)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)

        if not future.done():
            node.get_logger().error(f"Timed out saving map to {map_path}")
            return 1

        response = future.result()
        if response is None:
            node.get_logger().error("Map save service call failed")
            return 1
        if response.result != SaveMap.Response.RESULT_SUCCESS:
            node.get_logger().error(f"Map save failed with result code {response.result}")
            return 1

        node.get_logger().info(f"Saved map to {map_path}.yaml and {map_path}.pgm")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
