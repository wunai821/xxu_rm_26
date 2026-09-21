"""Deterministically spawn and move one Gazebo benchmark obstacle per trial."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import random
from typing import Any, Optional

from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose, SpawnEntity
from std_msgs.msg import Empty, String
import yaml


@dataclass(frozen=True)
class Keyframe:
    time_s: float
    x: float
    y: float
    z: float
    yaw: float


class ScenarioManager(Node):
    """One reproducible scenario timeline driven by simulation time.

    This node owns only its newly spawned obstacle. It never commands the XXU
    robot, changes Nav2 parameters, or alters the base world.
    """

    def __init__(self) -> None:
        super().__init__("flybrain_scenario_manager")
        self.declare_parameter("world_name", "complex_mapping")
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("scenario", "static_front")
        self.declare_parameter("seed", 1000)
        self.declare_parameter("obstacle_name", "flybrain_obstacle")
        self.declare_parameter("obstacle_sdf", "")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("autostart", False)
        self.declare_parameter("start_topic", "/benchmark/trial_start")

        self.world_name = str(self.get_parameter("world_name").value)
        self.scenario_name = str(self.get_parameter("scenario").value)
        self.seed = int(self.get_parameter("seed").value)
        self.obstacle_name = str(self.get_parameter("obstacle_name").value)
        self.obstacle_sdf = str(self.get_parameter("obstacle_sdf").value)
        self._scenario = self._load_scenario(str(self.get_parameter("scenario_file").value))
        self._keyframes, self._duration_s = self._prepare_keyframes(self._scenario)
        self._spawn_client = self.create_client(SpawnEntity, f"/world/{self.world_name}/create")
        self._pose_client = self.create_client(SetEntityPose, f"/world/{self.world_name}/set_pose")
        self._state_pub = self.create_publisher(String, "/benchmark/scenario_state", 10)
        self.create_subscription(
            Empty,
            str(self.get_parameter("start_topic").value),
            self._start_callback,
            10,
        )
        self._spawn_requested = False
        self._spawned = False
        # Spawn before navigation dispatch, but keep the trajectory frozen
        # until the paired NavigateToPose request has been accepted.
        self._start_requested = bool(self.get_parameter("autostart").value)
        self._started = False
        self._completed = False
        self._start_time_s: Optional[float] = None
        self._pose_pending = False
        self._last_log_state = ""
        rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        if self._start_requested:
            self._publish_state("waiting_for_gazebo")

    def _load_scenario(self, scenario_file: str) -> dict[str, Any]:
        path = Path(scenario_file)
        if not scenario_file or not path.is_file():
            raise FileNotFoundError("scenario_file must point to benchmark_scenarios.yaml")
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        scenarios = document.get("scenarios", {}) if isinstance(document, dict) else {}
        scenario = scenarios.get(self.scenario_name)
        if not isinstance(scenario, dict):
            raise ValueError(f"unknown benchmark scenario: {self.scenario_name}")
        return scenario

    def _prepare_keyframes(self, scenario: dict[str, Any]) -> tuple[list[Keyframe], float]:
        raw_keyframes = scenario.get("keyframes")
        if not isinstance(raw_keyframes, list) or len(raw_keyframes) < 2:
            raise ValueError("a scenario needs at least two keyframes")
        rng = random.Random(self.seed)
        jitter_limit = max(0.0, float(scenario.get("position_jitter_m", 0.0)))
        jitter_x = rng.uniform(-jitter_limit, jitter_limit)
        jitter_y = rng.uniform(-jitter_limit, jitter_limit)
        keyframes = [
            Keyframe(
                time_s=float(item["t"]),
                x=float(item["x"]) + jitter_x,
                y=float(item["y"]) + jitter_y,
                z=float(item.get("z", 0.4)),
                yaw=float(item.get("yaw", 0.0)),
            )
            for item in raw_keyframes
        ]
        keyframes.sort(key=lambda frame: frame.time_s)
        if any(second.time_s <= first.time_s for first, second in zip(keyframes, keyframes[1:])):
            raise ValueError("scenario keyframe times must strictly increase")
        duration_s = max(keyframes[-1].time_s, float(scenario.get("duration_s", keyframes[-1].time_s)))
        return keyframes, duration_s

    def _tick(self) -> None:
        if self._completed:
            return
        if not self._spawned:
            self._request_spawn()
            return
        if not self._start_requested:
            return
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s <= 0.0:
            return
        if not self._started:
            self._started = True
            self._start_time_s = now_s
            self._publish_state("running")
        elapsed_s = max(0.0, now_s - (self._start_time_s or now_s))
        self._set_pose(self._interpolate(min(elapsed_s, self._duration_s)))
        if elapsed_s >= self._duration_s:
            self._completed = True
            self._publish_state("complete")

    def _start_callback(self, _message: Empty) -> None:
        if self._completed or self._start_requested:
            return
        self._start_requested = True
        self._publish_state("start_requested")

    def _request_spawn(self) -> None:
        if self._spawn_requested:
            return
        if not self.obstacle_sdf:
            self._publish_state("error:missing_obstacle_sdf")
            self._completed = True
            return
        if not self._spawn_client.wait_for_service(timeout_sec=0.0):
            self._publish_state("waiting_for_create_service")
            return
        request = SpawnEntity.Request()
        request.entity_factory.name = self.obstacle_name
        request.entity_factory.allow_renaming = False
        request.entity_factory.sdf_filename = self.obstacle_sdf
        request.entity_factory.pose = self._to_pose(self._keyframes[0])
        request.entity_factory.relative_to = "world"
        self._spawn_requested = True
        future = self._spawn_client.call_async(request)
        future.add_done_callback(self._spawn_done)

    def _spawn_done(self, future) -> None:
        try:
            response = future.result()
            self._spawned = bool(response.success)
            if not self._spawned:
                self._completed = True
                self._publish_state("error:spawn_failed")
            else:
                self._publish_state("spawned")
        except Exception as exc:
            self._completed = True
            self._publish_state(f"error:spawn_exception:{type(exc).__name__}")

    def _set_pose(self, frame: Keyframe) -> None:
        if self._pose_pending or not self._pose_client.wait_for_service(timeout_sec=0.0):
            return
        request = SetEntityPose.Request()
        request.entity = Entity(name=self.obstacle_name, type=Entity.MODEL)
        request.pose = self._to_pose(frame)
        self._pose_pending = True
        future = self._pose_client.call_async(request)
        future.add_done_callback(self._pose_done)

    def _pose_done(self, future) -> None:
        self._pose_pending = False
        try:
            if not future.result().success:
                self._publish_state("error:set_pose_failed")
        except Exception as exc:
            self._publish_state(f"error:set_pose_exception:{type(exc).__name__}")

    def _interpolate(self, elapsed_s: float) -> Keyframe:
        for first, second in zip(self._keyframes, self._keyframes[1:]):
            if elapsed_s <= second.time_s:
                ratio = (elapsed_s - first.time_s) / (second.time_s - first.time_s)
                ratio = min(1.0, max(0.0, ratio))
                return Keyframe(
                    elapsed_s,
                    first.x + (second.x - first.x) * ratio,
                    first.y + (second.y - first.y) * ratio,
                    first.z + (second.z - first.z) * ratio,
                    first.yaw + (second.yaw - first.yaw) * ratio,
                )
        return self._keyframes[-1]

    @staticmethod
    def _to_pose(frame: Keyframe) -> Pose:
        pose = Pose()
        pose.position.x = frame.x
        pose.position.y = frame.y
        pose.position.z = frame.z
        pose.orientation.z = math.sin(frame.yaw / 2.0)
        pose.orientation.w = math.cos(frame.yaw / 2.0)
        return pose

    def _publish_state(self, state: str) -> None:
        if state == self._last_log_state:
            return
        self._last_log_state = state
        message = String()
        message.data = json.dumps({
            "scenario": self.scenario_name,
            "seed": self.seed,
            "state": state,
        }, sort_keys=True)
        self._state_pub.publish(message)
        self.get_logger().info(message.data)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioManager()
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
