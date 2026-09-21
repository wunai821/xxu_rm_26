"""Collect append-only per-trial safety and navigation metrics."""

from __future__ import annotations

import json
import math
from pathlib import Path
from statistics import mean
from typing import Optional

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, Float32


def _finite(value: float) -> Optional[float]:
    return float(value) if math.isfinite(value) else None


def _percentile(samples: list[float], percentile: float) -> Optional[float]:
    if not samples:
        return None
    ordered = sorted(samples)
    index = (len(ordered) - 1) * percentile / 100.0
    lower, upper = math.floor(index), math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def _summary(samples: list[float]) -> dict[str, Optional[float]]:
    return {
        "mean": mean(samples) if samples else None,
        "p50": _percentile(samples, 50.0),
        "p95": _percentile(samples, 95.0),
        "p99": _percentile(samples, 99.0),
        "max": max(samples) if samples else None,
    }


def _pose_payload(pose: Optional[tuple[float, float]]) -> Optional[dict[str, float]]:
    if pose is None:
        return None
    return {"x": _finite(pose[0]), "y": _finite(pose[1])}


class BenchmarkMetrics(Node):
    """Record a single trial without deleting or filtering any outcome."""

    def __init__(self) -> None:
        super().__init__("flybrain_benchmark_metrics")
        self._declare_parameters()
        self.scenario = str(self.get_parameter("scenario").value)
        self.controller = str(self.get_parameter("controller").value)
        self.seed = int(self.get_parameter("seed").value)
        self.timeout_s = float(self.get_parameter("trial_timeout_s").value)
        self.goal_success_guard_m = float(
            self.get_parameter("goal_success_guard_m").value
        )
        self.response_threshold = float(self.get_parameter("response_delta_threshold").value)
        self.start_s: Optional[float] = None
        self._trial_active = False
        self.completed = False
        self.collision = False
        self.collision_monitor_events = 0
        self.min_distance_m = math.inf
        self.min_ttc_s = math.inf
        self.path_length_m = 0.0
        self._last_pose: Optional[tuple[float, float]] = None
        self._odom_initial_pose: Optional[tuple[float, float]] = None
        self._odom_final_pose: Optional[tuple[float, float]] = None
        self._odom_sample_count = 0
        self._nav_odom_initial_pose: Optional[tuple[float, float]] = None
        self._nav_odom_final_pose: Optional[tuple[float, float]] = None
        self._nav_odom_sample_count = 0
        self._first_threat_s: Optional[float] = None
        self._first_aux_response_s: Optional[float] = None
        self._last_mppi: Optional[TwistStamped] = None
        self._mppi_sample_count = 0
        self._last_aux: Optional[TwistStamped] = None
        self._last_final: Optional[TwistStamped] = None
        self._final_sample_count = 0
        self._last_final_time_s: Optional[float] = None
        self._last_accel: Optional[tuple[float, float, float]] = None
        self.max_linear_accel = 0.0
        self.max_angular_accel = 0.0
        self.max_jerk = 0.0
        self.lambda_samples: list[float] = []
        self.fly_contribution_samples: list[float] = []
        self.brain_latency_ms: list[float] = []
        self.loop_latency_ms: list[float] = []
        self.brain_debug_samples: dict[str, list[float]] = {
            name: [] for name in (
                "turn_score", "front_score", "fired_descending",
                "signal_left", "signal_front", "signal_right",
            )
        }
        self.safety_gate_activation_count = 0
        self.safety_gate_active_time_s = 0.0
        self._safety_active = False
        self._safety_active_since_s: Optional[float] = None
        self._goal_start_distance_m: Optional[float] = None
        self._goal_initial_map_pose: Optional[tuple[float, float]] = None
        self._goal_final_map_pose: Optional[tuple[float, float]] = None
        self._goal_sent = False
        self._goal_handle = None
        self._nav2_active = False
        self._nav2_active_since_s: Optional[float] = None
        self._nav2_state_request_pending = False
        self._goal_rejections = 0
        self._command_samples: dict[str, list[float]] = {
            name: [] for name in (
                "nav", "smoothed", "fused", "collision", "transformed", "chassis",
            )
        }
        self._collision_monitor_action_counts: dict[str, int] = {}
        self._collision_monitor_polygon_counts: dict[str, int] = {}
        self._first_collision_monitor_action_s: Optional[float] = None
        self._watchdog_healthy: Optional[bool] = None
        self._watchdog_last_change_s: Optional[float] = None
        self._watchdog_healthy_time_s = 0.0
        self._watchdog_unhealthy_time_s = 0.0
        self._watchdog_transition_count = 0
        self._trial_start_pub = self.create_publisher(Empty, "/benchmark/trial_start", 10)

        self.create_subscription(Odometry, "/odom", self._odom_callback, 20)
        self.create_subscription(Odometry, "/odom_nav", self._nav_odom_callback, 20)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(Float32, "/flybrain/min_distance", self._min_distance_callback, 20)
        self.create_subscription(Float32, "/flybrain/min_ttc", self._min_ttc_callback, 20)
        self.create_subscription(Float32, "/flybrain/lambda", self._lambda_callback, 20)
        self.create_subscription(Float32, "/flybrain/brain_latency_ms", self._brain_latency_callback, 20)
        self.create_subscription(Float32, "/flybrain/control_loop_ms", self._loop_latency_callback, 20)
        self.create_subscription(TwistStamped, "/cmd_vel_nav", self._command_callback("nav"), 20)
        self.create_subscription(TwistStamped, "/cmd_vel_smoothed", self._mppi_callback, 20)
        self.create_subscription(TwistStamped, "/flybrain/cmd_vel_bias", self._aux_callback, 20)
        self.create_subscription(TwistStamped, "/cmd_vel_fused", self._final_callback, 20)
        self.create_subscription(TwistStamped, "/cmd_vel_collision", self._command_callback("collision"), 20)
        self.create_subscription(TwistStamped, "/cmd_vel_transformed", self._command_callback("transformed"), 20)
        self.create_subscription(TwistStamped, "/cmd_vel", self._command_callback("chassis"), 20)
        self.create_subscription(Bool, "/flybrain/safety_active", self._safety_callback, 20)
        self.create_subscription(Bool, "/cmd_vel_watchdog/healthy", self._watchdog_callback, 20)
        for name in self.brain_debug_samples:
            self.create_subscription(Float32, f"/flybrain/brain_{name}", self._brain_debug_callback(name), 20)
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self._collision_monitor_callback, 20)
        self.create_subscription(Contacts, "/benchmark/contacts", self._contacts_callback, 20)
        self.create_timer(0.1, self._check_timeout)
        self._goal_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._nav2_state_client = self.create_client(GetState, "bt_navigator/get_state")
        self.create_timer(0.5, self._try_dispatch_goal)
        self.get_logger().info(
            f"Recording trial scenario={self.scenario}, controller={self.controller}, seed={self.seed}"
        )

    def _declare_parameters(self) -> None:
        for name, default in {
            "scenario": "static_front",
            "controller": "mppi",
            "seed": 1000,
            "trial_timeout_s": 30.0,
            # This is an independent plausibility bound, not a replacement for
            # Nav2's goal checker.  The benchmark preserves Nav2's own result
            # and rejects only a distant planner-fallback false success.
            "goal_success_guard_m": 0.30,
            "response_delta_threshold": 0.02,
            "nav2_ready_delay_s": 1.0,
            "results_root": "/home/naiwu/fly_brain/gazebo_results",
            "dispatch_goal": False,
            "goal_frame": "map",
            "goal_x": 0.0,
            "goal_y": 0.0,
            "goal_yaw": 0.0,
        }.items():
            self.declare_parameter(name, default)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_callback(self, message: Odometry) -> None:
        if not self._trial_active or self.completed:
            return
        now_s = self._now_s()
        if now_s <= 0.0:
            return
        position = message.pose.pose.position
        current = (position.x, position.y)
        if self._odom_initial_pose is None:
            self._odom_initial_pose = current
        self._odom_final_pose = current
        self._odom_sample_count += 1
        if self._last_pose is not None:
            self.path_length_m += math.dist(self._last_pose, current)
        self._last_pose = current

    def _nav_odom_callback(self, message: Odometry) -> None:
        if not self._trial_active or self.completed:
            return
        position = message.pose.pose.position
        current = (position.x, position.y)
        if self._nav_odom_initial_pose is None:
            self._nav_odom_initial_pose = current
        self._nav_odom_final_pose = current
        self._nav_odom_sample_count += 1

    def _scan_callback(self, message: LaserScan) -> None:
        if not self._trial_active or self.completed:
            return
        values = [float(item) for item in message.ranges if math.isfinite(float(item)) and message.range_min <= item <= message.range_max]
        if values:
            self.min_distance_m = min(self.min_distance_m, min(values))

    def _min_distance_callback(self, message: Float32) -> None:
        if self._trial_active and not self.completed and math.isfinite(message.data):
            self.min_distance_m = min(self.min_distance_m, float(message.data))

    def _min_ttc_callback(self, message: Float32) -> None:
        if self._trial_active and not self.completed and math.isfinite(message.data):
            self.min_ttc_s = min(self.min_ttc_s, float(message.data))
            if message.data <= 1.5 and self._first_threat_s is None:
                self._first_threat_s = self._now_s()

    def _lambda_callback(self, message: Float32) -> None:
        value = float(message.data)
        if self._trial_active and not self.completed and math.isfinite(value):
            self.lambda_samples.append(value)

    def _brain_latency_callback(self, message: Float32) -> None:
        if self._trial_active and not self.completed and math.isfinite(message.data):
            self.brain_latency_ms.append(float(message.data))

    def _loop_latency_callback(self, message: Float32) -> None:
        if self._trial_active and not self.completed and math.isfinite(message.data):
            self.loop_latency_ms.append(float(message.data))

    def _mppi_callback(self, message: TwistStamped) -> None:
        if self._trial_active and not self.completed:
            self._last_mppi = message
            self._mppi_sample_count += 1
            self._record_command("smoothed", message)

    def _command_callback(self, name: str):
        def callback(message: TwistStamped) -> None:
            if self._trial_active and not self.completed:
                self._record_command(name, message)
        return callback

    def _record_command(self, name: str, message: TwistStamped) -> None:
        self._command_samples[name].append(math.sqrt(
            message.twist.linear.x ** 2
            + message.twist.linear.y ** 2
            + message.twist.angular.z ** 2
        ))

    def _aux_callback(self, message: TwistStamped) -> None:
        if not self._trial_active or self.completed:
            return
        self._last_aux = message
        if self._first_threat_s is None or self._first_aux_response_s is not None:
            return
        magnitude = math.sqrt(
            message.twist.linear.x ** 2
            + message.twist.linear.y ** 2
            + message.twist.angular.z ** 2
        )
        if magnitude >= self.response_threshold:
            self._first_aux_response_s = self._now_s()

    def _final_callback(self, message: TwistStamped) -> None:
        if not self._trial_active or self.completed:
            return
        self._record_command("fused", message)
        now_s = self._stamp_or_now(message)
        if self._last_aux is not None and self.lambda_samples:
            auxiliary_norm = math.sqrt(
                self._last_aux.twist.linear.x ** 2
                + self._last_aux.twist.linear.y ** 2
                + self._last_aux.twist.angular.z ** 2
            )
            final_norm = math.sqrt(
                message.twist.linear.x ** 2
                + message.twist.linear.y ** 2
                + message.twist.angular.z ** 2
            )
            if final_norm > 1e-9:
                self.fly_contribution_samples.append(
                    min(1.0, abs(self.lambda_samples[-1]) * auxiliary_norm / final_norm)
                )
        if self._last_final is not None and self._last_final_time_s is not None:
            dt = now_s - self._last_final_time_s
            if 0.001 <= dt <= 1.0:
                accel = (
                    (message.twist.linear.x - self._last_final.twist.linear.x) / dt,
                    (message.twist.linear.y - self._last_final.twist.linear.y) / dt,
                    (message.twist.angular.z - self._last_final.twist.angular.z) / dt,
                )
                self.max_linear_accel = max(self.max_linear_accel, math.hypot(accel[0], accel[1]))
                self.max_angular_accel = max(self.max_angular_accel, abs(accel[2]))
                if self._last_accel is not None:
                    jerk = math.sqrt(sum(((value - previous) / dt) ** 2 for value, previous in zip(accel, self._last_accel)))
                    self.max_jerk = max(self.max_jerk, jerk)
                self._last_accel = accel
        self._last_final = message
        self._final_sample_count += 1
        self._last_final_time_s = now_s

    def _safety_callback(self, message: Bool) -> None:
        if not self._trial_active or self.completed:
            return
        now_s = self._now_s()
        if message.data and not self._safety_active:
            self._safety_active = True
            self._safety_active_since_s = now_s
            self.safety_gate_activation_count += 1
        elif not message.data and self._safety_active:
            if self._safety_active_since_s is not None:
                self.safety_gate_active_time_s += max(0.0, now_s - self._safety_active_since_s)
            self._safety_active = False
            self._safety_active_since_s = None

    def _brain_debug_callback(self, name: str):
        def callback(message: Float32) -> None:
            if self._trial_active and not self.completed and math.isfinite(message.data):
                self.brain_debug_samples[name].append(float(message.data))
        return callback

    def _collision_monitor_callback(self, message: CollisionMonitorState) -> None:
        if self._trial_active and not self.completed and message.action_type != CollisionMonitorState.DO_NOTHING:
            self.collision_monitor_events += 1
            action = {
                CollisionMonitorState.STOP: "stop",
                CollisionMonitorState.SLOWDOWN: "slowdown",
                CollisionMonitorState.APPROACH: "approach",
                CollisionMonitorState.LIMIT: "limit",
            }.get(message.action_type, f"unknown_{message.action_type}")
            self._collision_monitor_action_counts[action] = (
                self._collision_monitor_action_counts.get(action, 0) + 1
            )
            polygon = message.polygon_name or "<unnamed>"
            self._collision_monitor_polygon_counts[polygon] = (
                self._collision_monitor_polygon_counts.get(polygon, 0) + 1
            )
            if self._first_collision_monitor_action_s is None:
                self._first_collision_monitor_action_s = self._now_s()

    def _watchdog_callback(self, message: Bool) -> None:
        if not self._trial_active or self.completed:
            return
        now_s = self._now_s()
        healthy = bool(message.data)
        if self._watchdog_healthy is None:
            self._watchdog_healthy = healthy
            self._watchdog_last_change_s = now_s
            return
        if healthy == self._watchdog_healthy:
            return
        if self._watchdog_last_change_s is not None:
            elapsed = max(0.0, now_s - self._watchdog_last_change_s)
            if self._watchdog_healthy:
                self._watchdog_healthy_time_s += elapsed
            else:
                self._watchdog_unhealthy_time_s += elapsed
        self._watchdog_healthy = healthy
        self._watchdog_last_change_s = now_s
        self._watchdog_transition_count += 1

    def _contacts_callback(self, message: Contacts) -> None:
        if self._trial_active and not self.completed and message.contacts:
            self.collision = True

    def _check_timeout(self) -> None:
        if self.completed or not self._trial_active or self.start_s is None:
            return
        if self._now_s() - self.start_s >= self.timeout_s:
            self._finalize(success=False, finish_reason="timeout")

    def _try_dispatch_goal(self) -> None:
        if self.completed or self._goal_sent or not bool(self.get_parameter("dispatch_goal").value):
            return
        if not self._nav2_active:
            self._request_nav2_state()
            return
        if self._nav2_active_since_s is None:
            return
        if self._now_s() - self._nav2_active_since_s < float(self.get_parameter("nav2_ready_delay_s").value):
            return
        if not self._goal_client.wait_for_server(timeout_sec=0.0):
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = str(self.get_parameter("goal_frame").value)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.get_parameter("goal_x").value)
        goal.pose.pose.position.y = float(self.get_parameter("goal_y").value)
        yaw = float(self.get_parameter("goal_yaw").value)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._goal_sent = True
        self._goal_client.send_goal_async(goal, feedback_callback=self._goal_feedback).add_done_callback(self._goal_accepted)

    def _request_nav2_state(self) -> None:
        if self._nav2_state_request_pending:
            return
        if not self._nav2_state_client.wait_for_service(timeout_sec=0.0):
            return
        self._nav2_state_request_pending = True
        self._nav2_state_client.call_async(GetState.Request()).add_done_callback(self._nav2_state_result)

    def _nav2_state_result(self, future) -> None:
        self._nav2_state_request_pending = False
        try:
            response = future.result()
            self._nav2_active = response.current_state.id == State.PRIMARY_STATE_ACTIVE
            if self._nav2_active and self._nav2_active_since_s is None:
                self._nav2_active_since_s = self._now_s()
        except Exception:
            self._nav2_active = False

    def _goal_accepted(self, future) -> None:
        try:
            self._goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"NavigateToPose request failed: {exc}")
            self._finalize(success=False, finish_reason="goal_request_error")
            return
        if not self._goal_handle.accepted:
            self._goal_sent = False
            self._nav2_active = False
            self._nav2_active_since_s = None
            self._goal_rejections += 1
            if self._goal_rejections >= 3:
                self._finalize(success=False, finish_reason="goal_rejected")
            return
        # Trial timing begins when Nav2 accepts the exact shared goal.
        self.start_s = self._now_s()
        self._trial_active = True
        self.path_length_m = 0.0
        self._last_pose = None
        self._trial_start_pub.publish(Empty())
        self._goal_handle.get_result_async().add_done_callback(self._goal_result)

    def _goal_feedback(self, feedback_message) -> None:
        if not self._trial_active:
            return
        current = feedback_message.feedback.current_pose
        if current.header.frame_id != str(self.get_parameter("goal_frame").value):
            return
        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)
        pose = (current.pose.position.x, current.pose.position.y)
        if self._goal_initial_map_pose is None:
            self._goal_initial_map_pose = pose
            self._goal_start_distance_m = math.dist(pose, (goal_x, goal_y))
        self._goal_final_map_pose = pose

    def _goal_result(self, future) -> None:
        try:
            result = future.result()
            success = result.status == GoalStatus.STATUS_SUCCEEDED
            self._finalize(success=success, finish_reason="goal_succeeded" if success else f"goal_status_{result.status}")
        except Exception as exc:
            self.get_logger().error(f"NavigateToPose result failed: {exc}")
            self._finalize(success=False, finish_reason="goal_result_error")

    def _finalize(self, *, success: bool, finish_reason: str) -> None:
        if self.completed:
            return
        now_s = self._now_s()
        if self._safety_active and self._safety_active_since_s is not None:
            self.safety_gate_active_time_s += max(0.0, now_s - self._safety_active_since_s)
            self._safety_active = False
            self._safety_active_since_s = None
        if self._watchdog_healthy is not None and self._watchdog_last_change_s is not None:
            watchdog_elapsed = max(0.0, now_s - self._watchdog_last_change_s)
            if self._watchdog_healthy:
                self._watchdog_healthy_time_s += watchdog_elapsed
            else:
                self._watchdog_unhealthy_time_s += watchdog_elapsed
            self._watchdog_last_change_s = now_s
        self.completed = True
        elapsed = now_s - self.start_s if self.start_s is not None else None
        detection_latency = self._first_threat_s - self.start_s if self.start_s is not None and self._first_threat_s is not None else None
        response_latency = (
            self._first_aux_response_s - self._first_threat_s
            if self._first_aux_response_s is not None and self._first_threat_s is not None
            else None
        )
        failure_reason = None
        valid_trial = self._trial_active and finish_reason not in {
            "goal_request_error", "goal_rejected", "goal_result_error",
        }
        if valid_trial and success and self._goal_start_distance_m is not None and self._goal_start_distance_m > 0.5:
            if self._mppi_sample_count == 0 or self._final_sample_count == 0:
                valid_trial = False
                failure_reason = "no_control_telemetry_before_goal_success"
            elif elapsed is not None and elapsed < 0.5:
                valid_trial = False
                failure_reason = "implausibly_fast_goal_success"
        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)
        final_goal_distance = (
            math.dist(self._goal_final_map_pose, (goal_x, goal_y))
            if self._goal_final_map_pose is not None
            else None
        )
        if success and (
            final_goal_distance is None
            or final_goal_distance > self.goal_success_guard_m
        ):
            valid_trial = False
            failure_reason = "goal_success_position_not_verified"
        path_efficiency = (
            self.path_length_m / self._goal_start_distance_m
            if self._goal_start_distance_m is not None and self._goal_start_distance_m > 1e-9
            else None
        )
        lambda_positive = [value for value in self.lambda_samples if value > 0.0]
        lambda_half = [value for value in self.lambda_samples if value > 0.5]
        lambda_full = [value for value in self.lambda_samples if value >= 1.0 - 1e-9]
        brain_stats = _summary(self.brain_latency_ms)
        loop_stats = _summary(self.loop_latency_ms)
        turn_samples = self.brain_debug_samples["turn_score"]
        left_scores = [max(0.0, value) for value in turn_samples]
        right_scores = [max(0.0, -value) for value in turn_samples]
        command_chain = {}
        for name, samples in self._command_samples.items():
            stats = _summary(samples)
            command_chain[name] = {
                "sample_count": len(samples),
                "nonzero_fraction": (
                    sum(value > 1e-6 for value in samples) / len(samples)
                    if samples else 0.0
                ),
                "mean_norm": stats["mean"],
                "max_norm": stats["max"],
            }
        odom_displacement = (
            math.dist(self._odom_initial_pose, self._odom_final_pose)
            if self._odom_initial_pose is not None and self._odom_final_pose is not None
            else None
        )
        nav_odom_displacement = (
            math.dist(self._nav_odom_initial_pose, self._nav_odom_final_pose)
            if self._nav_odom_initial_pose is not None and self._nav_odom_final_pose is not None
            else None
        )
        payload = {
            "scenario": self.scenario,
            "controller": self.controller,
            "seed": self.seed,
            "valid_trial": valid_trial,
            "failure_reason": None if valid_trial else (failure_reason or finish_reason),
            "success": bool(success),
            "goal_success": bool(success),
            "finish_reason": finish_reason,
            "timeout": finish_reason == "timeout",
            "simulation_start_s": _finite(self.start_s) if self.start_s is not None else None,
            "simulation_end_s": _finite(now_s),
            "collision": self.collision,
            "collision_monitor_events": self.collision_monitor_events,
            "min_distance_m": _finite(self.min_distance_m),
            "min_ttc_s": _finite(self.min_ttc_s),
            "first_threat_time_s": _finite(detection_latency) if detection_latency is not None else None,
            "first_aux_response_time_s": (
                _finite(self._first_aux_response_s - self.start_s)
                if self._first_aux_response_s is not None and self.start_s is not None
                else None
            ),
            "response_latency_ms": _finite(response_latency * 1000.0) if response_latency is not None else None,
            "threat_detect_latency_ms": _finite(detection_latency * 1000.0) if detection_latency is not None else None,
            "control_response_latency_ms": _finite(response_latency * 1000.0) if response_latency is not None else None,
            "time_to_goal_s": _finite(elapsed) if elapsed is not None else None,
            "path_length_m": self.path_length_m,
            "straight_line_start_goal_m": self._goal_start_distance_m,
            "goal_initial_map_pose": (
                {"x": self._goal_initial_map_pose[0], "y": self._goal_initial_map_pose[1]}
                if self._goal_initial_map_pose is not None else None
            ),
            "goal_final_map_pose": (
                {"x": self._goal_final_map_pose[0], "y": self._goal_final_map_pose[1]}
                if self._goal_final_map_pose is not None else None
            ),
            "goal_final_distance_m": final_goal_distance,
            "goal_success_guard_m": self.goal_success_guard_m,
            "path_efficiency_ratio": path_efficiency,
            "mppi_command_samples": self._mppi_sample_count,
            "final_command_samples": self._final_sample_count,
            "odom_initial_pose": _pose_payload(self._odom_initial_pose),
            "odom_final_pose": _pose_payload(self._odom_final_pose),
            "odom_displacement_m": odom_displacement,
            "odom_sample_count": self._odom_sample_count,
            "nav_odom_initial_pose": _pose_payload(self._nav_odom_initial_pose),
            "nav_odom_final_pose": _pose_payload(self._nav_odom_final_pose),
            "nav_odom_displacement_m": nav_odom_displacement,
            "nav_odom_sample_count": self._nav_odom_sample_count,
            "command_chain": command_chain,
            "max_linear_accel": self.max_linear_accel,
            "max_angular_accel": self.max_angular_accel,
            "max_jerk": self.max_jerk,
            "safety_gate_activation_count": self.safety_gate_activation_count,
            "safety_gate_active_time_s": self.safety_gate_active_time_s,
            "collision_monitor_action_counts": self._collision_monitor_action_counts,
            "collision_monitor_polygon_counts": self._collision_monitor_polygon_counts,
            "first_collision_monitor_action_time_s": (
                _finite(self._first_collision_monitor_action_s - self.start_s)
                if self._first_collision_monitor_action_s is not None and self.start_s is not None
                else None
            ),
            "watchdog_healthy_time_s": self._watchdog_healthy_time_s,
            "watchdog_unhealthy_time_s": self._watchdog_unhealthy_time_s,
            "watchdog_transition_count": self._watchdog_transition_count,
            "lambda_mean": mean(self.lambda_samples) if self.lambda_samples else 0.0,
            "lambda_max": max(self.lambda_samples) if self.lambda_samples else 0.0,
            "lambda_positive_fraction": len(lambda_positive) / len(self.lambda_samples) if self.lambda_samples else 0.0,
            "lambda_gt_0_5_fraction": len(lambda_half) / len(self.lambda_samples) if self.lambda_samples else 0.0,
            "lambda_eq_1_fraction": len(lambda_full) / len(self.lambda_samples) if self.lambda_samples else 0.0,
            "fly_contribution_ratio_mean": mean(self.fly_contribution_samples) if self.fly_contribution_samples else 0.0,
            "brain_mean_ms": brain_stats["mean"],
            "brain_p50_ms": brain_stats["p50"],
            "brain_p95_ms": brain_stats["p95"],
            "brain_p99_ms": brain_stats["p99"],
            "brain_max_ms": brain_stats["max"],
            "controller_loop_mean_ms": loop_stats["mean"],
            "controller_loop_p50_ms": loop_stats["p50"],
            "controller_loop_p95_ms": loop_stats["p95"],
            "controller_loop_p99_ms": loop_stats["p99"],
            "controller_loop_max_ms": loop_stats["max"],
            # V1 exposes only a signed turn readout.  The two directional
            # score fields are its positive/negative components; LC4/LPLC2
            # fields are encoder drive, not unexposed post-step spike counts.
            "left_score_mean": mean(left_scores) if left_scores else None,
            "front_score_mean": mean(self.brain_debug_samples["front_score"]) if self.brain_debug_samples["front_score"] else None,
            "right_score_mean": mean(right_scores) if right_scores else None,
            "fired_descending_mean": mean(self.brain_debug_samples["fired_descending"]) if self.brain_debug_samples["fired_descending"] else None,
            "lc4_l_drive_mean": mean(self.brain_debug_samples["signal_left"]) if self.brain_debug_samples["signal_left"] else None,
            "lc4_r_drive_mean": mean(self.brain_debug_samples["signal_right"]) if self.brain_debug_samples["signal_right"] else None,
            "lplc2_l_drive_mean": mean(self.brain_debug_samples["signal_left"]) if self.brain_debug_samples["signal_left"] else None,
            "lplc2_r_drive_mean": mean(self.brain_debug_samples["signal_right"]) if self.brain_debug_samples["signal_right"] else None,
            "cpu_utilization": None,
            "gpu_utilization": None,
        }
        root = Path(str(self.get_parameter("results_root").value)).expanduser()
        output_dir = root / "raw" / self.scenario / self.controller
        output_dir.mkdir(parents=True, exist_ok=True)
        output = output_dir / f"seed_{self.seed}.json"
        if output.exists():
            raise FileExistsError(f"refusing to overwrite trial result: {output}")
        output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.get_logger().info(f"Trial complete: {output}")

    @staticmethod
    def _command_delta(first: TwistStamped, second: TwistStamped) -> float:
        return math.sqrt(
            (first.twist.linear.x - second.twist.linear.x) ** 2
            + (first.twist.linear.y - second.twist.linear.y) ** 2
            + (first.twist.angular.z - second.twist.angular.z) ** 2
        )

    def _stamp_or_now(self, message: TwistStamped) -> float:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        return stamp if stamp > 0.0 else self._now_s()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BenchmarkMetrics()
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
