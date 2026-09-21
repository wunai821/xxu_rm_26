"""ROS 2 bridge node for shadow-mode validation and explicit fused operation."""

from __future__ import annotations

import math
import json
from pathlib import Path
import time
from typing import Dict, Mapping, Optional

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32

from .benchmark_logger import BenchmarkLogger
from .controller_fusion import HeuristicEscapePolicy, TwistCommand, fuse_command, smoothstep_lambda
from .flybrain_adapter import FlyBrainAdapter
from .safety_gate import SafetyGate
from .threat_extractor import SectorThreat, ThreatExtractor, most_urgent_threat


class FlyBrainNode(Node):
    """Keep auxiliary navigation evidence separate from the base safety chain."""

    VALID_GROUPS = {"mppi", "heuristic", "flybrain", "rewired"}
    VALID_MODES = {"shadow", "active"}

    def __init__(self) -> None:
        super().__init__("flybrain_node")
        self._declare_parameters()
        self.mode = str(self.get_parameter("mode").value).lower()
        self.control_group = str(self.get_parameter("control_group").value).lower()
        if self.mode not in self.VALID_MODES:
            raise ValueError(f"mode must be one of {sorted(self.VALID_MODES)}")
        if self.control_group not in self.VALID_GROUPS:
            raise ValueError(f"control_group must be one of {sorted(self.VALID_GROUPS)}")

        self.scan_timeout_s = float(self.get_parameter("scan_timeout_s").value)
        self.expected_scan_frame = str(self.get_parameter("expected_scan_frame").value)
        if not self.expected_scan_frame:
            raise ValueError("expected_scan_frame must be set for velocity fusion")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._command_frame = None
        self._last_scan_receipt_s: Optional[float] = None
        self._scan_frame_warned = False
        self._threats: Dict[str, SectorThreat] = {
            name: SectorThreat(name=name) for name in ThreatExtractor.SECTORS
        }
        self._robot_speed_mps = 0.0
        self._last_reason = ""
        self._brain_debug: Mapping[str, object] = {}

        self.extractor = ThreatExtractor(
            front_angle_deg=float(self.get_parameter("threat.front_angle_deg").value),
            side_angle_deg=float(self.get_parameter("threat.side_angle_deg").value),
            min_valid_points=int(self.get_parameter("threat.min_valid_points").value),
            distance_percentile=float(self.get_parameter("threat.distance_percentile").value),
            distance_filter_alpha=float(self.get_parameter("threat.distance_filter_alpha").value),
            closing_filter_alpha=float(self.get_parameter("threat.closing_filter_alpha").value),
            min_closing_speed_mps=float(self.get_parameter("threat.min_closing_speed_mps").value),
            max_closing_speed_mps=float(self.get_parameter("threat.max_closing_speed_mps").value),
        )
        self.heuristic = HeuristicEscapePolicy(
            max_vx=float(self.get_parameter("fusion.max_aux_vx").value),
            max_vy=float(self.get_parameter("fusion.max_aux_vy").value),
            max_wz=float(self.get_parameter("fusion.max_aux_wz").value),
            front_reverse_vx=float(self.get_parameter("heuristic.front_reverse_vx").value),
            front_lateral_vy=float(self.get_parameter("heuristic.front_lateral_vy").value),
        )
        self.safety_gate = SafetyGate(
            max_vx=float(self.get_parameter("safety.max_vx").value),
            max_vy=float(self.get_parameter("safety.max_vy").value),
            max_wz=float(self.get_parameter("safety.max_wz").value),
            max_ax=float(self.get_parameter("safety.max_ax").value),
            max_ay=float(self.get_parameter("safety.max_ay").value),
            max_awz=float(self.get_parameter("safety.max_awz").value),
            max_jx=float(self.get_parameter("safety.max_jx").value),
            max_jy=float(self.get_parameter("safety.max_jy").value),
            max_jwz=float(self.get_parameter("safety.max_jwz").value),
            emergency_distance_m=float(self.get_parameter("threat.distance_emergency_m").value),
        )
        brain_source_dir = str(self.get_parameter("brain.source_dir").value)
        real_data_dir = str(self.get_parameter("brain.real_data_dir").value)
        rewired_data_dir = ""
        brain_data_dir = real_data_dir
        if self.control_group == "rewired":
            rewired_data_dir = str(self.get_parameter("brain.rewired_data_dir").value)
            brain_data_dir = rewired_data_dir
        self.flybrain = FlyBrainAdapter(
            source_dir=brain_source_dir,
            data_dir=brain_data_dir,
            require_data_dir=self.control_group in {"flybrain", "rewired"},
            module_name=str(self.get_parameter("brain.module_name").value),
            class_name=str(self.get_parameter("brain.class_name").value),
            device=str(self.get_parameter("brain.device").value),
            nominal_speed=float(self.get_parameter("brain.nominal_speed").value),
            ros_vy_sign=float(self.get_parameter("brain.ros_vy_sign").value),
            ros_wz_sign=float(self.get_parameter("brain.ros_wz_sign").value),
        ) if self.control_group in {"flybrain", "rewired"} else None
        self.logger = BenchmarkLogger(
            str(self.get_parameter("logging.root_dir").value),
            bool(self.get_parameter("logging.enabled").value),
        )
        self._asset_metadata = self._load_asset_metadata(
            brain_source_dir,
            brain_data_dir,
            self.control_group == "rewired",
            str(self.get_parameter("brain.expected_brain_sha256").value),
        )
        self.logger.record({
            "event": "bridge_started",
            "controller": self.control_group,
            "scenario": str(self.get_parameter("benchmark.scenario").value),
            "seed": int(self.get_parameter("benchmark.seed").value),
            "brain_device": str(self.get_parameter("brain.device").value),
            "brain_asset": self._asset_metadata,
        })

        input_topic = str(self.get_parameter("input_cmd_topic").value)
        self.create_subscription(TwistStamped, input_topic, self._mppi_callback, 20)
        self.create_subscription(LaserScan, str(self.get_parameter("scan_topic").value), self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._odom_callback, 20)
        self.bias_pub = self.create_publisher(TwistStamped, "/flybrain/cmd_vel_bias", 20)
        self.fused_debug_pub = self.create_publisher(TwistStamped, "/flybrain/cmd_vel_fused", 20)
        self.lambda_pub = self.create_publisher(Float32, "/flybrain/lambda", 20)
        self.min_ttc_pub = self.create_publisher(Float32, "/flybrain/min_ttc", 20)
        self.min_distance_pub = self.create_publisher(Float32, "/flybrain/min_distance", 20)
        self.brain_latency_pub = self.create_publisher(Float32, "/flybrain/brain_latency_ms", 20)
        self.loop_latency_pub = self.create_publisher(Float32, "/flybrain/control_loop_ms", 20)
        self.safety_active_pub = self.create_publisher(Bool, "/flybrain/safety_active", 20)
        self.brain_debug_pubs = {
            name: self.create_publisher(Float32, f"/flybrain/{name}", 20)
            for name in (
                "brain_turn_score",
                "brain_front_score",
                "brain_fired_descending",
                "brain_signal_left",
                "brain_signal_front",
                "brain_signal_right",
            )
        }
        self.threat_pubs = {
            name: self.create_publisher(Float32, f"/flybrain/threat_{name}", 20)
            for name in ThreatExtractor.SECTORS
        }
        self.output_pub = None
        if self.mode == "active":
            output_topic = str(self.get_parameter("output_cmd_topic").value)
            if output_topic == input_topic:
                raise ValueError("active mode requires distinct input_cmd_topic and output_cmd_topic")
            self.output_pub = self.create_publisher(TwistStamped, output_topic, 20)

        state = "available" if self.flybrain and self.flybrain.available else "not requested"
        if self.flybrain and not self.flybrain.available:
            state = f"unavailable ({self.flybrain.error})"
        self.get_logger().info(
            f"FlyBrain bridge started in {self.mode} mode, group={self.control_group}, "
            f"input={input_topic}, brain={state}."
        )

    def destroy_node(self) -> bool:
        self.logger.close()
        return super().destroy_node()

    def _declare_parameters(self) -> None:
        parameters = {
            "mode": "shadow",
            "control_group": "flybrain",
            "input_cmd_topic": "/cmd_vel_nav",
            "output_cmd_topic": "/cmd_vel_nav",
            "scan_topic": "/scan",
            "odom_topic": "/odom",
            "expected_scan_frame": "base_footprint",
            "scan_timeout_s": 0.5,
            "threat.front_angle_deg": 20.0,
            "threat.side_angle_deg": 90.0,
            "threat.min_valid_points": 3,
            "threat.distance_percentile": 0.10,
            "threat.distance_filter_alpha": 0.45,
            "threat.closing_filter_alpha": 0.35,
            "threat.min_closing_speed_mps": 0.03,
            "threat.max_closing_speed_mps": 8.0,
            "threat.ttc_safe_s": 1.5,
            "threat.ttc_full_escape_s": 0.5,
            "threat.distance_emergency_m": 0.25,
            "fusion.max_aux_vx": 0.5,
            "fusion.max_aux_vy": 0.7,
            "fusion.max_aux_wz": 0.8,
            "heuristic.front_reverse_vx": 0.5,
            "heuristic.front_lateral_vy": 0.5,
            "safety.max_vx": 1.5,
            "safety.max_vy": 1.5,
            "safety.max_wz": 0.8,
            "safety.max_ax": 0.6,
            "safety.max_ay": 0.4,
            "safety.max_awz": 1.2,
            "safety.max_jx": 4.0,
            "safety.max_jy": 4.0,
            "safety.max_jwz": 8.0,
            "brain.source_dir": "/home/naiwu/fly_brain",
            # Group D must point to a separately generated asset. An empty
            # value disables D instead of silently measuring real MaleCNS.
            "brain.rewired_data_dir": "",
            "brain.real_data_dir": "/home/naiwu/fly-data",
            "brain.expected_brain_sha256": "",
            "brain.module_name": "controller",
            "brain.class_name": "FlyBrainController",
            "brain.device": "auto",
            # A bridge auxiliary is a threat bias, not a second navigator.
            # Keep V1's normal cruise component out of Group C/D.
            "brain.nominal_speed": 0.0,
            # V1 is right-positive; ROS base coordinates are left-positive.
            "brain.ros_vy_sign": -1.0,
            "brain.ros_wz_sign": -1.0,
            "logging.enabled": False,
            "logging.root_dir": "/home/naiwu/fly_brain/gazebo_results",
            "benchmark.scenario": "",
            "benchmark.seed": -1,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)

    def _scan_callback(self, message: LaserScan) -> None:
        receipt_s = time.monotonic()
        header_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        stamp_s = header_stamp if header_stamp > 0.0 else receipt_s
        if message.header.frame_id != self.expected_scan_frame:
            if not self._scan_frame_warned:
                self.get_logger().warning(
                    f"Scan frame is '{message.header.frame_id}', expected '{self.expected_scan_frame}'. "
                    "Threat sectors require a base-aligned scan frame."
                )
                self._scan_frame_warned = True
            self._last_scan_receipt_s = None
            return
        self._threats = self.extractor.process_scan(
            message.ranges,
            message.angle_min,
            message.angle_increment,
            message.range_min,
            message.range_max,
            stamp_s,
        )
        self._last_scan_receipt_s = receipt_s
        for name, threat in self._threats.items():
            debug = Float32()
            debug.data = float(threat.distance_m if threat.valid else math.inf)
            self.threat_pubs[name].publish(debug)

    def _odom_callback(self, message: Odometry) -> None:
        linear = message.twist.twist.linear
        self._robot_speed_mps = math.hypot(linear.x, linear.y)

    def _mppi_callback(self, message: TwistStamped) -> None:
        started_s = time.perf_counter()
        try:
            front_yaw = self._front_yaw(message)
        except (TransformException, ValueError) as exc:
            self.safety_gate.reset()
            self._publish_command(self.fused_debug_pub, TwistCommand(), message)
            if self.output_pub is not None:
                self._publish_command(self.output_pub, TwistCommand(), message)
            self.get_logger().warning(f"Stopping fusion: {exc}", throttle_duration_sec=2.0)
            return
        if self._command_frame != message.header.frame_id:
            self.safety_gate.reset()
            self._command_frame = message.header.frame_id
        mppi = TwistCommand(
            message.twist.linear.x,
            message.twist.linear.y,
            message.twist.angular.z,
        )
        scan_fresh = self._last_scan_receipt_s is not None and (time.monotonic() - self._last_scan_receipt_s) <= self.scan_timeout_s
        urgent = most_urgent_threat(self._threats)
        ttc_s = urgent.ttc_s if urgent is not None else math.inf
        blend = smoothstep_lambda(
            ttc_s,
            ttc_safe_s=float(self.get_parameter("threat.ttc_safe_s").value),
            ttc_full_escape_s=float(self.get_parameter("threat.ttc_full_escape_s").value),
        ) if scan_fresh else 0.0
        auxiliary, brain_latency_ms = self._auxiliary_command()
        auxiliary = auxiliary.rotated(front_yaw)
        if self.control_group == "mppi":
            blend = 0.0
        candidate = fuse_command(mppi, auxiliary, blend)
        command_stamp_s = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if command_stamp_s <= 0.0:
            command_stamp_s = time.monotonic()
        safety = self.safety_gate.apply(
            candidate=candidate,
            mppi_fallback=mppi,
            threats=self._threats,
            scan_fresh=scan_fresh,
            # Use simulation time when it is present so acceleration and jerk
            # metrics are reproducible across machine load; fall back to the
            # steady clock for non-stamped validation inputs.
            stamp_s=command_stamp_s,
            front_yaw=front_yaw,
        )
        if not safety.fly_enabled:
            blend = 0.0
            candidate = safety.command

        self._publish_command(self.bias_pub, auxiliary, message)
        self._publish_command(self.fused_debug_pub, safety.command, message)
        lambda_message = Float32()
        lambda_message.data = float(blend)
        self.lambda_pub.publish(lambda_message)
        safety_active = Bool()
        safety_active.data = safety.reason != "ok"
        self.safety_active_pub.publish(safety_active)
        min_ttc_message = Float32()
        min_ttc_message.data = float(ttc_s)
        self.min_ttc_pub.publish(min_ttc_message)
        valid_distances = [threat.distance_m for threat in self._threats.values() if threat.valid]
        min_distance_message = Float32()
        min_distance_message.data = float(min(valid_distances) if valid_distances else math.inf)
        self.min_distance_pub.publish(min_distance_message)
        if self.output_pub is not None:
            self._publish_command(self.output_pub, safety.command, message)

        elapsed_ms = (time.perf_counter() - started_s) * 1000.0
        brain_latency_message = Float32()
        brain_latency_message.data = float(brain_latency_ms)
        self.brain_latency_pub.publish(brain_latency_message)
        loop_latency_message = Float32()
        loop_latency_message.data = float(elapsed_ms)
        self.loop_latency_pub.publish(loop_latency_message)
        self._publish_brain_debug(message)
        self.logger.record({
            "wall_time_s": time.time(),
            "mode": self.mode,
            "controller": self.control_group,
            "scan_fresh": scan_fresh,
            "safety_reason": safety.reason,
            "lambda": blend,
            "min_ttc_s": self._json_number(ttc_s),
            "u_mppi": self._as_dict(mppi),
            "u_aux": self._as_dict(auxiliary),
            "u_final": self._as_dict(safety.command),
            "linear_accel": {"x": safety.accel.vx, "y": safety.accel.vy},
            "angular_accel_z": safety.accel.wz,
            "linear_jerk": {"x": safety.jerk.vx, "y": safety.jerk.vy},
            "angular_jerk_z": safety.jerk.wz,
            "brain_ms": brain_latency_ms,
            "brain_debug": self._json_mapping(self._brain_debug),
            "brain_asset": self._asset_metadata,
            "control_loop_ms": elapsed_ms,
        })
        if safety.reason != self._last_reason:
            self.get_logger().warning(f"FlyBrain safety state: {safety.reason}")
            self._last_reason = safety.reason

    def _auxiliary_command(self) -> tuple[TwistCommand, float]:
        if self.control_group == "mppi":
            self._brain_debug = {}
            return TwistCommand(), 0.0
        if self.control_group == "heuristic":
            self._brain_debug = {}
            return self.heuristic.update(self._threats), 0.0
        if self.flybrain is None or not self.flybrain.available:
            self._brain_debug = {}
            return TwistCommand(), 0.0
        started_s = time.perf_counter()
        command = self.flybrain.update(self._threats, self._robot_speed_mps)
        elapsed_ms = (time.perf_counter() - started_s) * 1000.0
        self._brain_debug = self.flybrain.last_debug
        if not self.flybrain.available:
            self.get_logger().error(
                f"MaleCNS disabled after controller failure: {self.flybrain.error}; falling back to MPPI."
            )
        return self._limit_auxiliary(command), elapsed_ms

    def _limit_auxiliary(self, command: TwistCommand) -> TwistCommand:
        """Use identical auxiliary envelopes for B, C and D before fusion."""
        max_vx = abs(float(self.get_parameter("fusion.max_aux_vx").value))
        max_vy = abs(float(self.get_parameter("fusion.max_aux_vy").value))
        max_wz = abs(float(self.get_parameter("fusion.max_aux_wz").value))
        return TwistCommand(
            vx=min(max_vx, max(-max_vx, command.vx)),
            vy=min(max_vy, max(-max_vy, command.vy)),
            wz=min(max_wz, max(-max_wz, command.wz)),
        )

    def _front_yaw(self, source: TwistStamped) -> float:
        """Yaw of the scan's physical forward axis in the command frame."""
        if not source.header.frame_id:
            raise ValueError("velocity command has no frame_id")
        if source.header.frame_id == self.expected_scan_frame:
            return 0.0
        transform = self.tf_buffer.lookup_transform(
            source.header.frame_id, self.expected_scan_frame,
            Time())
        # Commands are usually stamped just after the latest odometry TF.
        # Use the latest measured rotation, but never an indefinitely old one.
        tf_stamp = Time.from_msg(transform.header.stamp).nanoseconds
        command_stamp = Time.from_msg(source.header.stamp).nanoseconds
        if tf_stamp and command_stamp and abs(command_stamp - tf_stamp) * 1e-9 > self.scan_timeout_s:
            raise ValueError("stale velocity-frame transform")
        q = transform.transform.rotation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not math.isfinite(norm) or norm < 1e-9:
            raise ValueError("invalid velocity-frame rotation")
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))

    @staticmethod
    def _publish_command(publisher, command: TwistCommand, source: TwistStamped) -> None:
        message = TwistStamped()
        message.header = source.header
        message.twist.linear.x = command.vx
        message.twist.linear.y = command.vy
        message.twist.angular.z = command.wz
        publisher.publish(message)

    @staticmethod
    def _as_dict(command: TwistCommand) -> Mapping[str, float]:
        return {"vx": command.vx, "vy": command.vy, "wz": command.wz}

    @staticmethod
    def _json_number(value: float):
        return value if math.isfinite(value) else None

    def _publish_brain_debug(self, source: TwistStamped) -> None:
        signal = self._brain_debug.get("signal", {})
        signal = signal if isinstance(signal, Mapping) else {}
        values = {
            "brain_turn_score": self._brain_debug.get("turn_score", 0.0),
            "brain_front_score": self._brain_debug.get("front_score", 0.0),
            "brain_fired_descending": self._brain_debug.get("fired_descending", 0.0),
            "brain_signal_left": signal.get("left", 0.0),
            "brain_signal_front": signal.get("front", 0.0),
            "brain_signal_right": signal.get("right", 0.0),
        }
        for name, value in values.items():
            message = Float32()
            try:
                numeric = float(value)
            except (TypeError, ValueError):
                numeric = 0.0
            message.data = numeric if math.isfinite(numeric) else 0.0
            self.brain_debug_pubs[name].publish(message)

    @staticmethod
    def _json_mapping(values: Mapping[str, object]) -> Mapping[str, object]:
        result = {}
        for name, value in values.items():
            if isinstance(value, Mapping):
                result[str(name)] = FlyBrainNode._json_mapping(value)
            elif isinstance(value, (str, bool, int)):
                result[str(name)] = value
            elif isinstance(value, float):
                result[str(name)] = value if math.isfinite(value) else None
            elif isinstance(value, (tuple, list)):
                result[str(name)] = [
                    item if not isinstance(item, float) or math.isfinite(item) else None
                    for item in value
                ]
        return result

    @staticmethod
    def _load_asset_metadata(
        source_dir: str,
        data_dir: str,
        rewired: bool,
        expected_brain_sha256: str,
    ) -> Mapping[str, object]:
        metadata: dict[str, object] = {
            "source_dir": source_dir,
            "data_dir": data_dir,
            "asset_kind": "rewired" if rewired else "real",
            "brain_checksum_sha256": expected_brain_sha256,
        }
        manifest_path = Path(data_dir) / "rewire_manifest.json"
        if manifest_path.is_file():
            try:
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
                if isinstance(manifest, Mapping):
                    metadata["rewire_manifest"] = FlyBrainNode._json_mapping(manifest)
            except (OSError, ValueError, TypeError):
                metadata["rewire_manifest_error"] = "unreadable"
        return metadata


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlyBrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # A second SIGINT while rclpy tears down subscriptions is a normal
            # outcome of ros2 launch / timeout process-group shutdown.
            pass
        # SIGINT can already have shut down the default context while spin()
        # unwinds.  Treat that normal launch termination as successful.
        if rclpy.ok():
            rclpy.shutdown()
