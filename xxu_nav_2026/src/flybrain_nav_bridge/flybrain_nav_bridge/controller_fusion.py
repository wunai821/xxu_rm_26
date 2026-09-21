"""Shared TTC fusion and deterministic baseline policies."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

from .threat_extractor import SectorThreat, most_urgent_threat


@dataclass(frozen=True)
class TwistCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def finite(self) -> bool:
        return all(math.isfinite(value) for value in (self.vx, self.vy, self.wz))

    def scaled(self, factor: float) -> "TwistCommand":
        return TwistCommand(self.vx * factor, self.vy * factor, self.wz * factor)

    def plus(self, other: "TwistCommand") -> "TwistCommand":
        return TwistCommand(self.vx + other.vx, self.vy + other.vy, self.wz + other.wz)

    def rotated(self, yaw: float) -> "TwistCommand":
        """Express planar velocity in a frame rotated by the supplied TF yaw."""
        c, s = math.cos(yaw), math.sin(yaw)
        return TwistCommand(c * self.vx - s * self.vy, s * self.vx + c * self.vy, self.wz)


def smoothstep_lambda(ttc_s: float, *, ttc_safe_s: float = 1.5, ttc_full_escape_s: float = 0.5) -> float:
    """TTC blend specified by the benchmark protocol, with safe guardrails."""
    if not math.isfinite(ttc_s):
        return 0.0
    if ttc_safe_s <= ttc_full_escape_s:
        raise ValueError("ttc_safe_s must be greater than ttc_full_escape_s")
    x = min(1.0, max(0.0, (ttc_safe_s - ttc_s) / (ttc_safe_s - ttc_full_escape_s)))
    return x * x * (3.0 - 2.0 * x)


class HeuristicEscapePolicy:
    """Fixed B-group policy sharing sectors, TTC and lambda with the fly group."""

    def __init__(
        self,
        *,
        max_vx: float,
        max_vy: float,
        max_wz: float,
        front_reverse_vx: float,
        front_lateral_vy: float,
    ) -> None:
        self.max_vx = abs(max_vx)
        self.max_vy = abs(max_vy)
        self.max_wz = abs(max_wz)
        self.front_reverse_vx = min(abs(front_reverse_vx), self.max_vx)
        self.front_lateral_vy = min(abs(front_lateral_vy), self.max_vy)

    def update(self, threats: Mapping[str, SectorThreat]) -> TwistCommand:
        urgent = most_urgent_threat(threats)
        if urgent is None:
            return TwistCommand()
        if urgent.name == "left":
            # +y is left: a left-side threat produces a rightward command.
            return TwistCommand(vy=-self.max_vy, wz=-self.max_wz)
        if urgent.name == "right":
            return TwistCommand(vy=self.max_vy, wz=self.max_wz)

        left = threats.get("left")
        right = threats.get("right")
        left_clearance = left.distance_m if left and left.valid else 0.0
        right_clearance = right.distance_m if right and right.valid else 0.0
        # Drive into the clearer side. A tie stays neutral laterally so the
        # reverse component rather than an arbitrary sign resolves the threat.
        lateral = 0.0
        if left_clearance > right_clearance:
            lateral = self.front_lateral_vy
        elif right_clearance > left_clearance:
            lateral = -self.front_lateral_vy
        return TwistCommand(vx=-self.front_reverse_vx, vy=lateral)


def fuse_command(mppi: TwistCommand, auxiliary: TwistCommand, blend: float) -> TwistCommand:
    """Apply the same smooth blend for heuristic, MaleCNS and control groups."""
    return mppi.plus(auxiliary.scaled(min(1.0, max(0.0, blend))))
