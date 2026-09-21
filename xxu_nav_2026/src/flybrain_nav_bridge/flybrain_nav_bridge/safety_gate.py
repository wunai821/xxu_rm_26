"""Independent command sanitization for all experimental groups."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Optional

from .controller_fusion import TwistCommand
from .threat_extractor import SectorThreat


@dataclass(frozen=True)
class SafetyResult:
    command: TwistCommand
    fly_enabled: bool
    reason: str = "ok"
    accel: TwistCommand = TwistCommand()
    jerk: TwistCommand = TwistCommand()


class SafetyGate:
    """Limit velocity and acceleration without allowing an auxiliary bypass."""

    def __init__(
        self,
        *,
        max_vx: float,
        max_vy: float,
        max_wz: float,
        max_ax: float,
        max_ay: float,
        max_awz: float,
        max_jx: float,
        max_jy: float,
        max_jwz: float,
        emergency_distance_m: float,
    ) -> None:
        self.max_vx = abs(max_vx)
        self.max_vy = abs(max_vy)
        self.max_wz = abs(max_wz)
        self.max_ax = abs(max_ax)
        self.max_ay = abs(max_ay)
        self.max_awz = abs(max_awz)
        self.max_jx = abs(max_jx)
        self.max_jy = abs(max_jy)
        self.max_jwz = abs(max_jwz)
        self.emergency_distance_m = max(0.0, emergency_distance_m)
        self._last_command: Optional[TwistCommand] = None
        self._last_stamp_s: Optional[float] = None
        self._last_accel = TwistCommand()

    def apply(
        self,
        *,
        candidate: TwistCommand,
        mppi_fallback: TwistCommand,
        threats: Mapping[str, SectorThreat],
        scan_fresh: bool,
        stamp_s: float,
        front_yaw: float = 0.0,
    ) -> SafetyResult:
        fallback = self._velocity_clamp(mppi_fallback if mppi_fallback.finite() else TwistCommand())
        if not scan_fresh:
            return self._remember(fallback, stamp_s, False, "stale_or_missing_scan")
        valid_candidate = candidate.finite()
        output = self._velocity_clamp(candidate) if valid_candidate else fallback
        front = threats.get("front")
        if front and front.valid and front.distance_m <= self.emergency_distance_m:
            # The emergency rule is independent of controller identity.
            output = self._remove_forward(output, front_yaw)
            return self._remember(output, stamp_s, valid_candidate, "front_emergency", front_yaw=front_yaw)
        return self._remember(output, stamp_s, valid_candidate,
                              "ok" if valid_candidate else "invalid_auxiliary_command")

    def _remember(
        self,
        command: TwistCommand,
        stamp_s: float,
        fly_enabled: bool,
        reason: str,
        front_yaw: Optional[float] = None,
    ) -> SafetyResult:
        limited = command
        accel = TwistCommand()
        jerk = TwistCommand()
        if self._last_command is not None and self._last_stamp_s is not None:
            dt = stamp_s - self._last_stamp_s
            if 0.001 <= dt <= 1.0:
                desired_accel = TwistCommand(
                    vx=(command.vx - self._last_command.vx) / dt,
                    vy=(command.vy - self._last_command.vy) / dt,
                    wz=(command.wz - self._last_command.wz) / dt,
                )
                accel = TwistCommand(
                    vx=min(self.max_ax, max(-self.max_ax, desired_accel.vx)),
                    vy=min(self.max_ay, max(-self.max_ay, desired_accel.vy)),
                    wz=min(self.max_awz, max(-self.max_awz, desired_accel.wz)),
                )
                accel = TwistCommand(
                    vx=self._rate_limit(accel.vx, self._last_accel.vx, self.max_jx * dt),
                    vy=self._rate_limit(accel.vy, self._last_accel.vy, self.max_jy * dt),
                    wz=self._rate_limit(accel.wz, self._last_accel.wz, self.max_jwz * dt),
                )
                limited = TwistCommand(
                    vx=self._last_command.vx + accel.vx * dt,
                    vy=self._last_command.vy + accel.vy * dt,
                    wz=self._last_command.wz + accel.wz * dt,
                )
                # Do not let jerk smoothing carry a command past its target
                # (especially through zero into unintended reverse motion).
                limited = TwistCommand(*(
                    min(max(previous, target), max(min(previous, target), value))
                    for previous, target, value in zip(
                        (self._last_command.vx, self._last_command.vy, self._last_command.wz),
                        (command.vx, command.vy, command.wz),
                        (limited.vx, limited.vy, limited.wz))
                ))
                limited = self._velocity_clamp(limited)
                if front_yaw is not None:
                    limited = self._remove_forward(limited, front_yaw)
                accel = TwistCommand(
                    (limited.vx - self._last_command.vx) / dt,
                    (limited.vy - self._last_command.vy) / dt,
                    (limited.wz - self._last_command.wz) / dt,
                )
                jerk = TwistCommand(
                    vx=(accel.vx - self._last_accel.vx) / dt,
                    vy=(accel.vy - self._last_accel.vy) / dt,
                    wz=(accel.wz - self._last_accel.wz) / dt,
                )
        self._last_command = limited
        self._last_stamp_s = stamp_s
        # An emergency discontinuity is telemetry, not the starting
        # acceleration for the next smooth command.
        self._last_accel = TwistCommand() if front_yaw is not None else accel
        return SafetyResult(limited, fly_enabled, reason, accel, jerk)

    def reset(self) -> None:
        self._last_command = None
        self._last_stamp_s = None
        self._last_accel = TwistCommand()

    def _remove_forward(self, command: TwistCommand, yaw: float) -> TwistCommand:
        c, s = math.cos(yaw), math.sin(yaw)
        forward = max(0.0, command.vx * c + command.vy * s)
        vx, vy = command.vx - forward * c, command.vy - forward * s
        # Uniform scaling preserves the emergency half-plane even when
        # per-axis velocity limits differ.
        scale = min(1.0, self.max_vx / abs(vx) if vx else 1.0,
                    self.max_vy / abs(vy) if vy else 1.0)
        return TwistCommand(vx * scale, vy * scale, command.wz)

    def _velocity_clamp(self, command: TwistCommand) -> TwistCommand:
        return TwistCommand(
            vx=min(self.max_vx, max(-self.max_vx, command.vx)),
            vy=min(self.max_vy, max(-self.max_vy, command.vy)),
            wz=min(self.max_wz, max(-self.max_wz, command.wz)),
        )

    @staticmethod
    def _rate_limit(value: float, previous: float, max_delta: float) -> float:
        return min(previous + max_delta, max(previous - max_delta, value))
