"""LaserScan-only threat extraction with conservative temporal filtering."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Iterable, Mapping, Optional


@dataclass(frozen=True)
class SectorThreat:
    """Closest reliable return and radial closing estimate for one scan sector."""

    name: str
    distance_m: float = math.inf
    relative_speed_mps: float = 0.0
    ttc_s: float = math.inf
    bearing_rad: float = 0.0
    valid: bool = False
    samples: int = 0

    def as_observation(self) -> dict:
        return {
            "distance": self.distance_m,
            "relative_speed": self.relative_speed_mps,
            "ttc": self.ttc_s,
            "bearing": self.bearing_rad,
            "valid": self.valid,
            "samples": self.samples,
        }


@dataclass
class _SectorState:
    distance_m: Optional[float] = None
    closing_mps: float = 0.0
    stamp_s: Optional[float] = None


class ThreatExtractor:
    """Estimate left/front/right threats from scans expressed in a base-aligned frame.

    This implementation relies on standard ROS planar conventions: ``+x`` is
    forward and ``+y`` is left.  LaserScan angles therefore increase toward the
    robot's left.  The node warns when the frame is not the configured frame;
    frame transformation is intentionally left to the scan producer so a
    callback never silently uses an invalid transform.
    """

    SECTORS = ("left", "front", "right")

    def __init__(
        self,
        *,
        front_angle_deg: float = 20.0,
        side_angle_deg: float = 90.0,
        min_valid_points: int = 3,
        distance_percentile: float = 0.10,
        distance_filter_alpha: float = 0.45,
        closing_filter_alpha: float = 0.35,
        min_closing_speed_mps: float = 0.03,
        max_closing_speed_mps: float = 8.0,
        max_ttc_s: float = 60.0,
    ) -> None:
        if not 0.0 < front_angle_deg < side_angle_deg <= 180.0:
            raise ValueError("sector angles must satisfy 0 < front < side <= 180")
        self.front_angle_rad = math.radians(front_angle_deg)
        self.side_angle_rad = math.radians(side_angle_deg)
        self.min_valid_points = max(1, int(min_valid_points))
        self.distance_percentile = min(0.5, max(0.0, float(distance_percentile)))
        self.distance_filter_alpha = min(1.0, max(0.0, float(distance_filter_alpha)))
        self.closing_filter_alpha = min(1.0, max(0.0, float(closing_filter_alpha)))
        self.min_closing_speed_mps = max(0.0, float(min_closing_speed_mps))
        self.max_closing_speed_mps = max(self.min_closing_speed_mps, float(max_closing_speed_mps))
        self.max_ttc_s = max(0.1, float(max_ttc_s))
        self._state = {name: _SectorState() for name in self.SECTORS}

    def process_scan(
        self,
        ranges: Iterable[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        stamp_s: float,
    ) -> Dict[str, SectorThreat]:
        """Process a scan and return filtered sector measurements.

        A low percentile rather than a raw minimum rejects isolated lidar noise
        while still responding to an approaching obstacle near the sector edge.
        Closing speed is only calculated across plausible scan intervals.
        """
        buckets: Dict[str, list[tuple[float, float]]] = {
            name: [] for name in self.SECTORS
        }
        for index, raw_distance in enumerate(ranges):
            try:
                distance = float(raw_distance)
            except (TypeError, ValueError):
                continue
            if not math.isfinite(distance) or distance < range_min or distance > range_max:
                continue
            angle = angle_min + index * angle_increment
            sector = self._sector_for_angle(angle)
            if sector is not None:
                buckets[sector].append((distance, angle))

        result: Dict[str, SectorThreat] = {}
        for name in self.SECTORS:
            result[name] = self._measure_sector(name, buckets[name], stamp_s)
        return result

    def _sector_for_angle(self, angle: float) -> Optional[str]:
        # Positive laser angles are left under REP-103 when the scan frame is
        # base-aligned.  Keep the exact front boundary exclusively in front.
        if -self.front_angle_rad <= angle <= self.front_angle_rad:
            return "front"
        if self.front_angle_rad < angle <= self.side_angle_rad:
            return "left"
        if -self.side_angle_rad <= angle < -self.front_angle_rad:
            return "right"
        return None

    def _measure_sector(
        self,
        name: str,
        samples: list[tuple[float, float]],
        stamp_s: float,
    ) -> SectorThreat:
        state = self._state[name]
        if len(samples) < self.min_valid_points:
            return SectorThreat(name=name, samples=len(samples))

        samples.sort(key=lambda item: item[0])
        percentile_index = min(
            len(samples) - 1,
            int(round((len(samples) - 1) * self.distance_percentile)),
        )
        raw_distance, bearing = samples[percentile_index]
        if state.distance_m is None:
            distance = raw_distance
        else:
            alpha = self.distance_filter_alpha
            distance = alpha * raw_distance + (1.0 - alpha) * state.distance_m

        closing_raw = 0.0
        if state.distance_m is not None and state.stamp_s is not None:
            dt = stamp_s - state.stamp_s
            if 0.02 <= dt <= 1.0:
                # Positive means the radial separation is decreasing.
                closing_raw = (state.distance_m - distance) / dt
                closing_raw = min(self.max_closing_speed_mps, max(0.0, closing_raw))
        alpha = self.closing_filter_alpha
        closing = alpha * closing_raw + (1.0 - alpha) * state.closing_mps
        if closing <= self.min_closing_speed_mps:
            closing = 0.0
            ttc = math.inf
        else:
            ttc = min(self.max_ttc_s, distance / closing)

        state.distance_m = distance
        state.closing_mps = closing
        state.stamp_s = stamp_s
        return SectorThreat(
            name=name,
            distance_m=distance,
            relative_speed_mps=closing,
            ttc_s=ttc,
            bearing_rad=bearing,
            valid=True,
            samples=len(samples),
        )


def most_urgent_threat(threats: Mapping[str, SectorThreat]) -> Optional[SectorThreat]:
    """Return the finite-TTC threat that drives the shared fusion lambda."""
    candidates = [threat for threat in threats.values() if threat.valid and math.isfinite(threat.ttc_s)]
    return min(candidates, key=lambda threat: threat.ttc_s) if candidates else None
