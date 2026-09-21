"""Fault-contained adapter around the externally maintained MaleCNS controller."""

from __future__ import annotations

import importlib
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Optional

from .controller_fusion import TwistCommand
from .threat_extractor import SectorThreat


class FlyBrainAdapter:
    """Load the controller lazily so an unavailable GPU never blocks Nav2.

    The adapter intentionally does not write any brain asset or alter connectome
    data.  A backend failure is represented as an unavailable controller and the
    calling node then sets the auxiliary contribution to zero.
    """

    def __init__(
        self,
        source_dir: str,
        data_dir: str = "",
        require_data_dir: bool = False,
        module_name: str = "controller",
        class_name: str = "FlyBrainController",
        device: str = "auto",
        nominal_speed: float = 0.0,
        ros_vy_sign: float = -1.0,
        ros_wz_sign: float = -1.0,
    ) -> None:
        self.source_dir = str(Path(source_dir).expanduser())
        self.data_dir = str(Path(data_dir).expanduser()) if data_dir else ""
        self.require_data_dir = bool(require_data_dir)
        self.module_name = module_name
        self.class_name = class_name
        self.device = device
        self.nominal_speed = float(nominal_speed)
        self.ros_vy_sign = float(ros_vy_sign)
        self.ros_wz_sign = float(ros_wz_sign)
        self.controller: Optional[Any] = None
        self.error: Optional[str] = None
        # A copy of V1's same-tick diagnostic state.  This is observability
        # only; it never participates in fusion or safety decisions.
        self.last_debug: Mapping[str, Any] = {}
        self._load()

    @property
    def available(self) -> bool:
        return self.controller is not None

    def _load(self) -> None:
        try:
            source = Path(self.source_dir)
            if not source.is_dir():
                raise FileNotFoundError(f"MaleCNS source directory does not exist: {source}")
            if self.require_data_dir and not self.data_dir:
                raise ValueError("Group D requires an explicit rewired MaleCNS data directory")
            # The ROS workspace uses the system Python while MaleCNS owns a
            # separate Python 3.12 virtual environment.  Make its packages
            # visible to the adapter only, avoiding a second ROS installation
            # or any modification of that environment.
            venv_lib = source / ".venv" / "lib"
            if venv_lib.is_dir():
                for site_packages in sorted(venv_lib.glob("python*/site-packages")):
                    site_packages_str = str(site_packages)
                    if site_packages_str not in sys.path:
                        sys.path.insert(0, site_packages_str)
            # The system ROS Python currently exposes a coverage package whose
            # typing API is incompatible with the bundled Numba. MaleCNS's own
            # venv intentionally has no coverage package, so emulate that
            # supported environment rather than changing either installation.
            try:
                coverage_module = importlib.import_module("coverage")
            except ImportError:
                pass
            else:
                coverage_types = getattr(coverage_module, "types", None)
                if not hasattr(coverage_types, "Tracer"):
                    sys.modules["coverage"] = None
            if self.source_dir not in sys.path:
                sys.path.insert(0, self.source_dir)
            module = importlib.import_module(self.module_name)
            controller_class = getattr(module, self.class_name)
            if self.data_dir:
                data = Path(self.data_dir)
                if not data.is_dir():
                    raise FileNotFoundError(f"rewired MaleCNS data directory does not exist: {data}")
                flybrain_module = importlib.import_module("flybrain")
                brain_class = getattr(flybrain_module, "FlyBrain")
                self.controller = controller_class(
                    brain=brain_class(data=str(data), device=self.device),
                    nominal_speed=self.nominal_speed,
                )
            else:
                try:
                    self.controller = controller_class(device=self.device, nominal_speed=self.nominal_speed)
                except TypeError:
                    self.controller = controller_class()
        except Exception as exc:  # Brain failures must not escape into ROS callbacks.
            self.controller = None
            self.error = f"{type(exc).__name__}: {exc}"

    def update(self, threats: Mapping[str, SectorThreat], robot_speed_mps: float) -> TwistCommand:
        if self.controller is None:
            return TwistCommand()
        observation = {
            "sectors": {name: threat.as_observation() for name, threat in threats.items()},
            "left": threats["left"].as_observation(),
            "front": threats["front"].as_observation(),
            "right": threats["right"].as_observation(),
            "robot_speed": float(robot_speed_mps),
        }
        # MaleCNS V1 consumes these six scalar fields.  Never pass None or
        # infinity to the external controller: both lead to unchecked float
        # conversion paths there. A very distant, non-closing pseudo-measurement
        # is equivalent to no threat for its alpha/TTC + beta/distance encoder.
        for name, threat in threats.items():
            distance = threat.distance_m if threat.valid and math.isfinite(threat.distance_m) else 1_000_000.0
            ttc = threat.ttc_s if threat.valid and math.isfinite(threat.ttc_s) else 1_000_000.0
            observation[f"{name}_distance"] = max(1e-6, distance)
            observation[f"{name}_ttc"] = max(1e-6, ttc)
        try:
            # V1's public update() returns its own linearly fused command. The
            # research protocol requires a single shared *smoothstep* fusion,
            # therefore advance V1 exactly once with a zero base command and
            # consume only its same-tick weighted-readout diagnostic. Calling
            # update twice would advance the stateful network twice and corrupt
            # timing. If this compatibility readout changes in a future V1, fail
            # closed rather than applying a doubly fused command.
            self.controller.update(observation, mppi_command=None)
            debug = getattr(self.controller, "last_debug", None)
            self.last_debug = dict(debug) if isinstance(debug, Mapping) else {}
            raw_output = debug.get("fly_command") if isinstance(debug, Mapping) else None
            if raw_output is None:
                raise RuntimeError("MaleCNS V1 did not expose last_debug['fly_command']")
            if isinstance(raw_output, Mapping):
                command = TwistCommand(
                    float(raw_output.get("vx", 0.0)),
                    float(raw_output.get("vy", 0.0)),
                    float(raw_output.get("wz", 0.0)),
                )
            else:
                vx, vy, wz = raw_output
                command = TwistCommand(float(vx), float(vy), float(wz))
            if not command.finite():
                raise ValueError("controller returned a non-finite command")
            # V1's lateral/yaw signs use right-positive coordinates. Convert
            # once here into the base-aligned ROS convention (+y left, +z
            # counter-clockwise/left turn) before any common safety handling.
            return TwistCommand(
                vx=command.vx,
                vy=command.vy * self.ros_vy_sign,
                wz=command.wz * self.ros_wz_sign,
            )
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.controller = None
            self.last_debug = {}
            return TwistCommand()
