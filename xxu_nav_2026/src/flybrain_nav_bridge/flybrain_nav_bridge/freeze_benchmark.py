"""Create an append-only code and asset manifest before a benchmark run."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from typing import Iterable


WORKSPACE = Path("/home/naiwu/xxu_2026")
NAV_ROOT = WORKSPACE / "xxu_nav_2026"
PACKAGE = NAV_ROOT / "src/flybrain_nav_bridge"
FLY_ROOT = Path("/home/naiwu/fly_brain")
FLYBRAIN_PYTHON = FLY_ROOT / ".venv/bin/python"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_state(path: Path) -> dict[str, object]:
    try:
        commit = subprocess.check_output(
            ["git", "-C", str(path), "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL
        ).strip()
        status = subprocess.check_output(
            ["git", "-C", str(path), "status", "--short"], text=True, stderr=subprocess.DEVNULL
        ).splitlines()
        return {"is_git_repository": True, "commit": commit, "status_short": status}
    except (OSError, subprocess.CalledProcessError):
        return {"is_git_repository": False}


def brain_validation(asset_dir: Path) -> dict[str, object]:
    manifest_path = asset_dir / "rewire_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    checks = {
        "manifest_present": manifest_path.is_file(),
        "brain_sha256": sha256(asset_dir / "brain.npz"),
        "weights_sha256": sha256(asset_dir / "weights.npz"),
        "manifest": manifest,
    }
    expected = manifest.get("output", {}) if isinstance(manifest, dict) else {}
    checks["brain_checksum_matches_manifest"] = checks["brain_sha256"] == expected.get("brain_sha256")
    checks["weights_checksum_matches_manifest"] = checks["weights_sha256"] == expected.get("weights_sha256")
    try:
        if not FLYBRAIN_PYTHON.is_file():
            raise FileNotFoundError(f"missing FlyBrain runtime: {FLYBRAIN_PYTHON}")
        probe = (
            "import json, sys; from flybrain import FlyBrain; "
            "brain = FlyBrain(data=sys.argv[1], device='cpu'); "
            "print(json.dumps({'neuron_count': int(brain.n), "
            "'connection_count': int(len(brain.weights))}))"
        )
        measured = json.loads(
            subprocess.check_output(
                [str(FLYBRAIN_PYTHON), "-c", probe, str(asset_dir)],
                text=True,
                stderr=subprocess.STDOUT,
            )
        )
        checks["brain_load"] = "passed"
        checks["brain_runtime_python"] = str(FLYBRAIN_PYTHON)
        checks["neuron_count"] = measured["neuron_count"]
        checks["connection_count"] = measured["connection_count"]
        checks["connection_count_matches_manifest"] = checks["connection_count"] == manifest.get("connection_count")
    except Exception as exc:
        checks["brain_load"] = f"failed: {type(exc).__name__}: {exc}"
        checks["connection_count_matches_manifest"] = False
    return checks


def source_asset_validation(asset_dir: Path) -> dict[str, object]:
    """Record the real MaleCNS asset that Group C is explicitly instructed to load."""
    required = (asset_dir / "brain.npz", asset_dir / "weights.npz")
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"real MaleCNS asset is incomplete: {missing}")
    return {
        "data_dir": str(asset_dir),
        "brain_sha256": sha256(asset_dir / "brain.npz"),
        "weights_sha256": sha256(asset_dir / "weights.npz"),
    }


def main(argv: Iterable[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--plan", type=Path, default=PACKAGE / "config/benchmark_plan.yaml")
    parser.add_argument(
        "--scenario-config", type=Path, default=PACKAGE / "config/benchmark_scenarios.yaml"
    )
    parser.add_argument(
        "--asset-dir", type=Path, default=FLY_ROOT / "rewired/seed_1000"
    )
    parser.add_argument("--real-asset-dir", type=Path, default=Path("/home/naiwu/fly-data"))
    args = parser.parse_args(argv)
    output = args.output.expanduser()
    plan_path = args.plan.expanduser()
    scenario_config_path = args.scenario_config.expanduser()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite freeze manifest: {output}")
    tracked = {
        "malecns_controller": FLY_ROOT / "controller.py",
        "malecns_system_id_readout": FLY_ROOT / "results/system_id_readout.json",
        "threat_extractor": PACKAGE / "flybrain_nav_bridge/threat_extractor.py",
        "fusion": PACKAGE / "flybrain_nav_bridge/controller_fusion.py",
        "safety_gate": PACKAGE / "flybrain_nav_bridge/safety_gate.py",
        "flybrain_adapter": PACKAGE / "flybrain_nav_bridge/flybrain_adapter.py",
        "bridge_node": PACKAGE / "flybrain_nav_bridge/flybrain_node.py",
        "scenario_manager": PACKAGE / "flybrain_nav_bridge/scenario_manager.py",
        "metrics_logger": PACKAGE / "flybrain_nav_bridge/benchmark_metrics.py",
        "benchmark_runner": PACKAGE / "flybrain_nav_bridge/benchmark_batch.py",
        "benchmark_validator": PACKAGE / "flybrain_nav_bridge/validate_benchmark.py",
        "benchmark_launch": PACKAGE / "launch/benchmark.launch.py",
        "bridge_launch": PACKAGE / "launch/flybrain_bridge.launch.py",
        "scenario_launch": PACKAGE / "launch/scenario.launch.py",
        "metrics_launch": PACKAGE / "launch/metrics.launch.py",
        "benchmark_obstacle_model": PACKAGE / "models/flybrain_obstacle/model.sdf",
        "benchmark_plan": plan_path,
        "scenario_config": scenario_config_path,
        "nav2_parameters": NAV_ROOT / "src/xxu_bringup/config/nav2_navigation.yaml",
        "navigation_launch": NAV_ROOT / "src/xxu_bringup/launch/navigation.launch.py",
        "simulation_launch": NAV_ROOT / "src/xxu_bringup/launch/simulation.launch.py",
        "gazebo_launch": NAV_ROOT / "src/xxu_description/launch/gazebo.launch.py",
        "pointcloud_to_scan_source": NAV_ROOT / "src/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp",
        "pointcloud_to_scan_header": NAV_ROOT / "src/pointcloud_to_laserscan/include/pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp",
        "map_yaml": NAV_ROOT / "src/xxu_bringup/maps/complex_map.yaml",
        "map_image": NAV_ROOT / "src/xxu_bringup/maps/complex_map.pgm",
        "gazebo_world": NAV_ROOT / "src/xxu_description/worlds/complex_mapping.sdf",
    }
    missing = [name for name, path in tracked.items() if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"missing frozen inputs: {missing}")
    payload = {
        "workspace": git_state(WORKSPACE),
        "fly_brain": git_state(FLY_ROOT),
        "files": {
            name: {"path": str(path), "sha256": sha256(path)}
            for name, path in tracked.items()
        },
        "real_asset_validation": source_asset_validation(args.real_asset_dir.expanduser()),
        "rewired_asset_validation": brain_validation(args.asset_dir.expanduser()),
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({
        "output": str(output),
        "rewired_brain_load": payload["rewired_asset_validation"]["brain_load"],
        "real_brain_sha256": payload["real_asset_validation"]["brain_sha256"],
        "connection_count": payload["rewired_asset_validation"].get("connection_count"),
    }, sort_keys=True))


if __name__ == "__main__":
    main()
