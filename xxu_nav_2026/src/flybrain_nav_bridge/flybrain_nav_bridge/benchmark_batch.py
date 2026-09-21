"""Launch paired Gazebo trials one-at-a-time and preserve every result."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Any, Iterable

import yaml


GROUPS = ("mppi", "heuristic", "flybrain", "rewired")


def _numeric_pose(document: dict[str, Any], name: str) -> tuple[float, float, float]:
    """Validate one explicitly frozen three-degree-of-freedom pose."""
    value = document.get(name)
    if not isinstance(value, dict) or not {"x", "y", "yaw"} <= set(value):
        raise ValueError(f"benchmark plan start.{name} needs numeric x, y, yaw")
    try:
        return tuple(float(value[key]) for key in ("x", "y", "yaw"))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"benchmark plan start.{name} has non-numeric coordinates") from exc


def _frozen_start(plan: dict[str, Any]) -> tuple[tuple[float, float, float], tuple[float, float, float], str]:
    """Return the Gazebo and map poses that every paired trial must receive."""
    start = plan.get("start")
    if not isinstance(start, dict):
        raise ValueError("benchmark plan must include a start mapping")
    gazebo_world = _numeric_pose(start, "gazebo_world")
    map_reference = _numeric_pose(start, "map_reference")
    frame = str(start["map_reference"].get("frame", "")).strip()
    if frame != "map":
        raise ValueError("benchmark plan start.map_reference.frame must be 'map'")
    # A scan-match relocalization introduces an unrecorded, trial-dependent map
    # pose. The paired benchmark deliberately fixes the map initial pose instead.
    if start["map_reference"].get("relocalize", False) is not False:
        raise ValueError("benchmark plan must set start.map_reference.relocalize: false")
    return gazebo_world, map_reference, frame


def _frozen_path(plan: dict[str, Any], key: str) -> str:
    value = str(plan.get(key, "")).strip()
    if not value:
        raise ValueError(f"benchmark plan must provide {key}")
    path = Path(value).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"benchmark plan {key} is not a file: {path}")
    return str(path)


def _frozen_directory(plan: dict[str, Any], key: str) -> str:
    value = str(plan.get(key, "")).strip()
    if not value:
        raise ValueError(f"benchmark plan must provide {key}")
    path = Path(value).expanduser()
    if not path.is_dir():
        raise NotADirectoryError(f"benchmark plan {key} is not a directory: {path}")
    return str(path)


def load_plan(path: Path) -> dict[str, Any]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("benchmark plan must be a YAML mapping")
    preflight_status = document.get("preflight_status")
    if preflight_status is not None and preflight_status != "ready":
        raise RuntimeError(
            f"benchmark plan is not eligible to run (preflight_status={preflight_status!r})"
        )
    for key in (
        "scenarios", "controllers", "seeds", "goals", "start", "scenario_file",
        "map_file", "world_file", "nav2_params_file", "real_data_dir",
        "real_brain_sha256", "rewired_brain_sha256", "scan_self_filter_radius",
        "scan_min_height", "planner_tolerance",
    ):
        if key not in document:
            raise ValueError(f"benchmark plan is missing '{key}'")
    controllers = tuple(document["controllers"])
    invalid = set(controllers) - set(GROUPS)
    if invalid:
        raise ValueError(f"unknown controller groups: {sorted(invalid)}")
    if "rewired" in controllers and not document.get("rewired_data_dir"):
        raise ValueError("Group D requires rewired_data_dir in the benchmark plan")
    scenarios = tuple(document["scenarios"])
    if len(set(scenarios)) != len(scenarios):
        raise ValueError("benchmark plan contains duplicate scenarios")
    if len(set(controllers)) != len(controllers):
        raise ValueError("benchmark plan contains duplicate controllers")
    if not scenarios or not controllers:
        raise ValueError("benchmark plan needs at least one scenario and controller")
    _frozen_start(document)
    for key in ("scenario_file", "map_file", "world_file", "nav2_params_file"):
        _frozen_path(document, key)
    _frozen_directory(document, "real_data_dir")
    for key in ("real_brain_sha256", "rewired_brain_sha256"):
        checksum = str(document.get(key, "")).lower()
        if len(checksum) != 64 or any(character not in "0123456789abcdef" for character in checksum):
            raise ValueError(f"benchmark plan {key} must be a SHA256 checksum")
    for key in ("scan_self_filter_radius", "scan_min_height", "planner_tolerance"):
        try:
            value = float(document[key])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"benchmark plan {key} must be numeric") from exc
        if not value >= 0.0:
            raise ValueError(f"benchmark plan {key} must be non-negative")
    return document


def seed_values(value: Any) -> list[int]:
    if isinstance(value, list):
        return [int(item) for item in value]
    if isinstance(value, dict) and {"start", "count"} <= set(value):
        start, count = int(value["start"]), int(value["count"])
        return list(range(start, start + count))
    raise ValueError("seeds must be a list or {start: N, count: N}")


def trial_command(plan: dict[str, Any], scenario: str, controller: str, seed: int) -> list[str]:
    goal = plan["goals"].get(scenario)
    if not isinstance(goal, dict) or not {"x", "y", "yaw"} <= set(goal):
        raise ValueError(f"scenario '{scenario}' needs a goal with x, y, yaw")
    try:
        goal_x, goal_y, goal_yaw = (float(goal[key]) for key in ("x", "y", "yaw"))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"scenario '{scenario}' has placeholder or non-numeric goal coordinates") from exc
    value = lambda key, default: plan.get(key, default)
    gazebo_start, map_start, _ = _frozen_start(plan)
    scenario_file = _frozen_path(plan, "scenario_file")
    map_file = _frozen_path(plan, "map_file")
    world_file = _frozen_path(plan, "world_file")
    nav2_params_file = _frozen_path(plan, "nav2_params_file")
    real_data_dir = _frozen_directory(plan, "real_data_dir")
    brain_checksum = (
        str(plan["rewired_brain_sha256"])
        if controller == "rewired"
        else str(plan["real_brain_sha256"])
    )
    run_scenario = bool(plan.get("run_scenario", True))
    return [
        "ros2", "launch", "flybrain_nav_bridge", "benchmark.launch.py",
        f"control_group:={controller}",
        f"brain_device:={value('brain_device', 'auto')}",
        f"rewired_data_dir:={value('rewired_data_dir', '')}",
        f"real_data_dir:={real_data_dir}",
        f"brain_checksum_sha256:={brain_checksum}",
        f"scenario:={scenario}", f"seed:={seed}",
        f"scenario_file:={scenario_file}",
        f"map:={map_file}", f"world:={world_file}",
        f"nav2_params_file:={nav2_params_file}",
        f"spawn_x:={gazebo_start[0]}", f"spawn_y:={gazebo_start[1]}",
        f"spawn_yaw:={gazebo_start[2]}",
        "initial_pose_relocalize:=false",
        f"initial_pose_x:={map_start[0]}", f"initial_pose_y:={map_start[1]}",
        f"initial_pose_yaw:={map_start[2]}",
        f"scan_self_filter_radius:={value('scan_self_filter_radius', 0.42)}",
        f"scan_min_height:={value('scan_min_height', 0.30)}",
        f"planner_tolerance:={value('planner_tolerance', 0.0)}",
        f"trial_timeout_s:={value('trial_timeout_s', 30.0)}",
        f"goal_success_guard_m:={value('goal_success_guard_m', 0.30)}",
        f"response_delta_threshold:={value('response_delta_threshold', 0.02)}",
        f"nav2_ready_delay_s:={value('nav2_ready_delay_s', 1.0)}",
        f"results_root:={value('results_root', '/home/naiwu/fly_brain/gazebo_results')}",
        "dispatch_goal:=true",
        f"goal_frame:={goal.get('frame', 'map')}",
        f"goal_x:={goal_x}", f"goal_y:={goal_y}", f"goal_yaw:={goal_yaw}",
        "gazebo_gui:=false", f"run_scenario:={'true' if run_scenario else 'false'}", "record_metrics:=true",
    ]


def terminate(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10)


def run_trial(
    command: list[str],
    result_path: Path,
    deadline_s: float,
    environment: dict[str, str],
    log_path: Path,
) -> None:
    if result_path.exists():
        raise FileExistsError(f"refusing to overwrite existing trial: {result_path}")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log:
        log.write(json.dumps({"command": command}, sort_keys=True) + "\n")
        log.flush()
        process = subprocess.Popen(
            command,
            start_new_session=True,
            env=environment,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        try:
            while time.monotonic() < deadline_s:
                if result_path.is_file():
                    return
                if process.poll() is not None:
                    raise RuntimeError(f"launch exited before writing result (exit={process.returncode})")
                time.sleep(1.0)
            raise TimeoutError(f"trial did not write {result_path} before deadline")
        finally:
            terminate(process)


def write_failure_record(result_path: Path, *, scenario: str, controller: str, seed: int, reason: str) -> None:
    """Persist process-level failures that prevent the ROS metrics node running."""
    if result_path.exists():
        return
    result_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "scenario": scenario,
        "controller": controller,
        "seed": seed,
        "valid_trial": False,
        "failure_reason": reason,
        "goal_success": False,
        "collision": False,
        "timeout": False,
    }
    result_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_dry_run_plan(path: Path, trials: list[tuple[str, str, int]], plan: dict[str, Any]) -> None:
    if path.exists():
        raise FileExistsError(f"refusing to overwrite dry-run plan: {path}")
    lines = [
        "# FlyBrain benchmark dry-run plan",
        "",
        f"Expected trials: {len(trials)}",
        f"Results root: `{plan.get('results_root')}`",
        f"DDS backend: `{plan.get('rmw_implementation', 'default')}`",
        f"Rewired asset: `{plan.get('rewired_data_dir', '')}`",
        f"Real MaleCNS asset: `{plan.get('real_data_dir', '')}`",
        f"Scenario configuration: `{plan.get('scenario_file')}`",
        f"Map: `{plan.get('map_file')}`",
        f"World: `{plan.get('world_file')}`",
        f"Nav2 parameters: `{plan.get('nav2_params_file')}`",
        f"Scan self-filter radius: `{plan.get('scan_self_filter_radius')}` m",
        f"Scan minimum retained height: `{plan.get('scan_min_height')}` m",
        f"Navfn endpoint tolerance: `{plan.get('planner_tolerance')}` m",
        (
            "Frozen start: "
            f"Gazebo ({plan['start']['gazebo_world']['x']}, {plan['start']['gazebo_world']['y']}, "
            f"{plan['start']['gazebo_world']['yaw']}); map "
            f"({plan['start']['map_reference']['x']}, {plan['start']['map_reference']['y']}, "
            f"{plan['start']['map_reference']['yaw']}); relocalize=false"
        ),
        "",
        "| # | Scenario | Controller | Seed | Goal | Obstacle configuration | Rewired asset | Output |",
        "| ---: | --- | --- | ---: | --- | --- | --- | --- |",
    ]
    for index, (scenario, controller, seed) in enumerate(trials, start=1):
        goal = plan["goals"][scenario]
        goal_text = f"{goal.get('frame', 'map')} ({goal['x']}, {goal['y']}, {goal['yaw']})"
        output = Path(str(plan["results_root"])) / "raw" / scenario / controller / f"seed_{seed}.json"
        obstacle = f"`{plan['scenario_file']}` ({scenario}, seed {seed})"
        rewired = f"`{plan.get('rewired_data_dir', '')}`" if controller == "rewired" else "—"
        lines.append(
            f"| {index} | {scenario} | {controller} | {seed} | {goal_text} | {obstacle} | {rewired} | `{output}` |"
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: Iterable[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    plan = load_plan(args.plan.expanduser())
    root = Path(plan.get("results_root", "/home/naiwu/fly_brain/gazebo_results")).expanduser()
    seeds = seed_values(plan["seeds"])
    if len(set(seeds)) != len(seeds):
        raise ValueError("benchmark plan contains duplicate seeds")
    trials = [(scenario, controller, seed) for scenario in plan["scenarios"] for controller in plan["controllers"] for seed in seeds]
    print(json.dumps({"trial_count": len(trials), "seeds": seeds, "controllers": plan["controllers"], "scenarios": plan["scenarios"]}, sort_keys=True))
    for scenario, controller, seed in trials:
        command = trial_command(plan, scenario, controller, seed)
        if args.dry_run:
            print(json.dumps({"scenario": scenario, "controller": controller, "seed": seed, "command": command}))
            continue
        result = root / "raw" / scenario / controller / f"seed_{seed}.json"
        grace_s = float(plan.get("startup_grace_s", 90.0))
        timeout_s = float(plan.get("trial_timeout_s", 30.0))
        environment = os.environ.copy()
        rmw_implementation = str(plan.get("rmw_implementation", "")).strip()
        if rmw_implementation:
            environment["RMW_IMPLEMENTATION"] = rmw_implementation
        cyclonedds_uri = str(plan.get("cyclonedds_uri", "")).strip()
        if cyclonedds_uri:
            environment["CYCLONEDDS_URI"] = cyclonedds_uri
        try:
            run_trial(
                command,
                result,
                time.monotonic() + grace_s + timeout_s,
                environment,
                root / "logs" / scenario / controller / f"seed_{seed}.log",
            )
        except Exception as exc:
            write_failure_record(
                result,
                scenario=scenario,
                controller=controller,
                seed=seed,
                reason=f"{type(exc).__name__}: {exc}",
            )
            print(json.dumps({"scenario": scenario, "controller": controller, "seed": seed, "failure": str(exc)}), file=sys.stderr)
    if args.dry_run:
        write_dry_run_plan(root / "DRY_RUN_PLAN.md", trials, plan)


if __name__ == "__main__":
    main()
