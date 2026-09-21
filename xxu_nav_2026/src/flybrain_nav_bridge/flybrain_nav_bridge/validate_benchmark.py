"""Validate append-only paired benchmark results before statistical analysis."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable

from .benchmark_batch import load_plan, seed_values


def _identity(record: dict[str, Any]) -> tuple[str, str, int] | None:
    try:
        return str(record["scenario"]), str(record["controller"]), int(record["seed"])
    except (KeyError, TypeError, ValueError):
        return None


def _nonfinite_paths(value: Any, prefix: str = "$") -> list[str]:
    """Find JSON numeric values that cannot be analysed honestly."""
    if isinstance(value, float):
        return [] if math.isfinite(value) else [prefix]
    if isinstance(value, dict):
        return [
            path
            for key, item in value.items()
            for path in _nonfinite_paths(item, f"{prefix}.{key}")
        ]
    if isinstance(value, list):
        return [
            path
            for index, item in enumerate(value)
            for path in _nonfinite_paths(item, f"{prefix}[{index}]")
        ]
    return []


def _read_json(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return None, f"{type(exc).__name__}: {exc}"
    if not isinstance(payload, dict):
        return None, "result JSON must be an object"
    return payload, None


def _valid_timing_errors(record: dict[str, Any]) -> list[str]:
    if record.get("valid_trial") is not True:
        return []
    errors: list[str] = []
    start, end = record.get("simulation_start_s"), record.get("simulation_end_s")
    if not isinstance(start, (int, float)) or not math.isfinite(float(start)):
        errors.append("valid trial lacks finite simulation_start_s")
    if not isinstance(end, (int, float)) or not math.isfinite(float(end)):
        errors.append("valid trial lacks finite simulation_end_s")
    if not errors and float(end) < float(start):
        errors.append("simulation_end_s precedes simulation_start_s")
    elapsed = record.get("time_to_goal_s")
    if not isinstance(elapsed, (int, float)) or not math.isfinite(float(elapsed)) or float(elapsed) < 0.0:
        errors.append("valid trial lacks non-negative finite time_to_goal_s")
    elif bool(record.get("goal_success")) and float(elapsed) < 0.5:
        straight = record.get("straight_line_start_goal_m")
        if isinstance(straight, (int, float)) and math.isfinite(float(straight)) and float(straight) > 0.5:
            errors.append("implausibly fast success for a start-goal separation above 0.5 m")
    if bool(record.get("goal_success")):
        final_error = record.get("goal_final_distance_m")
        guard = record.get("goal_success_guard_m")
        if not isinstance(final_error, (int, float)) or not math.isfinite(float(final_error)):
            errors.append("goal success lacks a finite final map-frame error")
        elif not isinstance(guard, (int, float)) or not math.isfinite(float(guard)):
            errors.append("goal success lacks a finite final-error guard")
        elif float(final_error) > float(guard):
            errors.append("goal success final map-frame error exceeds the frozen guard")
    return errors


def _runtime_asset_errors(root: Path, expected: set[tuple[str, str, int]], plan: dict[str, Any]) -> list[str]:
    """Prove Group C/D loaded their intended brain assets at runtime, not just in YAML."""
    runtime = root / "raw" / "runtime" / "flybrain_runtime.jsonl"
    if not runtime.is_file():
        return [f"missing runtime asset log: {runtime}"]
    events: dict[tuple[str, str, int], list[dict[str, Any]]] = defaultdict(list)
    errors: list[str] = []
    for line_number, line in enumerate(runtime.read_text(encoding="utf-8").splitlines(), start=1):
        try:
            record = json.loads(line)
        except ValueError as exc:
            errors.append(f"runtime log line {line_number} is invalid JSON: {exc}")
            continue
        if not isinstance(record, dict) or record.get("event") != "bridge_started":
            continue
        identity = _identity(record)
        if identity is not None and identity[1] in {"flybrain", "rewired"}:
            events[identity].append(record)
    for scenario, controller, seed in sorted(expected):
        if controller not in {"flybrain", "rewired"}:
            continue
        matching = events.get((scenario, controller, seed), [])
        if len(matching) != 1:
            errors.append(
                f"runtime bridge-start evidence for {(scenario, controller, seed)} has count {len(matching)}, expected 1"
            )
            continue
        asset = matching[0].get("brain_asset")
        if not isinstance(asset, dict):
            errors.append(f"runtime asset metadata missing for {(scenario, controller, seed)}")
            continue
        expected_kind = "rewired" if controller == "rewired" else "real"
        expected_path = str(plan["rewired_data_dir"] if controller == "rewired" else plan["real_data_dir"])
        expected_checksum = str(
            plan["rewired_brain_sha256"] if controller == "rewired" else plan["real_brain_sha256"]
        )
        if asset.get("asset_kind") != expected_kind:
            errors.append(f"runtime asset kind mismatch for {(scenario, controller, seed)}")
        if asset.get("data_dir") != expected_path:
            errors.append(f"runtime asset path mismatch for {(scenario, controller, seed)}")
        if asset.get("brain_checksum_sha256") != expected_checksum:
            errors.append(f"runtime brain checksum mismatch for {(scenario, controller, seed)}")
        if controller == "rewired":
            manifest = asset.get("rewire_manifest")
            if not isinstance(manifest, dict):
                errors.append(f"runtime rewired manifest missing for {(scenario, controller, seed)}")
            elif int(manifest.get("connection_count", -1)) != 25_582_938:
                errors.append(f"runtime rewired connection count mismatch for {(scenario, controller, seed)}")
    return errors


def validate(plan_path: Path, results_root: Path) -> dict[str, Any]:
    plan = load_plan(plan_path)
    expected = {
        (scenario, controller, seed)
        for scenario in plan["scenarios"]
        for controller in plan["controllers"]
        for seed in seed_values(plan["seeds"])
    }
    files = sorted(
        path for path in (results_root / "raw").glob("**/*.json")
        if "runtime" not in path.relative_to(results_root / "raw").parts
    )
    seen: dict[tuple[str, str, int], list[Path]] = defaultdict(list)
    records: dict[tuple[str, str, int], dict[str, Any]] = {}
    parse_errors: list[str] = []
    unexpected_path_errors: list[str] = []
    nonfinite: list[str] = []
    timing_errors: list[str] = []
    invalid_trials: list[dict[str, Any]] = []
    for path in files:
        record, error = _read_json(path)
        if error is not None or record is None:
            parse_errors.append(f"{path}: {error}")
            continue
        identity = _identity(record)
        if identity is None:
            parse_errors.append(f"{path}: missing or invalid scenario/controller/seed")
            continue
        seen[identity].append(path)
        records.setdefault(identity, record)
        expected_path = results_root / "raw" / identity[0] / identity[1] / f"seed_{identity[2]}.json"
        if path != expected_path:
            unexpected_path_errors.append(f"{path}: identity belongs at {expected_path}")
        nonfinite.extend(f"{path}:{item}" for item in _nonfinite_paths(record))
        timing_errors.extend(f"{path}: {item}" for item in _valid_timing_errors(record))
        if record.get("valid_trial") is not True:
            invalid_trials.append({
                "scenario": identity[0], "controller": identity[1], "seed": identity[2],
                "failure_reason": record.get("failure_reason"),
            })
    observed = set(seen)
    duplicates = {
        "/".join(map(str, identity)): [str(path) for path in paths]
        for identity, paths in seen.items() if len(paths) > 1
    }
    missing = sorted(expected - observed)
    extras = sorted(observed - expected)
    paired_missing: list[dict[str, Any]] = []
    for scenario in plan["scenarios"]:
        for seed in seed_values(plan["seeds"]):
            present = sorted(controller for current_scenario, controller, current_seed in observed
                             if current_scenario == scenario and current_seed == seed)
            expected_controllers = sorted(plan["controllers"])
            if present != expected_controllers:
                paired_missing.append({
                    "scenario": scenario, "seed": seed,
                    "present": present, "expected": expected_controllers,
                })
    runtime_errors = _runtime_asset_errors(results_root, expected, plan)
    summary: dict[str, Any] = {
        "plan": str(plan_path),
        "results_root": str(results_root),
        "expected_trial_count": len(expected),
        "result_file_count": len(files),
        "unique_trial_count": len(observed),
        "valid_trial_count": sum(record.get("valid_trial") is True for record in records.values()),
        "missing_trials": [list(item) for item in missing],
        "extra_trials": [list(item) for item in extras],
        "duplicate_trials": duplicates,
        "invalid_trials": invalid_trials,
        "parse_errors": parse_errors,
        "nonfinite_values": nonfinite,
        "timing_errors": timing_errors,
        "unexpected_path_errors": unexpected_path_errors,
        "paired_completeness_errors": paired_missing,
        "runtime_asset_errors": runtime_errors,
    }
    summary["passed"] = not any((
        len(files) != len(expected), missing, extras, duplicates, invalid_trials,
        parse_errors, nonfinite, timing_errors, unexpected_path_errors, paired_missing,
        runtime_errors,
    ))
    return summary


def main(argv: Iterable[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path)
    parser.add_argument("--results-root", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    plan_path = args.plan.expanduser()
    plan = load_plan(plan_path)
    results_root = (args.results_root or Path(str(plan["results_root"]))).expanduser()
    output = (args.output or results_root / "SMOKE_VALIDATION.json").expanduser()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite validation result: {output}")
    summary = validate(plan_path, results_root)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({
        "passed": summary["passed"],
        "expected_trial_count": summary["expected_trial_count"],
        "result_file_count": summary["result_file_count"],
        "valid_trial_count": summary["valid_trial_count"],
        "output": str(output),
    }, sort_keys=True))
    if not summary["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
