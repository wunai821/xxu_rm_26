#!/usr/bin/env python3
"""Analyze stationary noise recordings made by measure_velocity_noise.py."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def read_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def column(rows, name):
    return np.array([float(row[name]) for row in rows])


def stats(values):
    values = np.asarray(values)
    if not len(values) or not np.isfinite(values).all():
        raise ValueError("Empty or nonfinite measurement")
    return {"n": len(values), "mean": float(np.mean(values)),
            "std": float(np.std(values, ddof=1)) if len(values) > 1 else 0.,
            "rms": float(np.sqrt(np.mean(values**2))),
            "abs_p95": float(np.percentile(np.abs(values), 95)),
            "abs_p99": float(np.percentile(np.abs(values), 99)),
            "abs_max": float(np.max(np.abs(values)))}


def window(rows, start, end):
    return [r for r in rows if start <= float(r["t"]) < end]


def odom_summary(rows):
    t = column(rows, "t")
    dt = np.diff(t)
    if len(rows) < 2 or np.any(dt <= 0):
        raise ValueError("Need at least two strictly increasing odometry stamps")
    vx, vy = column(rows, "vx"), column(rows, "vy")
    speed = np.hypot(vx, vy)
    x, y = column(rows, "x"), column(rows, "y")
    yaw = column(rows, "yaw")
    drift = np.hypot(x-x[0], y-y[0])
    yaw_drift = np.arctan2(np.sin(yaw-yaw[0]), np.cos(yaw-yaw[0]))
    # Verify the published velocity against the same odom pose difference.
    fd_x, fd_y = np.diff(x)/dt, np.diff(y)/dt
    fd_vx = np.cos(yaw[1:])*fd_x + np.sin(yaw[1:])*fd_y
    fd_vy = -np.sin(yaw[1:])*fd_x + np.cos(yaw[1:])*fd_y
    return {"samples": len(rows), "span_s": float(t[-1]-t[0]),
            "received_hz": float((len(rows)-1)/(t[-1]-t[0])),
            "dt_median_s": float(np.median(dt)), "dt_max_s": float(np.max(dt)),
            "frames": sorted(set((r["frame"], r["child_frame"]) for r in rows)),
            "vx_mps": stats(vx), "vy_mps": stats(vy),
            "speed_mps": stats(speed), "wz_rad_s": stats(column(rows, "wz")),
            "speed_over_fraction": {str(limit): float(np.mean(speed > limit)) for limit in [.01, .05, .1, .2]},
            "xy_drift_max_m": float(np.max(drift)), "xy_drift_final_m": float(drift[-1]),
            "yaw_drift_max_rad": float(np.max(np.abs(yaw_drift))),
            "pose_difference_twist_error_mps": stats(np.hypot(vx[1:]-fd_vx, vy[1:]-fd_vy)),
            "receipt_latency_s": stats(column(rows, "receipt_t")-t),
            "twist_covariance_diagonal_median": {k: float(np.median(column(rows, k))) for k in ["cov_vx", "cov_vy", "cov_wz"]}}


def analyze(root):
    results = {}
    traces = {}
    for folder in sorted(root.iterdir()):
        if not (folder / "measurement.json").exists():
            continue
        manifest = json.loads((folder / "measurement.json").read_text())
        if not manifest["complete"]:
            raise ValueError(f"Incomplete test: {folder}")
        start = manifest["measurement_start_t"]
        end = start + manifest["measurement_duration_s"]
        data = {topic: window(read_rows(folder / f"{topic}.csv"), start, end)
                for topic in ["odom", "odom_nav", "joints", "commands", "truth"]}
        odom = {topic: odom_summary(data[topic]) for topic in ["odom", "odom_nav"]}
        truth = data["truth"]
        t = column(truth, "t")
        if len(t) < 2 or (np.diff(t) <= 0).any():
            raise ValueError(f"Invalid truth stamps: {folder}")
        x, y = column(truth, "world_x"), column(truth, "world_y")
        qx, qy, qz, qw = [column(truth, k) for k in ["qx", "qy", "qz", "qw"]]
        yaw = np.unwrap(np.arctan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz)))
        true_speed = np.hypot(np.diff(x), np.diff(y))/np.diff(t)
        true_wz = np.diff(yaw)/np.diff(t)
        true_drift = np.hypot(x-x[0], y-y[0])
        commands = data["commands"]
        cmd_max = max(float(np.max(np.abs(column(commands, k)))) for k in ["vx", "vy", "wz"]) if commands else None
        stationary = bool(np.max(true_speed) < .001 and np.max(np.abs(true_wz)) < .001 and np.max(true_drift) < .001)
        nav_by_stamp = {r["t"]: r for r in data["odom_nav"]}
        paired = [(r, nav_by_stamp[r["t"]]) for r in data["odom"] if r["t"] in nav_by_stamp]
        nav_speed_difference = [abs(np.hypot(float(a["vx"]), float(a["vy"]))-np.hypot(float(b["vx"]),float(b["vy"]))) for a,b in paired]
        results[folder.name] = {"measurement_start_t": start, "measurement_end_t": end,
            "stationary_verified": stationary, "odom": odom,
            "truth": {"samples": len(truth), "speed_mps": stats(true_speed),
                      "wz_rad_s": stats(true_wz), "xy_drift_max_m": float(np.max(true_drift))},
            "joints": {"samples": len(data["joints"]),
                       "gimbal_wz_rad_s": stats(column(data["joints"], "gimbal_wz")),
                       "wheel_abs_max_rad_s": stats(column(data["joints"], "wheel_abs_max"))},
            "command_samples": len(commands), "command_absolute_max": cmd_max,
            "paired_odom_nav_speed_difference_mps": stats(nav_speed_difference),
            "limitations": ["One trial per gimbal mode; simulation only; no moving-chassis accuracy or repeatability claim.",
                            "Stationarity checked with Gazebo model poses at about 20 Hz; numerical pose differences are not a force/contact measurement.",
                            "Reported noise is measured velocity during verified stationarity, including estimator drift and bias.",
                            "Published covariance is a configured constant, not a measured variance."]}
        traces[folder.name] = (start, data)
    if not results:
        raise ValueError("No completed measurements found")
    (root / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(len(results), 2, figsize=(12, 3.5*len(results)), squeeze=False)
    for axis, (name, (start, data)) in zip(axes, traces.items()):
        rows = data["odom"]
        t = column(rows, "t")-start
        speed = np.hypot(column(rows, "vx"), column(rows, "vy"))
        axis[0].plot(t, speed, color="#2563eb", lw=.8, label="/odom speed")
        tr = data["truth"]
        true_speed = np.hypot(np.diff(column(tr, "world_x")), np.diff(column(tr, "world_y")))/np.diff(column(tr,"t"))
        axis[0].plot(column(tr,"t")[1:]-start, true_speed, color="#16a34a", lw=1, label="Gazebo truth")
        axis[0].axhline(.05, color="#dc2626", ls="--", lw=.8, label="0.05 m/s reference")
        axis[0].set(title=f"Stationary chassis / gimbal {name}", xlabel="Simulation time in measurement (s)", ylabel="Planar speed (m/s)")
        axis[0].legend(loc="upper right", fontsize=8)
        axis[1].hist(speed, bins=35, color="#2563eb", alpha=.8, density=True)
        p95 = np.percentile(speed,95)
        axis[1].axvline(p95, color="#dc2626", ls="--", label=f"P95 = {p95:.4f} m/s")
        axis[1].set(title="/odom speed distribution", xlabel="Planar speed (m/s)", ylabel="Density")
        axis[1].legend(fontsize=8)
    fig.suptitle("Small Point-LIO velocity noise: 15 s warmup + 60 s measurement", fontsize=14)
    fig.tight_layout()
    fig.savefig(root / "velocity_noise.png", dpi=180)
    fig.savefig(root / "velocity_noise.svg")
    plt.close(fig)
    lines = ["# 里程计静止速度噪声测试（2026-10-04）", "",
             "complex_mapping 世界，默认出生位姿，底盘不运动，使用云台补偿版 Small Point-LIO。",
             "ROS_DOMAIN_ID=84，Gazebo transport 使用独立分区；不启动 Nav2，不发布运动命令。",
             "两组独立启动；从首条 /odom 起预热 15 秒，随后测量 60 秒仿真时间。",
             "里程计按消息时间戳采样并排除重复或倒退；Gazebo 真值按模型名称 xxu 选择。", "",
             "| 云台工况 | 真值确认静止 | /odom 样本 | 平均速度 m/s | P95 m/s | 最大 m/s | vx 标准差 m/s | vy 标准差 m/s | wz 标准差 rad/s | >0.05 m/s 占比 |",
             "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for name, result in results.items():
        v = result["odom"]["odom"]
        lines.append(f"| {name} | {result['stationary_verified']} | {v['samples']} | {v['speed_mps']['mean']:.5f} | {v['speed_mps']['abs_p95']:.5f} | {v['speed_mps']['abs_max']:.5f} | {v['vx_mps']['std']:.5f} | {v['vy_mps']['std']:.5f} | {v['wz_rad_s']['std']:.5f} | {100*v['speed_over_fraction']['0.05']:.1f}% |")
    lines += ["", "## 数据质量与解释", ""]
    for name, result in results.items():
        v = result["odom"]["odom"]
        n = result["odom"]["odom_nav"]
        lines.append(f"- {name}：里程计 {v['received_hz']:.2f} Hz，最大消息间隔 {v['dt_max_s']:.3f} s；云台速度均值 {result['joints']['gimbal_wz_rad_s']['mean']:.5f} rad/s；真值速度峰值 {result['truth']['speed_mps']['abs_max']:.3e} m/s，真值最大位移 {result['truth']['xy_drift_max_m']:.3e} m。")
        lines.append(f"- {name}：/odom 最大相对位移 {v['xy_drift_max_m']:.5f} m；/odom_nav 平均/P95/峰值 {n['speed_mps']['mean']:.5f}/{n['speed_mps']['abs_p95']:.5f}/{n['speed_mps']['abs_max']:.5f} m/s；同时间戳两话题速度大小差峰值 {result['paired_odom_nav_speed_difference_mps']['abs_max']:.3e} m/s。")
        lines.append(f"- {name}：发布 twist 与同一位姿差分的水平速度差峰值 {v['pose_difference_twist_error_mps']['abs_max']:.3e} m/s。最终命令绝对值峰值 {result['command_absolute_max']}。")
    lines += ["", "当前发布代码用相邻 base 位姿差分计算速度；若上述差分一致性误差接近零，说明输出速度与位姿差分一致，静止时的速度主要表现为位姿抖动经差分放大。该观察不能单独定位估计器内部抖动根因。",
              "仅测试每种云台工况一轮仿真静止表现，不代表运动精度、实车噪声或重复性；没有给定验收阈值，因此不作合格/不合格判定。0.05 m/s 仅用作比较参考。",
              "twist covariance 是配置常数，不是本次测得的方差。本次不修改里程计、滤波器或导航参数。", "",
              "两组采样均完整结束。结束后的在线参数快照查询超时，启动配置已保存为各组 lio_config.yaml，且与安装目录中的配置 SHA256 一致；不把该配置副本视为完整在线参数快照。", "",
              "完整数值：results.json；原始数据和启动日志：hold/、spin/；图表：velocity_noise.png、velocity_noise.svg。", ""]
    (root / "REPORT.md").write_text("\n".join(lines))
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.recordings), indent=2))
