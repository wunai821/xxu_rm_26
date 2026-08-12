"""AI 控制器调参工具 —— 核心节点。

通过记录机器人的追踪误差、速度命令和实际速度等数据，
使用 AI（LLM）分析每个调参窗口的指标并建议参数调整，
自动应用于 Nav2 控制器，实现全向 PID 追踪控制器的系统化调优。
提供 Web UI 实时监控调参过程。
"""

import json
import math
import os
import re
import statistics
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PointStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

# 默认可调参数列表（全向 PID 追踪控制器的全部可调参数）
DEFAULT_TUNABLE_PARAMETERS = [
    "translation_kp",
    "translation_ki",
    "translation_kd",
    "rotation_kp",
    "rotation_ki",
    "rotation_kd",
    "lookahead_dist",
    "min_lookahead_dist",
    "max_lookahead_dist",
    "lookahead_time",
    "curvature_min",
    "curvature_max",
    "reduction_ratio_at_high_curvature",
    "max_velocity_scaling_factor_rate",
    "path_smoothing_max_offset",
    "curvature_lookahead_dist",
    "curvature_sample_dist",
    "max_lateral_accel",
    "curvature_max_deceleration",
    "min_approach_linear_velocity",
    "approach_velocity_scaling_dist",
]

# 每个参数的安全范围（下限, 上限），AI 建议值会被裁剪到此范围内
PARAMETER_LIMITS = {
    "translation_kp": (0.0, 10.0),
    "translation_ki": (0.0, 2.0),
    "translation_kd": (0.0, 5.0),
    "rotation_kp": (0.0, 10.0),
    "rotation_ki": (0.0, 2.0),
    "rotation_kd": (0.0, 5.0),
    "lookahead_dist": (0.05, 3.0),
    "min_lookahead_dist": (0.05, 2.0),
    "max_lookahead_dist": (0.05, 4.0),
    "lookahead_time": (0.1, 5.0),
    "curvature_min": (0.0, 5.0),
    "curvature_max": (0.01, 10.0),
    "reduction_ratio_at_high_curvature": (0.05, 1.0),
    "max_velocity_scaling_factor_rate": (0.01, 1.0),
    "path_smoothing_max_offset": (0.0, 0.15),
    "curvature_lookahead_dist": (0.2, 5.0),
    "curvature_sample_dist": (0.05, 0.5),
    "max_lateral_accel": (0.1, 5.0),
    "curvature_max_deceleration": (0.1, 5.0),
    "min_approach_linear_velocity": (0.01, 0.3),
    "approach_velocity_scaling_dist": (0.1, 2.0),
}

# 各调参阶段的配置：调优目标和允许修改的参数列表
STAGE_PROFILES = {
    "translation": {
        "objective": "Tune straight-line translation tracking. Prioritize low linear velocity error, low path tracking RMSE, and minimal lateral oscillation. Do not change rotation, lookahead, curvature, or approach parameters.",
        "parameters": ["translation_kp", "translation_ki", "translation_kd"],
    },
    "lookahead": {
        "objective": "Tune lookahead behavior on longer straight runs. Prioritize smooth path tracking without lag or overshoot. Do not change PID, curvature, or approach parameters.",
        "parameters": ["lookahead_dist", "min_lookahead_dist", "max_lookahead_dist", "lookahead_time"],
    },
    "rotation": {
        "objective": "Tune heading control. Prioritize low angular velocity error, low angular oscillation, and stable final yaw. Do not change translation, lookahead, curvature, or approach parameters.",
        "parameters": ["rotation_kp", "rotation_ki", "rotation_kd"],
    },
    "curvature": {
        "objective": "Tune curve speed limiting. Prioritize smooth turns, reduced command saturation, and low oscillation through L-shaped paths. Do not change PID, lookahead, or approach parameters.",
        "parameters": [
            "curvature_min",
            "curvature_max",
            "reduction_ratio_at_high_curvature",
            "max_velocity_scaling_factor_rate",
            "path_smoothing_max_offset",
            "curvature_lookahead_dist",
            "curvature_sample_dist",
            "max_lateral_accel",
            "curvature_max_deceleration",
        ],
    },
    "approach": {
        "objective": "Tune final approach slowdown. Prioritize stable stopping near short goals without crawling too early or overshooting. Do not change PID, lookahead, or curvature parameters.",
        "parameters": ["min_approach_linear_velocity", "approach_velocity_scaling_dist"],
    },
    "comprehensive": {
        "objective": "Validate and make small final adjustments across all controller parameters. Prefer conservative changes and avoid undoing good stage-specific behavior.",
        "parameters": DEFAULT_TUNABLE_PARAMETERS,
    },
    "custom": {
        "objective": "Tune the controller for the custom goal sequence. Prefer conservative changes and only adjust parameters allowed by the configuration.",
        "parameters": DEFAULT_TUNABLE_PARAMETERS,
    },
}

# 默认阶段推进顺序（不包含 custom）
DEFAULT_STAGE_SEQUENCE = [
    "translation",
    "lookahead",
    "rotation",
    "curvature",
    "approach",
    "comprehensive",
]


# ---- 数学工具函数 ----

def mean(values: Iterable[float]) -> Optional[float]:
    """计算平均值。"""
    values = list(values)
    return sum(values) / len(values) if values else None


def rmse(values: Iterable[float]) -> Optional[float]:
    """计算均方根误差 (Root Mean Square Error)。"""
    values = list(values)
    if not values:
        return None
    return math.sqrt(sum(v * v for v in values) / len(values))


def variance(values: Iterable[float]) -> Optional[float]:
    """计算总体方差（用于衡量振荡程度）。"""
    values = list(values)
    if len(values) < 2:
        return None
    return statistics.pvariance(values)


def sign_changes(values: List[float], deadband: float = 0.02) -> int:
    """统计信号穿越死区的次数（用于衡量振荡频率）。"""
    last_sign = 0
    changes = 0
    for value in values:
        sign = 1 if value > deadband else -1 if value < -deadband else 0
        if sign == 0:
            continue
        if last_sign != 0 and sign != last_sign:
            changes += 1
        last_sign = sign
    return changes


def quaternion_to_yaw(quaternion: Any) -> float:
    """将 ROS 四元数消息转换为偏航角（绕 Z 轴旋转）。"""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(current: float, previous: float) -> float:
    """计算两个角度之间的最短差值（归一化到 [-pi, pi]）。"""
    return math.atan2(math.sin(current - previous), math.cos(current - previous))
    last_sign = 0
    changes = 0
    for value in values:
        sign = 1 if value > deadband else -1 if value < -deadband else 0
        if sign == 0:
            continue
        if last_sign != 0 and sign != last_sign:
            changes += 1
        last_sign = sign
    return changes


def quaternion_to_yaw(quaternion: Any) -> float:
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(current: float, previous: float) -> float:
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


UI_HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>AI Controller Tuner</title>
  <style>
    :root { color-scheme: dark; --bg: #0f1317; --panel: #171d23; --line: #2b3844; --text: #e8eef2; --muted: #9fb0bc; --accent: #8bd3ff; --ok: #7ee787; --warn: #f2cc60; --bad: #ff7b72; }
    * { box-sizing: border-box; }
    body { margin: 0; font-family: system-ui, sans-serif; background: var(--bg); color: var(--text); }
    header { height: 56px; display: flex; align-items: center; justify-content: space-between; padding: 0 18px; background: #151b21; border-bottom: 1px solid var(--line); }
    h1 { margin: 0; font-size: 19px; font-weight: 650; }
    .header-status { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; justify-content: flex-end; }
    .pill { border: 1px solid var(--line); border-radius: 999px; padding: 5px 10px; color: var(--muted); font-size: 13px; }
    main { height: calc(100vh - 56px); display: grid; grid-template-columns: minmax(420px, 1fr) 360px; gap: 0; }
    #chat { display: flex; flex-direction: column; min-width: 0; border-right: 1px solid var(--line); }
    .live-summary { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 8px; padding: 10px 14px; background: #11181e; border-bottom: 1px solid var(--line); }
    .summary-card { min-width: 0; padding: 9px 10px; border: 1px solid var(--line); border-radius: 7px; background: var(--panel); }
    .summary-card .label { color: var(--muted); font-size: 11px; }
    .summary-card .number { display: block; margin-top: 3px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; color: var(--text); font-size: 17px; font-weight: 700; font-variant-numeric: tabular-nums; }
    .timeline-head { display: flex; justify-content: space-between; align-items: center; padding: 10px 18px 0; color: var(--muted); font-size: 12px; }
    .timeline-head strong { color: var(--accent); font-size: 13px; }
    #messages { flex: 1; overflow-y: auto; padding: 12px 18px 18px; display: flex; flex-direction: column; gap: 12px; }
    .msg { max-width: 860px; border: 1px solid var(--line); border-radius: 8px; padding: 12px 13px; background: var(--panel); }
    .msg.ai { border-color: #31506a; background: #14212b; margin-left: 42px; }
    .msg.system { border-color: #3d4550; background: #171a1f; }
    .msg.apply { border-color: #2e5d46; background: #132119; }
    .msg.error { border-color: #6b3434; background: #2a1517; }
    .meta { display: flex; gap: 10px; align-items: center; margin-bottom: 7px; color: var(--muted); font-size: 12px; }
    .role { color: var(--accent); font-weight: 650; }
    .msg.error .role { color: var(--bad); }
    .msg.apply .role { color: var(--ok); }
    .content { white-space: pre-wrap; overflow-wrap: anywhere; line-height: 1.45; }
    .bar { height: 4px; background: #26313b; }
    .bar > div { height: 100%; width: 0%; background: linear-gradient(90deg, #8bd3ff, #7ee787); transition: width .25s ease; }
    aside { overflow-y: auto; padding: 14px; background: #11161b; }
    section { background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 12px; margin-bottom: 12px; }
    h2 { margin: 0 0 9px; font-size: 14px; color: var(--accent); }
    .row { display: flex; justify-content: space-between; gap: 12px; padding: 5px 0; border-bottom: 1px solid #23303a; }
    .row:last-child { border-bottom: 0; }
    .key { color: var(--muted); }
    .value { text-align: right; font-variant-numeric: tabular-nums; }
    pre { margin: 0; white-space: pre-wrap; overflow-wrap: anywhere; color: #dbe7ef; font-size: 12px; }
    @media (max-width: 900px) { main { grid-template-columns: 1fr; height: auto; } #chat { height: 70vh; border-right: 0; border-bottom: 1px solid var(--line); } aside { height: auto; } .live-summary { grid-template-columns: repeat(2, minmax(0, 1fr)); } }
  </style>
</head>
<body>
  <header>
    <h1>AI Controller Tuner</h1>
    <div class="header-status">
      <div class="pill" id="ui-address">UI: connecting</div>
      <div class="pill" id="headline">connecting</div>
    </div>
  </header>
  <main>
    <div id="chat">
      <div class="bar"><div id="progress"></div></div>
      <div class="live-summary">
        <div class="summary-card"><span class="label">阶段</span><strong class="number" id="stage-progress">-</strong></div>
        <div class="summary-card"><span class="label">当前轮次</span><strong class="number" id="round-progress">-</strong></div>
        <div class="summary-card"><span class="label">总完成步数</span><strong class="number" id="total-progress">-</strong></div>
        <div class="summary-card"><span class="label">本阶段最佳</span><strong class="number" id="best-score">-</strong></div>
      </div>
      <div class="timeline-head"><strong>实时窗口记录</strong><span id="timeline-status">跟随最新</span></div>
      <div id="messages"></div>
    </div>
    <aside>
      <section><h2>Status</h2><div id="status"></div></section>
      <section><h2>Current Window</h2><div id="window"></div></section>
      <section><h2>Last Metrics</h2><div id="metrics"></div></section>
      <section><h2>Active Parameters</h2><pre id="params"></pre></section>
      <section><h2>Best Observed</h2><pre id="best"></pre></section>
    </aside>
  </main>
  <script>
    let lastEventSignature = "";
    let firstEventRender = true;
    const fmt = (v) => {
      if (v === null || v === undefined) return "-";
      if (typeof v === "number") return Number.isInteger(v) ? String(v) : v.toFixed(4);
      return String(v);
    };
    const rows = (obj) => Object.entries(obj).map(([k,v]) =>
      `<div class="row"><span class="key">${k}</span><span class="value">${fmt(v)}</span></div>`
    ).join("");
    const esc = (s) => String(s ?? "").replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
    const timeText = (ts) => ts ? new Date(ts * 1000).toLocaleTimeString() : "";
    const eventText = (ev) => {
      if (ev.details && Object.keys(ev.details).length) {
        return `${ev.message}\\n${JSON.stringify(ev.details, null, 2)}`;
      }
      return ev.message;
    };
    const eventLabel = (ev) => {
      const parts = [];
      if (ev.stage) parts.push(ev.stage);
      if (ev.window_id) parts.push(`window ${ev.window_id}`);
      return parts.length ? parts.join(" / ") : "";
    };
    async function refresh() {
      let data;
      try {
        const response = await fetch("/status", {cache: "no-store"});
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        data = await response.json();
      } catch (err) {
        document.getElementById("headline").textContent = `UI disconnected: ${err}`;
        return;
      }
      const age = ((Date.now() / 1000) - data.updated_at).toFixed(1);
      const uiAddress = data.ui_url || (data.ui_host && data.ui_port ? `http://${data.ui_host}:${data.ui_port}` : "-");
      const progress = data.progress || {};
      const stageIndex = progress.stage_index || "-";
      const stageTotal = progress.stage_total || "-";
      const stageRound = progress.current_stage_round || 1;
      const completedTotal = progress.completed_windows_total ?? 0;
      const bestScore = data.best_score;
      document.getElementById("stage-progress").textContent = `${stageIndex} / ${stageTotal} · ${data.stage || "-"}`;
      document.getElementById("round-progress").textContent = `第 ${stageRound} 轮`;
      document.getElementById("total-progress").textContent = `第 ${completedTotal} 步`;
      document.getElementById("best-score").textContent = bestScore === null || bestScore === undefined ? "-" : fmt(bestScore);
      document.getElementById("ui-address").textContent = data.ui_listening
        ? `UI: ${uiAddress}`
        : `UI unavailable${data.ui_error ? `: ${data.ui_error}` : ""}`;
      document.getElementById("headline").textContent = `${data.stage} / ${data.ai_status}`;
      const elapsed = data.current_window?.elapsed_sec || 0;
      const duration = data.record_duration_sec || 60;
      document.getElementById("progress").style.width = `${Math.min(100, 100 * elapsed / duration)}%`;
      document.getElementById("status").innerHTML = rows({
        stage: data.stage,
        stage_progress: `${stageIndex} / ${stageTotal}`,
        current_round: stageRound,
        completed_total: completedTotal,
        objective: data.objective_name,
        ai_status: data.ai_status,
        auto_apply: data.auto_apply,
        auto_stage_advance: data.auto_stage_advance,
        ui_address: uiAddress,
        ui_port_configured: data.ui_configured_port,
        ui_listening: data.ui_listening,
        stage_windows: data.stage_window_count,
        last_error: data.last_error || "-",
        next_stage_rule: `${data.stage_min_windows || "-"} min / ${data.stage_advance_after_non_improving_windows || "-"} flat`,
        updated_age_sec: age
      });
      document.getElementById("window").innerHTML = rows({
        window_id: data.current_window?.window_id,
        stage_round: data.current_window?.stage_round,
        stage_windows_completed: data.current_window?.stage_windows_completed,
        stage: data.current_window?.stage,
        elapsed_sec: elapsed,
        duration_sec: duration,
        tracking_errors: data.current_window?.tracking_errors,
        cmd_nav: data.current_window?.cmd_nav,
        odom_samples: data.current_window?.actual_velocities
      });
      document.getElementById("metrics").innerHTML = rows({
        score: data.last_score,
        samples: data.last_metrics?.sample_count,
        tracking_rmse: data.last_metrics?.tracking_error_rmse,
        actual_speed: data.last_metrics?.actual_linear_mean,
        speed_utilization: data.last_metrics?.speed_utilization,
        saturation: data.last_metrics?.command_saturation_ratio
      });
      document.getElementById("params").textContent = JSON.stringify(data.current_active_params || {}, null, 2);
      document.getElementById("best").textContent = JSON.stringify({
        best_score: data.best_score,
        best_params: data.best_params,
        best_metrics: data.best_metrics,
        non_improving_windows: data.non_improving_windows,
        stage_window_count: data.stage_window_count,
        stage_progress: `${stageIndex} / ${stageTotal}`,
        log_path: data.log_path
      }, null, 2);
      const events = data.dialog_events || [];
      const box = document.getElementById("messages");
      const eventSignature = events.map(ev => `${ev.time}|${ev.title}|${ev.window_id}|${ev.message}`).join("\\n");
      if (eventSignature !== lastEventSignature) {
        const followLatest = firstEventRender || (box.scrollHeight - box.scrollTop - box.clientHeight < 48);
        box.innerHTML = events.map(ev =>
          `<div class="msg ${esc(ev.kind || "system")}"><div class="meta"><span class="role">${esc(ev.title || ev.kind || "event")}</span><span>${esc(eventLabel(ev))}</span><span>${esc(timeText(ev.time))}</span></div><div class="content">${esc(eventText(ev))}</div></div>`
        ).join("");
        if (followLatest) box.scrollTop = box.scrollHeight;
        document.getElementById("timeline-status").textContent = followLatest ? "跟随最新" : "已暂停跟随（滚到底部恢复）";
        lastEventSignature = eventSignature;
        firstEventRender = false;
      }
    }
    document.getElementById("messages").addEventListener("scroll", (event) => {
      const box = event.currentTarget;
      const atLatest = box.scrollHeight - box.scrollTop - box.clientHeight < 48;
      document.getElementById("timeline-status").textContent = atLatest
        ? "跟随最新"
        : "已暂停跟随（滚到底部恢复）";
    });
    refresh();
    setInterval(refresh, 1000);
  </script>
</body>
</html>
"""


class AiControllerTuner(Node):
    """AI 控制器调参节点。

    核心流程：
    1. 每个窗口（record_duration_sec）采集追踪误差、速度命令、实际速度等数据
    2. 计算评分指标，调用 AI API 获取参数调整建议
    3. 对建议做安全裁剪后自动应用到控制器
    4. 跟踪最佳参数，支持阶段自动推进
    5. 通过 Web UI 实时展示调参状态
    """

    def __init__(self) -> None:
        super().__init__("ai_controller_tuner")

        # ======== ROS 参数声明 ========
        self.declare_parameter("controller_node", "/controller_server")
        self.declare_parameter("plugin_name", "FollowPath")
        self.declare_parameter("record_duration_sec", 30.0)
        self.declare_parameter("auto_apply", True)
        self.declare_parameter("api_base_url", "https://api.openai.com/v1")
        self.declare_parameter("api_key", "")
        self.declare_parameter("api_key_env", "OPENAI_API_KEY")
        self.declare_parameter("model", "gpt-4o-mini")
        self.declare_parameter("api_timeout_sec", 90.0)
        self.declare_parameter("api_max_tokens", 500)
        self.declare_parameter("api_json_mode", True)
        self.declare_parameter("debug_stage", "translation")
        self.declare_parameter("objective_name", "fast_stable")
        self.declare_parameter("target_speed_utilization", 0.75)
        self.declare_parameter("max_parameter_step_ratio", 0.10)
        self.declare_parameter("max_changed_parameters_per_window", 2)
        self.declare_parameter("min_ai_confidence", 0.60)
        self.declare_parameter("score_tracking_weight", 4.0)
        self.declare_parameter("score_linear_error_weight", 2.0)
        self.declare_parameter("score_angular_error_weight", 1.5)
        self.declare_parameter("score_lateral_oscillation_weight", 0.8)
        self.declare_parameter("score_angular_oscillation_weight", 0.5)
        self.declare_parameter("score_saturation_weight", 0.5)
        self.declare_parameter("score_sign_change_weight", 0.1)
        self.declare_parameter("score_speed_shortfall_weight", 1.2)
        self.declare_parameter("tunable_parameters", DEFAULT_TUNABLE_PARAMETERS)
        self.declare_parameter("keep_best_observed", True)
        self.declare_parameter("auto_restore_best", False)
        self.declare_parameter("restore_after_non_improving_windows", 3)
        self.declare_parameter("auto_stage_advance", False)
        self.declare_parameter("stage_sequence", DEFAULT_STAGE_SEQUENCE)
        self.declare_parameter("stage_min_windows", 2)
        self.declare_parameter("stage_advance_after_non_improving_windows", 2)
        self.declare_parameter("apply_best_on_stage_advance", True)
        self.declare_parameter("stop_after_final_stage_stable", True)
        self.declare_parameter("stage_topic", "/ai_controller_tuner/current_stage")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_nav_topic", "/cmd_vel_nav")
        self.declare_parameter("cmd_vel_smoothed_topic", "/cmd_vel_smoothed")
        self.declare_parameter("local_plan_topic", "/local_plan")
        self.declare_parameter("lookahead_point_topic", "/lookahead_point")
        self.declare_parameter("log_dir", "~/.ros/ai_controller_tuner")
        self.declare_parameter("mock_ai_response", "")
        self.declare_parameter("ui_enabled", True)
        self.declare_parameter("ui_host", "127.0.0.1")
        self.declare_parameter("ui_port", 8765)
        self.declare_parameter("ui_fallback_to_ephemeral_port", True)

        # ======== 读取并缓存参数 ========
        self.controller_node = self.get_parameter("controller_node").value
        self.plugin_name = self.get_parameter("plugin_name").value
        self.record_duration_sec = float(self.get_parameter("record_duration_sec").value)
        self.auto_apply = bool(self.get_parameter("auto_apply").value)
        self.api_base_url = str(self.get_parameter("api_base_url").value).rstrip("/")
        self.api_key = self.get_parameter("api_key").value
        self.api_key_env = self.get_parameter("api_key_env").value
        self.model = self.get_parameter("model").value
        self.api_timeout_sec = float(self.get_parameter("api_timeout_sec").value)
        self.api_max_tokens = int(self.get_parameter("api_max_tokens").value)
        self.api_json_mode = bool(self.get_parameter("api_json_mode").value)
        self.debug_stage = self.get_parameter("debug_stage").value
        self.objective_name = self.get_parameter("objective_name").value
        self.target_speed_utilization = float(self.get_parameter("target_speed_utilization").value)
        self.max_parameter_step_ratio = float(
            self.get_parameter("max_parameter_step_ratio").value
        )
        self.max_changed_parameters_per_window = max(
            1, int(self.get_parameter("max_changed_parameters_per_window").value)
        )
        self.min_ai_confidence = float(self.get_parameter("min_ai_confidence").value)
        self.score_weights = {
            "tracking": float(self.get_parameter("score_tracking_weight").value),
            "linear_error": float(self.get_parameter("score_linear_error_weight").value),
            "angular_error": float(self.get_parameter("score_angular_error_weight").value),
            "lateral_oscillation": float(self.get_parameter("score_lateral_oscillation_weight").value),
            "angular_oscillation": float(self.get_parameter("score_angular_oscillation_weight").value),
            "saturation": float(self.get_parameter("score_saturation_weight").value),
            "sign_change": float(self.get_parameter("score_sign_change_weight").value),
            "speed_shortfall": float(self.get_parameter("score_speed_shortfall_weight").value),
        }
        self.tunable_parameters = list(self.get_parameter("tunable_parameters").value)
        self.keep_best_observed = bool(self.get_parameter("keep_best_observed").value)
        self.auto_restore_best = bool(self.get_parameter("auto_restore_best").value)
        self.restore_after_non_improving_windows = int(
            self.get_parameter("restore_after_non_improving_windows").value
        )
        self.auto_stage_advance = bool(self.get_parameter("auto_stage_advance").value)
        self.stage_sequence = [
            str(stage)
            for stage in self.get_parameter("stage_sequence").value
            if str(stage) in STAGE_PROFILES and str(stage) != "custom"
        ]
        if not self.stage_sequence:
            self.stage_sequence = list(DEFAULT_STAGE_SEQUENCE)
        self.stage_min_windows = int(self.get_parameter("stage_min_windows").value)
        self.stage_advance_after_non_improving_windows = int(
            self.get_parameter("stage_advance_after_non_improving_windows").value
        )
        self.apply_best_on_stage_advance = bool(
            self.get_parameter("apply_best_on_stage_advance").value
        )
        self.stop_after_final_stage_stable = bool(
            self.get_parameter("stop_after_final_stage_stable").value
        )
        self.stage_topic = self.get_parameter("stage_topic").value
        self.mock_ai_response = self.get_parameter("mock_ai_response").value
        self.ui_enabled = bool(self.get_parameter("ui_enabled").value)
        self.ui_host = str(self.get_parameter("ui_host").value)
        self.ui_port = int(self.get_parameter("ui_port").value)
        self.ui_fallback_to_ephemeral_port = bool(
            self.get_parameter("ui_fallback_to_ephemeral_port").value
        )

        # ======== 日志目录 ========
        log_dir = Path(os.path.expanduser(self.get_parameter("log_dir").value))
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / f"session_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"

        # ======== 内部状态 ========
        self._lock = threading.Lock()                # 数据采集与分析的线程锁
        self._analysis_running = False               # 是否正在执行 AI 分析
        self._latest_odom: Optional[Odometry] = None
        self._latest_odom_sample: Optional[Tuple[float, float, float, float]] = None  # (timestamp, x, y, yaw)
        self._best_score: Optional[float] = None     # 当前阶段的最佳评分
        self._best_params: Dict[str, float] = {}     # 当前阶段的最佳参数
        self._best_metrics: Dict[str, Any] = {}      # 最佳评分对应的指标快照
        self._non_improving_windows = 0              # 连续未改进的窗口数
        self._stage_window_count = 0                 # 当前阶段已完成的窗口数
        self._window_id = 1                          # 全局窗口编号（递增）
        self._tuning_complete = False                # 调参是否已完成
        self._ui_state_lock = threading.Lock()
        self._ui_state: Dict[str, Any] = {}          # Web UI 共享状态
        self._dialog_events: List[Dict[str, Any]] = []  # Web UI 事件流
        self._ui_server = None
        self._ui_thread = None

        # 初始化第一个窗口的数据缓冲区
        self._reset_window()
        self._update_ui_state(
            stage=self.debug_stage,
            objective=self._stage_objective(),
            objective_name=self.objective_name,
            record_duration_sec=self.record_duration_sec,
            target_speed_utilization=self.target_speed_utilization,
            max_parameter_step_ratio=self.max_parameter_step_ratio,
            max_changed_parameters_per_window=self.max_changed_parameters_per_window,
            min_ai_confidence=self.min_ai_confidence,
            score_weights=self.score_weights,
            active_parameters=self._active_tunable_parameters(),
            auto_apply=self.auto_apply,
            keep_best_observed=self.keep_best_observed,
            auto_restore_best=self.auto_restore_best,
            auto_stage_advance=self.auto_stage_advance,
            stage_sequence=self.stage_sequence,
            stage_min_windows=self.stage_min_windows,
            stage_advance_after_non_improving_windows=self.stage_advance_after_non_improving_windows,
            stop_after_final_stage_stable=self.stop_after_final_stage_stable,
            log_path=str(self.log_path),
            ai_status="recording",
            last_metrics=None,
            last_score=None,
            current_active_params={},
            best_score=None,
            best_params={},
            best_metrics={},
            non_improving_windows=0,
            stage_window_count=0,
            last_raw_suggestions={},
            last_safe_suggestions={},
            last_apply_result={},
            last_error=None,
            ui_enabled=self.ui_enabled,
            ui_configured_host=self.ui_host,
            ui_configured_port=self.ui_port,
            ui_host=self.ui_host,
            ui_port=self.ui_port,
            ui_url=None,
            ui_listening=False,
            ui_error=None,
        )
        self._add_dialog_event(
            "system",
            "Tuner Started",
            (
                f"Stage: {self.debug_stage}\n"
                f"Objective: {self.objective_name}\n"
                f"Window: {self.record_duration_sec:.1f}s\n"
                f"Active params: {self._active_tunable_parameters()}"
            ),
        )

        # ======== ROS 客户端与话题 ========
        controller_node = self.controller_node.rstrip("/")
        # 用于读取/写入控制器参数的服务客户端
        self._get_params_client = self.create_client(GetParameters, f"{controller_node}/get_parameters")
        self._set_params_client = self.create_client(SetParameters, f"{controller_node}/set_parameters")
        # 阶段切换话题发布者（TRANSIENT_LOCAL 确保新订阅者能获取最后一条消息）
        stage_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._stage_publisher = self.create_publisher(String, self.stage_topic, stage_qos)
        self.create_timer(2.0, self._publish_current_stage)
        self._publish_current_stage()

        # 订阅里程计、速度命令、平滑速度、局部路径、前视点
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._on_odom, 50
        )
        self.create_subscription(
            TwistStamped, self.get_parameter("cmd_vel_nav_topic").value, self._on_cmd_vel_nav, 50
        )
        self.create_subscription(
            Twist, self.get_parameter("cmd_vel_nav_topic").value, self._on_cmd_vel_nav, 50
        )
        self.create_subscription(
            TwistStamped,
            self.get_parameter("cmd_vel_smoothed_topic").value,
            self._on_cmd_vel_smoothed,
            50,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_smoothed_topic").value,
            self._on_cmd_vel_smoothed,
            50,
        )
        self.create_subscription(
            PathMsg, self.get_parameter("local_plan_topic").value, self._on_local_plan, 20
        )
        self.create_subscription(
            PointStamped,
            self.get_parameter("lookahead_point_topic").value,
            self._on_lookahead_point,
            20,
        )

        # 窗口定时器：每 record_duration_sec 触发一次分析
        self._window_timer = self.create_timer(self.record_duration_sec, self._finish_window)
        self.get_logger().info(
            f"AI controller tuner recording {self.record_duration_sec:.1f}s windows; "
            f"stage={self.debug_stage}; logs: {self.log_path}"
        )
        self.get_logger().info(
            "Tuning mode: "
            f"stage={self.debug_stage}, "
            f"objective_name={self.objective_name}, "
            f"objective='{self._stage_objective()}', "
            f"target_speed_utilization={self.target_speed_utilization:.2f}, "
            f"active_parameters={self._active_tunable_parameters()}, "
            f"auto_apply={self.auto_apply}, "
            f"keep_best_observed={self.keep_best_observed}, "
            f"auto_restore_best={self.auto_restore_best}, "
            f"auto_stage_advance={self.auto_stage_advance}, "
            f"stage_sequence={self.stage_sequence}"
        )
        if self.ui_enabled:
            self._start_ui_server()

    def _reset_window(self) -> None:
        """重置当前窗口的所有数据缓冲区。"""
        self._window_started_at = time.time()
        self._tracking_errors: List[Tuple[float, float]] = []  # (timestamp, cross-track error)
        self._lookahead_distances: List[float] = []     # 前视距离序列
        self._cmd_nav: List[Tuple[float, float, float, float]] = []       # (timestamp, x, y, w)
        self._cmd_smoothed: List[Tuple[float, float, float, float]] = []  # (timestamp, x, y, w)
        self._actual_velocities: List[Tuple[float, float, float, float]] = []  # (timestamp, x, y, w)

    def _on_odom(self, msg: Odometry) -> None:
        """里程计回调：记录实际速度和位姿。

        当里程计报告速度为零但机器人实际上在移动时（Gazebo 常见的里程计 bug），
        通过位姿差分自行计算速度作为补充。
        """
        twist = msg.twist.twist
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1.0e-9
        pose = msg.pose.pose.position
        linear_speed = math.hypot(twist.linear.x, twist.linear.y)
        angular_speed = twist.angular.z
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        with self._lock:
            if self._latest_odom_sample is not None and linear_speed < 1.0e-6:
                last_stamp, last_x, last_y, last_yaw = self._latest_odom_sample
                dt = stamp - last_stamp
                if 0.001 <= dt <= 1.0:
                    linear_speed = math.hypot(pose.x - last_x, pose.y - last_y) / dt
                    angular_speed = angle_diff(yaw, last_yaw) / dt
            self._latest_odom_sample = (stamp, pose.x, pose.y, yaw)
            self._latest_odom = msg
            self._actual_velocities.append((stamp, linear_speed, 0.0, angular_speed))

    def _on_cmd_vel_nav(self, msg) -> None:
        """导航速度命令回调（兼容 Twist 和 TwistStamped 两种消息类型）。

        同时计算命令速度与实际速度之间的误差。
        """
        twist = msg.twist if hasattr(msg, "twist") else msg
        stamp = self._message_timestamp(msg)
        cmd = (stamp, twist.linear.x, twist.linear.y, twist.angular.z)
        with self._lock:
            self._cmd_nav.append(cmd)

    def _on_cmd_vel_smoothed(self, msg) -> None:
        """平滑后速度命令回调（用于观测速度平滑效果）。"""
        twist = msg.twist if hasattr(msg, "twist") else msg
        with self._lock:
            self._cmd_smoothed.append(
                (self._message_timestamp(msg), twist.linear.x, twist.linear.y, twist.angular.z)
            )

    def _on_local_plan(self, msg: PathMsg) -> None:
        """局部路径回调：计算最近路径点到机器人的距离作为追踪误差。"""
        if not msg.poses:
            return
        closest = min(
            math.hypot(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses
        )
        with self._lock:
            self._tracking_errors.append((self._message_timestamp(msg), closest))

    def _message_timestamp(self, msg: Any) -> float:
        """Use the message stamp when present; otherwise use the local ROS clock."""
        if hasattr(msg, "header"):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
            if stamp > 0.0:
                return stamp
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_lookahead_point(self, msg: PointStamped) -> None:
        """前视点回调：记录前视距离。"""
        with self._lock:
            self._lookahead_distances.append(math.hypot(msg.point.x, msg.point.y))

    def _finish_window(self) -> None:
        """窗口定时器回调：快照当前窗口数据，启动后台线程进行 AI 分析。

        如果上一轮分析尚未完成或调参已结束，则跳过。
        """
        with self._lock:
            if self._tuning_complete:
                return
            if self._analysis_running:
                self.get_logger().warn("Previous AI tuning request is still running; skipping window.")
                return
            self._analysis_running = True
            snapshot = {
                "stage": self.debug_stage,
                "window_id": self._window_id,
                "started_at": self._window_started_at,
                "finished_at": time.time(),
                "tracking_errors": list(self._tracking_errors),
                "lookahead_distances": list(self._lookahead_distances),
                "cmd_nav": list(self._cmd_nav),
                "cmd_smoothed": list(self._cmd_smoothed),
                "actual_velocities": list(self._actual_velocities),
            }
            self._window_id += 1
            self._reset_window()
            self._update_ui_state(ai_status="analyzing", last_error=None)
            self._add_dialog_event(
                "system",
                "Window Closed",
                "Collected one tuning window. Computing metrics now.",
                self._snapshot_counts(snapshot),
                stage=snapshot["stage"],
                window_id=snapshot["window_id"],
            )

        threading.Thread(target=self._analyze_and_tune, args=(snapshot,), daemon=True).start()

    def _analyze_and_tune(self, snapshot: Dict[str, Any]) -> None:
        """后台线程：分析窗口数据、评分、请求 AI 建议、应用参数。

        完整流程：
        1. 计算指标 → 2. 数据充足性检查 → 3. 评分 → 4. 更新最佳参数
        5. 必要时恢复最佳 → 6. 请求 AI 建议 → 7. 安全裁剪 → 8. 自动应用
        """
        result: Dict[str, Any] = {
            "event": "tuning_window",
            "stage": snapshot.get("stage"),
            "window_id": snapshot.get("window_id"),
            "snapshot": self._snapshot_counts(snapshot),
        }
        try:
            metrics = self._compute_metrics(snapshot, {})
            result["metrics"] = metrics

            if metrics["data_insufficient"]:
                self.get_logger().warn("Not enough data in this window; logging metrics without AI tuning.")
                self._add_dialog_event(
                    "error",
                    "Not Enough Data",
                    "Skipping AI tuning because this window did not collect enough controller data.",
                    metrics,
                    stage=snapshot.get("stage"),
                    window_id=snapshot.get("window_id"),
                )
                self._update_ui_state(
                    ai_status="waiting_for_data",
                    last_metrics=metrics,
                    last_error="not enough data in this window",
                )
                self._write_log(result)
                return

            current_params = self._get_controller_parameters()
            metrics = self._compute_metrics(snapshot, current_params)
            score = self._score_metrics(metrics)
            current_active_params = {
                name: current_params.get(name)
                for name in self._active_tunable_parameters()
                if name in current_params
            }
            self._update_ui_state(
                ai_status="scored",
                last_metrics=metrics,
                last_score=score,
                current_active_params=current_active_params,
            )
            self._log_window_summary(metrics, score, current_params)
            self._add_dialog_event(
                "system",
                "Metrics Ready",
                (
                    f"score={score:.4f}, tracking_rmse={metrics.get('tracking_error_rmse')}, "
                    f"actual_speed={metrics.get('actual_linear_mean')}, "
                    f"speed_utilization={metrics.get('speed_utilization')}, "
                    f"saturation={metrics.get('command_saturation_ratio')}"
                ),
                {
                    "active_params": current_active_params,
                    "best_score": self._best_score,
                },
                stage=snapshot.get("stage"),
                window_id=snapshot.get("window_id"),
            )
            best_update = self._update_best_observed(score, current_params, metrics)
            result["current_params"] = current_params
            result["metrics"] = metrics
            result["score"] = score
            result["best_observed"] = best_update

            if self._should_restore_best(best_update):
                result["apply_result"] = self._apply_parameters(self._best_params)
                result["apply_result"]["reason"] = "restored best observed parameters"
                result["stage_advance"] = self._maybe_advance_stage(
                    current_params=current_params,
                    safe_suggestions={},
                )
                self._non_improving_windows = 0
                self._update_ui_state(
                    ai_status="recording",
                    last_apply_result=result["apply_result"],
                )
                self._write_log(result)
                return

            self._update_ui_state(ai_status="requesting_ai")
            self._add_dialog_event(
                "ai",
                "Asking AI",
                "Sending metrics and current parameters to the AI API.",
                stage=snapshot.get("stage"),
                window_id=snapshot.get("window_id"),
            )
            ai_response = self._request_ai_suggestion(current_params, metrics)
            self._update_ui_state(ai_status="ai_received")
            suggestions = self._extract_suggestions(ai_response)
            ai_guard_reason = self._ai_change_guard(ai_response)
            safe_suggestions = (
                {}
                if ai_guard_reason
                else self._sanitize_suggestions(suggestions, current_params)
            )

            result["ai_response"] = ai_response
            result["safe_suggestions"] = safe_suggestions
            result["ai_guard_reason"] = ai_guard_reason
            self._update_ui_state(
                last_raw_suggestions=suggestions,
                last_safe_suggestions=safe_suggestions,
            )
            self._log_ai_suggestions(suggestions, safe_suggestions, current_params)
            self._add_dialog_event(
                "ai",
                "AI Suggestion",
                str(ai_response.get("reasoning", "AI returned a tuning suggestion.")),
                {
                    "risk_level": ai_response.get("risk_level"),
                    "action": ai_response.get("action"),
                    "confidence": ai_response.get("confidence"),
                    "guard_reason": ai_guard_reason,
                    "raw": suggestions,
                    "safe": safe_suggestions,
                },
                stage=snapshot.get("stage"),
                window_id=snapshot.get("window_id"),
            )

            if self.auto_apply and safe_suggestions:
                self._update_ui_state(ai_status="applying")
                result["apply_result"] = self._apply_parameters(safe_suggestions)
                self._add_dialog_event(
                    "apply",
                    "Parameters Applied",
                    "Applied sanitized AI suggestions to controller_server.",
                    result["apply_result"],
                    stage=snapshot.get("stage"),
                    window_id=snapshot.get("window_id"),
                )
            else:
                result["apply_result"] = {
                    "applied": False,
                    "reason": ai_guard_reason or "auto_apply disabled or empty suggestions",
                }
                self._add_dialog_event(
                    "system",
                    "No Apply",
                    result["apply_result"]["reason"],
                    stage=snapshot.get("stage"),
                    window_id=snapshot.get("window_id"),
                )
            result["stage_advance"] = self._maybe_advance_stage(
                current_params=current_params,
                safe_suggestions=safe_suggestions,
            )
            self._update_ui_state(
                ai_status="recording",
                last_apply_result=result["apply_result"],
            )

            self._write_log(result)
        except Exception as exc:  # noqa: BLE001 - ROS node should survive tuning failures.
            result["error"] = str(exc)
            self.get_logger().error(f"AI tuning window failed: {exc}")
            self._add_dialog_event(
                "error",
                "Tuning Failed",
                str(exc),
                stage=snapshot.get("stage"),
                window_id=snapshot.get("window_id"),
            )
            self._update_ui_state(ai_status="error", last_error=str(exc))
            self._write_log(result)
        finally:
            with self._lock:
                self._analysis_running = False
            if self._get_ui_state().get("ai_status") != "error":
                self._update_ui_state(ai_status="recording")

    def _snapshot_counts(self, snapshot: Dict[str, Any]) -> Dict[str, int]:
        """返回快照中各列表数据的样本计数。"""
        return {
            key: len(value) for key, value in snapshot.items() if isinstance(value, list)
        }

    def _compute_metrics(self, snapshot: Dict[str, Any], current_params: Dict[str, float]) -> Dict[str, Any]:
        """从窗口快照数据计算各项性能指标。

        包括追踪误差、速度误差、速度利用率、命令饱和率、振荡指标等。
        """
        # The robot executes the velocity-smoother output when available.  Measuring
        # against the upstream Nav2 command would reward changes that the smoother
        # subsequently removes.  Pair samples by timestamp rather than list index.
        effective_cmd = snapshot["cmd_smoothed"] or snapshot["cmd_nav"]
        actual = snapshot["actual_velocities"]
        max_pair_age_sec = 0.20
        active_command_speed = 0.03
        matched = []
        actual_index = 0
        for cmd_stamp, cmd_x, cmd_y, cmd_w in effective_cmd:
            while actual_index + 1 < len(actual) and actual[actual_index + 1][0] <= cmd_stamp:
                actual_index += 1
            candidates = actual[max(0, actual_index - 1):actual_index + 2]
            if not candidates:
                continue
            odom = min(candidates, key=lambda sample: abs(sample[0] - cmd_stamp))
            if abs(odom[0] - cmd_stamp) > max_pair_age_sec:
                continue
            if math.hypot(cmd_x, cmd_y) < active_command_speed and abs(cmd_w) < active_command_speed:
                continue
            matched.append(((cmd_stamp, cmd_x, cmd_y, cmd_w), odom))

        cmd_linear = [math.hypot(cmd[1], cmd[2]) for cmd, _ in matched]
        cmd_y = [cmd[2] for cmd, _ in matched]
        cmd_angular = [cmd[3] for cmd, _ in matched]
        actual_linear = [math.hypot(odom[1], odom[2]) for _, odom in matched]
        linear_errors = [command - measured for command, measured in zip(cmd_linear, actual_linear)]
        angular_errors = [cmd[3] - odom[3] for cmd, odom in matched]
        active_stamps = [cmd[0] for cmd, _ in matched]
        tracking_errors = [
            error for stamp, error in snapshot["tracking_errors"]
            if any(abs(stamp - command_stamp) <= max_pair_age_sec for command_stamp in active_stamps)
        ]

        max_linear = max(abs(float(current_params.get("v_linear_max", 0.0))), 1.0e-6)
        max_angular = max(abs(float(current_params.get("v_angular_max", 0.0))), 1.0e-6)
        actual_linear_mean = mean(actual_linear)
        speed_utilization = (
            min(max(actual_linear_mean / max_linear, 0.0), 1.5)
            if actual_linear_mean is not None
            else None
        )
        speed_shortfall = (
            max(self.target_speed_utilization - speed_utilization, 0.0)
            if speed_utilization is not None
            else None
        )
        saturated = [
            1.0
            for linear, angular in zip(cmd_linear, cmd_angular)
            if linear >= 0.95 * max_linear or abs(angular) >= 0.95 * max_angular
        ]

        sample_count = len(matched)
        return {
            "duration_sec": snapshot["finished_at"] - snapshot["started_at"],
            "sample_count": sample_count,
            "tracking_error_mean": mean(tracking_errors),
            "tracking_error_max": max(tracking_errors, default=None),
            "tracking_error_rmse": rmse(tracking_errors),
            "velocity_linear_error_mean": mean(linear_errors),
            "velocity_linear_error_rmse": rmse(linear_errors),
            "velocity_angular_error_mean": mean(angular_errors),
            "velocity_angular_error_rmse": rmse(angular_errors),
            "cmd_linear_mean": mean(cmd_linear),
            "actual_linear_mean": actual_linear_mean,
            "target_speed_utilization": self.target_speed_utilization,
            "speed_utilization": speed_utilization,
            "speed_shortfall": speed_shortfall,
            "angular_velocity_variance": variance(cmd_angular),
            "lateral_velocity_variance": variance(cmd_y),
            "linear_sign_changes": sign_changes(cmd_linear),
            "angular_sign_changes": sign_changes(cmd_angular),
            "mean_lookahead_distance": mean(snapshot["lookahead_distances"]),
            "command_saturation_ratio": len(saturated) / len(cmd_linear) if cmd_linear else None,
            "command_source": "cmd_vel_smoothed" if snapshot["cmd_smoothed"] else "cmd_vel_nav",
            "data_insufficient": sample_count < 10 or len(tracking_errors) < 3,
        }

    def _get_controller_parameters(self) -> Dict[str, float]:
        """从 Nav2 控制器节点读取当前参数值。

        同时读取可调参数和 v_linear_max/v_angular_max（用于计算速度利用率）。
        """
        names = sorted(set(self.tunable_parameters + ["v_linear_max", "v_angular_max"]))
        full_names = [self._full_parameter_name(name) for name in names]
        if not self._get_params_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"Parameter service is not available: {self.controller_node}/get_parameters")

        request = GetParameters.Request(names=full_names)
        future = self._get_params_client.call_async(request)
        response = self._wait_for_future(future, 5.0)
        values = [self._parameter_value_to_python(value) for value in response.values]
        return {
            name: value
            for name, value in zip(names, values)
            if isinstance(value, (int, float, bool))
        }

    def _request_ai_suggestion(self, current_params: Dict[str, float], metrics: Dict[str, Any]) -> Dict[str, Any]:
        """调用 AI API 获取参数调整建议。

        使用 OpenAI 兼容接口，发送系统提示词、当前参数和指标数据，
        要求 AI 返回 JSON 格式的调参建议。
        支持 mock_ai_response 用于离线测试。
        """
        if self.mock_ai_response:
            return json.loads(self.mock_ai_response)

        api_key = self.api_key or os.environ.get(self.api_key_env)
        if not api_key:
            raise RuntimeError(
                f"Missing API key. Set parameter api_key or environment variable {self.api_key_env} "
                "or use mock_ai_response for offline testing."
            )

        payload = {
            "model": self.model,
            "temperature": 0.1,
            "max_tokens": self.api_max_tokens,
            "messages": [
                {
                    "role": "system",
                    "content": (
                        "You are a conservative ROS2 Nav2 holonomic PID pure-pursuit tuning agent. "
                        "Tune only the current stage and only parameters in active_parameters. "
                        "Never change parameters belonging to another stage. "
                        "If data_insufficient is true, action must be hold and suggested_params must be empty. "
                        "If current_score is not better than best_score, prefer hold unless a small change is clearly justified. "
                        "Change at most max_changed_parameters_per_window parameters, and never exceed "
                        "max_parameter_step_ratio per parameter in one window. "
                        "Keep tracking error, velocity error, oscillation, saturation, overshoot, and final-pose stability low. "
                        "Do not trade a small speed gain for worse stability. "
                        "If confidence is below min_ai_confidence or risk is high, hold. "
                        "Do not undo parameters that were already tuned in earlier stages. "
                        "Return JSON only, with no markdown and no chain-of-thought. "
                        "Use this schema: {action: 'adjust'|'hold', suggested_params: {}, reasoning: 'short factual reason', "
                        "risk_level: 'low'|'medium'|'high', confidence: 0.0, expected_effect: 'short description', "
                        "rollback_condition: 'short condition'}. "
                        "When evidence is weak, return hold with an empty suggested_params object."
                    ),
                },
                {
                    "role": "user",
                    "content": json.dumps(
                        {
                            "debug_stage": self.debug_stage,
                            "objective_name": self.objective_name,
                            "stage_objective": self._stage_objective(),
                            "stage_index": (
                                self.stage_sequence.index(self.debug_stage) + 1
                                if self.debug_stage in self.stage_sequence
                                else 0
                            ),
                            "stage_total": len(self.stage_sequence),
                            "stage_round": self._stage_window_count,
                            "completed_windows_total": max(0, self._window_id - 1),
                            "active_parameters": self._active_tunable_parameters(),
                            "goal": (
                                "Tune for fast_stable behavior: maximize useful speed up to the target speed "
                                "utilization while keeping path RMSE, velocity error, oscillation, and command "
                                "saturation low. Improve only the active debug stage while preserving previously "
                                "tuned behavior."
                            ),
                            "current_params": current_params,
                            "metrics": metrics,
                            "current_score": self._score_metrics(metrics),
                            "best_score": self._best_score,
                            "score_delta_from_best": (
                                None
                                if self._best_score is None
                                else self._score_metrics(metrics) - self._best_score
                            ),
                            "data_insufficient": bool(metrics.get("data_insufficient")),
                            "score_weights": self.score_weights,
                            "target_speed_utilization": self.target_speed_utilization,
                            "max_parameter_step_ratio": self.max_parameter_step_ratio,
                            "max_changed_parameters_per_window": self.max_changed_parameters_per_window,
                            "min_ai_confidence": self.min_ai_confidence,
                            "best_observed": {
                                "score": self._best_score,
                                "params": self._best_params,
                                "metrics": self._best_metrics,
                            },
                            "allowed_parameter_limits": {
                                key: PARAMETER_LIMITS[key]
                                for key in self._active_tunable_parameters()
                                if key in PARAMETER_LIMITS
                            },
                        },
                        ensure_ascii=True,
                    ),
                },
            ],
        }
        if self.api_json_mode:
            payload["response_format"] = {"type": "json_object"}
        body = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            f"{self.api_base_url}/chat/completions",
            data=body,
            headers={
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.api_timeout_sec) as response:
                response_body = response.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"AI API HTTP {exc.code}: {detail}") from exc

        try:
            raw = json.loads(response_body)
        except json.JSONDecodeError as exc:
            preview = response_body[:500].replace("\n", "\\n")
            raise RuntimeError(f"AI API returned non-JSON response: {preview}") from exc

        message = raw["choices"][0].get("message", {})
        content = message.get("content", "")
        if isinstance(content, list):
            content = "".join(
                item.get("text", "") if isinstance(item, dict) else str(item)
                for item in content
            )
        if not str(content).strip():
            preview = json.dumps(raw, ensure_ascii=True)[:1000]
            self.get_logger().warn(
                f"AI response message content was empty; treating this window as no-change. Raw preview: {preview}"
            )
            return {
                "suggested_params": {},
                "reasoning": "AI response message content was empty; no parameter changes applied.",
                "risk_level": "unknown",
                "_raw_response": raw,
            }
        parsed = self._parse_json_content(content)
        parsed["_raw_content"] = content
        return parsed

    def _extract_suggestions(self, ai_response: Dict[str, Any]) -> Dict[str, Any]:
        """从 AI 响应中提取 suggested_params 字段。"""
        suggestions = ai_response.get("suggested_params", {})
        if not isinstance(suggestions, dict):
            raise RuntimeError("AI response suggested_params must be a JSON object.")
        return suggestions

    def _sanitize_suggestions(
        self, suggestions: Dict[str, Any], current_params: Dict[str, float]
    ) -> Dict[str, float]:
        """对 AI 建议做安全过滤和裁剪。

        - 忽略不在当前阶段可调范围内的参数
        - 忽略非数值类型的值
        - 将数值裁剪到 PARAMETER_LIMITS 范围内
        - 校验 lookahead 和 curvature 的 min/max 关系
        """
        safe: Dict[str, float] = {}
        active_parameters = self._active_tunable_parameters()
        for name, value in suggestions.items():
            if name not in active_parameters or name not in PARAMETER_LIMITS:
                self.get_logger().warn(f"Ignoring non-tunable suggested parameter: {name}")
                continue
            if name not in current_params or not isinstance(current_params[name], (int, float)):
                self.get_logger().warn(f"Ignoring suggestion without a readable current value: {name}")
                continue
            try:
                numeric_value = float(value)
            except (TypeError, ValueError):
                self.get_logger().warn(f"Ignoring non-numeric suggested value for {name}: {value}")
                continue
            low, high = PARAMETER_LIMITS[name]
            target_value = min(max(numeric_value, low), high)
            current_value = float(current_params[name])
            step_limit = max(
                abs(current_value) * self.max_parameter_step_ratio,
                (high - low) * 0.01,
            )
            bounded_delta = min(
                max(target_value - current_value, -step_limit), step_limit
            )
            bounded_value = min(max(current_value + bounded_delta, low), high)
            if abs(bounded_value - current_value) <= 1.0e-12:
                continue
            safe[name] = bounded_value
            if len(safe) >= self.max_changed_parameters_per_window:
                self.get_logger().info(
                    "Limiting AI suggestions to "
                    f"{self.max_changed_parameters_per_window} parameter(s) per window."
                )
                break

        merged = dict(current_params)
        merged.update(safe)
        if (
            "min_lookahead_dist" in merged
            and "max_lookahead_dist" in merged
            and merged["min_lookahead_dist"] > merged["max_lookahead_dist"]
        ):
            self.get_logger().warn("Ignoring lookahead suggestions because min_lookahead_dist > max_lookahead_dist.")
            safe.pop("min_lookahead_dist", None)
            safe.pop("max_lookahead_dist", None)
        if (
            "curvature_min" in merged
            and "curvature_max" in merged
            and merged["curvature_min"] >= merged["curvature_max"]
        ):
            self.get_logger().warn("Ignoring curvature suggestions because curvature_min >= curvature_max.")
            safe.pop("curvature_min", None)
            safe.pop("curvature_max", None)
        return safe

    def _ai_change_guard(self, ai_response: Dict[str, Any]) -> Optional[str]:
        """判断 AI 响应是否允许进入参数安全裁剪和应用流程。"""
        action = str(ai_response.get("action", "adjust")).strip().lower()
        if action in {"hold", "no_change", "no-change", "stop"}:
            return f"AI action={action}; holding current parameters"
        if str(ai_response.get("risk_level", "low")).strip().lower() == "high":
            return "AI risk_level=high; holding current parameters"
        confidence = ai_response.get("confidence")
        if confidence is not None:
            try:
                if float(confidence) < self.min_ai_confidence:
                    return (
                        f"AI confidence={float(confidence):.2f} below "
                        f"minimum={self.min_ai_confidence:.2f}"
                    )
            except (TypeError, ValueError):
                return "AI confidence is invalid; holding current parameters"
        return None

    def _log_window_summary(
        self, metrics: Dict[str, Any], score: float, current_params: Dict[str, float]
    ) -> None:
        """打印窗口评分和关键指标摘要日志。"""
        active_values = {
            name: current_params.get(name)
            for name in self._active_tunable_parameters()
            if name in current_params
        }
        self.get_logger().info(
            "Tuning window summary: "
            f"stage={self.debug_stage}, "
            f"score={score:.4f}, "
            f"samples={metrics.get('sample_count')}, "
            f"tracking_rmse={metrics.get('tracking_error_rmse')}, "
            f"linear_vel_rmse={metrics.get('velocity_linear_error_rmse')}, "
            f"angular_vel_rmse={metrics.get('velocity_angular_error_rmse')}, "
            f"actual_speed={metrics.get('actual_linear_mean')}, "
            f"speed_utilization={metrics.get('speed_utilization')}, "
            f"speed_shortfall={metrics.get('speed_shortfall')}, "
            f"saturation={metrics.get('command_saturation_ratio')}, "
            f"active_params={active_values}"
        )

    def _log_ai_suggestions(
        self,
        suggestions: Dict[str, Any],
        safe_suggestions: Dict[str, float],
        current_params: Dict[str, float],
    ) -> None:
        """打印 AI 建议的详细对比日志（原始值 vs 安全裁剪值 vs 旧值）。"""
        if not suggestions:
            self.get_logger().info(f"AI returned no parameter changes for stage={self.debug_stage}.")
            return
        self.get_logger().info(f"AI raw suggestions for stage={self.debug_stage}: {suggestions}")
        if not safe_suggestions:
            self.get_logger().warn("All AI suggestions were filtered out by stage/safety limits.")
            return
        for name, new_value in safe_suggestions.items():
            old_value = current_params.get(name)
            raw_value = suggestions.get(name)
            self.get_logger().info(
                "Parameter update candidate: "
                f"{self._full_parameter_name(name)} "
                f"old={old_value} raw={raw_value} safe={new_value}"
            )

    def _active_tunable_parameters(self) -> List[str]:
        """返回当前阶段允许调整的参数列表。

        取阶段预设参数、用户配置的 tunable_parameters、PARAMETER_LIMITS 三者的交集。
        """
        stage_parameters = STAGE_PROFILES.get(self.debug_stage, STAGE_PROFILES["custom"])["parameters"]
        configured = set(self.tunable_parameters)
        return [name for name in stage_parameters if name in configured and name in PARAMETER_LIMITS]

    def _stage_objective(self) -> str:
        """返回当前阶段的调优目标描述文本（发送给 AI）。"""
        return STAGE_PROFILES.get(self.debug_stage, STAGE_PROFILES["custom"])["objective"]

    def _score_metrics(self, metrics: Dict[str, Any]) -> float:
        """计算综合评分（越低越好）。

        评分 = 各项指标 × 对应权重的加权和。
        指标包括：追踪 RMSE、线/角速度误差、振荡方差、命令饱和率、
        速度缺口、符号变化次数。
        """
        sample_count = max(float(metrics.get("sample_count") or 1.0), 1.0)

        def value(name: str) -> float:
            metric = metrics.get(name)
            return float(metric) if metric is not None else 0.0

        return (
            self.score_weights["tracking"] * value("tracking_error_rmse")
            + self.score_weights["linear_error"] * abs(value("velocity_linear_error_rmse"))
            + self.score_weights["angular_error"] * abs(value("velocity_angular_error_rmse"))
            + self.score_weights["lateral_oscillation"] * value("lateral_velocity_variance")
            + self.score_weights["angular_oscillation"] * value("angular_velocity_variance")
            + self.score_weights["saturation"] * value("command_saturation_ratio")
            + self.score_weights["speed_shortfall"] * value("speed_shortfall")
            + self.score_weights["sign_change"] * value("linear_sign_changes") / sample_count
            + self.score_weights["sign_change"] * value("angular_sign_changes") / sample_count
        )

    def _update_best_observed(
        self, score: float, current_params: Dict[str, float], metrics: Dict[str, Any]
    ) -> Dict[str, Any]:
        """更新当前阶段的最佳观测参数。

        如果当前评分优于历史最佳，记录为新的最佳参数；
        否则递增连续未改进窗口计数。
        """
        self._stage_window_count += 1
        if not self.keep_best_observed:
            self._update_ui_state(
                best_score=None,
                best_params={},
                best_metrics={},
                stage_window_count=self._stage_window_count,
            )
            return {"enabled": False, "stage_window_count": self._stage_window_count}

        active_params = self._active_tunable_parameters()
        current_active_params = {
            name: float(current_params[name])
            for name in active_params
            if name in current_params and isinstance(current_params[name], (int, float))
        }
        improved = self._best_score is None or score < self._best_score
        if improved:
            self._best_score = score
            self._best_params = current_active_params
            self._best_metrics = dict(metrics)
            self._non_improving_windows = 0
            self.get_logger().info(f"New best observed score for stage {self.debug_stage}: {score:.4f}")
        else:
            self._non_improving_windows += 1
        self._update_ui_state(
            best_score=self._best_score,
            best_params=self._best_params,
            best_metrics=self._best_metrics,
            non_improving_windows=self._non_improving_windows,
            stage_window_count=self._stage_window_count,
        )

        return {
            "enabled": True,
            "improved": improved,
            "best_score": self._best_score,
            "best_params": self._best_params,
            "non_improving_windows": self._non_improving_windows,
            "stage_window_count": self._stage_window_count,
        }

    def _should_restore_best(self, best_update: Dict[str, Any]) -> bool:
        """判断是否应该恢复到最佳参数。

        条件：auto_restore_best 启用 + 存在最佳参数 + 连续未改进窗口数达到阈值。
        """
        if not self.auto_restore_best or not best_update.get("enabled"):
            return False
        if not self._best_params or best_update.get("improved"):
            return False
        return self._non_improving_windows >= self.restore_after_non_improving_windows

    def _maybe_advance_stage(
        self,
        current_params: Dict[str, float],
        safe_suggestions: Dict[str, float],
    ) -> Dict[str, Any]:
        """判断是否应该推进到下一个调参阶段。

        推进条件：
        - auto_stage_advance 启用
        - 已完成足够的窗口数（>= stage_min_windows）
        - 连续未改进窗口数达到阈值

        AI 在稳定阶段仍可能重复返回建议，不能用“建议非空”阻止阶段推进。
        阶段切换前会恢复该阶段观测到的最佳参数，避免最后一次建议覆盖最佳结果。
        """
        if not self.auto_stage_advance:
            return {"advanced": False, "reason": "auto_stage_advance disabled"}
        if self.debug_stage not in self.stage_sequence:
            return {"advanced": False, "reason": f"stage {self.debug_stage} is not in stage_sequence"}
        current_index = self.stage_sequence.index(self.debug_stage)
        if current_index >= len(self.stage_sequence) - 1:
            if self._should_complete_final_stage():
                restore_result = self._restore_best_before_completion(current_params)
                self._complete_tuning(restore_result)
                return {
                    "advanced": False,
                    "completed": True,
                    "reason": "final stage stable",
                    "restore_result": restore_result,
                }
            return {"advanced": False, "reason": "already at final stage"}
        if self._stage_window_count < self.stage_min_windows:
            return {
                "advanced": False,
                "reason": "stage_min_windows not reached",
                "stage_window_count": self._stage_window_count,
            }
        if self._non_improving_windows < self.stage_advance_after_non_improving_windows:
            return {
                "advanced": False,
                "reason": "non-improving window threshold not reached",
                "non_improving_windows": self._non_improving_windows,
            }

        from_stage = self.debug_stage
        to_stage = self.stage_sequence[current_index + 1]
        completed_windows = self._stage_window_count
        completed_non_improving = self._non_improving_windows
        restore_result: Optional[Dict[str, Any]] = None
        if self.apply_best_on_stage_advance and self._best_params:
            active_best = {
                name: value
                for name, value in self._best_params.items()
                if name in self._active_tunable_parameters()
            }
            current_active = {
                name: current_params.get(name)
                for name in active_best
                if name in current_params
            }
            if active_best and any(
                abs(float(current_active.get(name, value)) - float(value)) > 1.0e-9
                for name, value in active_best.items()
            ):
                restore_result = self._apply_parameters(active_best)
                restore_result["reason"] = "applied best observed parameters before stage advance"

        self._set_stage(to_stage)
        self.get_logger().info(
            f"Auto stage advance: {from_stage} -> {to_stage}; "
            f"windows={completed_windows}, non_improving={completed_non_improving}"
        )
        result = {
            "advanced": True,
            "from_stage": from_stage,
            "to_stage": to_stage,
            "stage_window_count": completed_windows,
            "non_improving_windows": completed_non_improving,
            "restore_result": restore_result,
        }
        self._add_dialog_event(
            "system",
            "Stage Advanced",
            f"{from_stage} -> {to_stage}",
            result,
        )
        return result

    def _should_complete_final_stage(self) -> bool:
        """判断最后阶段是否已达到稳定（可以结束调参）。"""
        if not self.stop_after_final_stage_stable:
            return False
        if self._stage_window_count < self.stage_min_windows:
            return False
        return self._non_improving_windows >= self.stage_advance_after_non_improving_windows

    def _restore_best_before_completion(
        self, current_params: Dict[str, float]
    ) -> Optional[Dict[str, Any]]:
        """调参完成前恢复到本阶段最佳参数（如果当前参数偏离了最佳值）。"""
        if not self.apply_best_on_stage_advance or not self._best_params:
            return None
        active_best = {
            name: value
            for name, value in self._best_params.items()
            if name in self._active_tunable_parameters()
        }
        if not active_best:
            return None
        if not any(
            abs(float(current_params.get(name, value)) - float(value)) > 1.0e-9
            for name, value in active_best.items()
        ):
            return None
        result = self._apply_parameters(active_best)
        result["reason"] = "applied best observed parameters before completing tuning"
        return result

    def _complete_tuning(self, restore_result: Optional[Dict[str, Any]]) -> None:
        """标记调参完成：停止窗口定时器，更新 UI 状态。"""
        self._tuning_complete = True
        if hasattr(self, "_window_timer"):
            self._window_timer.cancel()
        self.get_logger().info(
            f"Tuning complete at final stage={self.debug_stage}; "
            f"best_score={self._best_score}, best_params={self._best_params}"
        )
        details = {
            "best_score": self._best_score,
            "best_params": self._best_params,
            "best_metrics": self._best_metrics,
            "restore_result": restore_result,
        }
        self._update_ui_state(ai_status="complete", tuning_complete=True)
        # Publish a latched completion marker so goal_sender cancels its active
        # goal and exits instead of waiting forever for another stage update.
        self._publish_current_stage()
        self._add_dialog_event(
            "apply",
            "Tuning Complete",
            "Final stage is stable; stopped automatic tuning windows.",
            details,
        )

    def _set_stage(self, stage: str) -> None:
        """切换到新的调参阶段：重置所有阶段相关状态并发布阶段话题。"""
        self.debug_stage = stage
        self._best_score = None
        self._best_params = {}
        self._best_metrics = {}
        self._non_improving_windows = 0
        self._stage_window_count = 0
        self._reset_window()
        self._publish_current_stage()
        self._update_ui_state(
            stage=self.debug_stage,
            objective=self._stage_objective(),
            active_parameters=self._active_tunable_parameters(),
            ai_status="recording",
            last_metrics=None,
            last_score=None,
            current_active_params={},
            best_score=None,
            best_params={},
            best_metrics={},
            non_improving_windows=0,
            stage_window_count=0,
            last_raw_suggestions={},
            last_safe_suggestions={},
            last_apply_result={},
            last_error=None,
        )

    def _publish_current_stage(self) -> None:
        """发布当前调参阶段到阶段话题（供 goal_sender 等节点订阅）。"""
        if not hasattr(self, "_stage_publisher"):
            return
        msg = String()
        msg.data = "complete" if self._tuning_complete else self.debug_stage
        self._stage_publisher.publish(msg)

    def _apply_parameters(self, suggestions: Dict[str, float]) -> Dict[str, Any]:
        """将参数写入 Nav2 控制器节点。

        通过 SetParameters 服务将参数以 plugin_name.parameter_name 格式写入。
        """
        if not self._set_params_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"Parameter service is not available: {self.controller_node}/set_parameters")

        self.get_logger().info(
            f"Applying {len(suggestions)} parameter(s) to {self.controller_node} "
            f"for stage={self.debug_stage}: {suggestions}"
        )

        request = SetParameters.Request()
        request.parameters = [
            ParameterMsg(
                name=self._full_parameter_name(name),
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)),
            )
            for name, value in suggestions.items()
        ]
        future = self._set_params_client.call_async(request)
        response = self._wait_for_future(future, 5.0)
        results = [
            {"successful": result.successful, "reason": result.reason}
            for result in response.results
        ]
        all_successful = all(result["successful"] for result in results)
        for name, value, set_result in zip(suggestions.keys(), suggestions.values(), results):
            if set_result["successful"]:
                self.get_logger().info(
                    f"Applied {self._full_parameter_name(name)}={value}"
                )
            else:
                self.get_logger().error(
                    f"Rejected {self._full_parameter_name(name)}={value}: {set_result['reason']}"
                )
        self.get_logger().info(f"Parameter apply complete: successful={all_successful}")
        return {"applied": all_successful, "parameters": suggestions, "results": results}

    def _full_parameter_name(self, parameter_name: str) -> str:
        """构造完整的 ROS 参数名：plugin_name.parameter_name。"""
        return f"{self.plugin_name}.{parameter_name}"

    def _wait_for_future(self, future: Any, timeout_sec: float) -> Any:
        """在非回调上下文中同步等待 ROS Future 完成。"""
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout_sec):
            raise TimeoutError("Timed out waiting for ROS service response.")
        return future.result()

    def _parameter_value_to_python(self, value: ParameterValue) -> Any:
        """将 ROS ParameterValue 转换为 Python 原生类型。"""
        if value.type == ParameterType.PARAMETER_BOOL:
            return value.bool_value
        if value.type == ParameterType.PARAMETER_INTEGER:
            return value.integer_value
        if value.type == ParameterType.PARAMETER_DOUBLE:
            return value.double_value
        if value.type == ParameterType.PARAMETER_STRING:
            return value.string_value
        return None

    def _parse_json_content(self, content: str) -> Dict[str, Any]:
        """解析 AI 返回的内容为 JSON。

        处理常见的格式问题：markdown 代码块包裹、多余的文本前缀/后缀。
        """
        stripped = str(content).strip()
        if not stripped:
            raise RuntimeError("AI response message content was empty.")
        if stripped.startswith("```"):
            lines = stripped.splitlines()
            stripped = "\n".join(lines[1:-1]).strip()
            if stripped.startswith("json"):
                stripped = stripped[4:].strip()
        try:
            return json.loads(stripped)
        except json.JSONDecodeError as exc:
            match = re.search(r"\{.*\}", stripped, flags=re.DOTALL)
            if match:
                try:
                    return json.loads(match.group(0))
                except json.JSONDecodeError:
                    pass
            preview = stripped[:500].replace("\n", "\\n")
            raise RuntimeError(f"AI response content was not valid JSON: {preview}") from exc

    def _write_log(self, record: Dict[str, Any]) -> None:
        """将调参记录追加写入 JSONL 日志文件。"""
        record["logged_at"] = time.time()
        with self.log_path.open("a", encoding="utf-8") as log_file:
            log_file.write(json.dumps(record, ensure_ascii=True) + "\n")

    def _update_ui_state(self, **kwargs: Any) -> None:
        """线程安全地更新 Web UI 共享状态。"""
        with self._ui_state_lock:
            self._ui_state.update(kwargs)
            self._ui_state["updated_at"] = time.time()

    def _add_dialog_event(
        self,
        kind: str,
        title: str,
        message: str,
        details: Optional[Dict[str, Any]] = None,
        stage: Optional[str] = None,
        window_id: Optional[int] = None,
    ) -> None:
        """向 Web UI 事件流中添加一条事件（保留最近 400 条）。"""
        event = {
            "time": time.time(),
            "kind": kind,
            "title": title,
            "message": message,
            "details": details or {},
            "stage": stage or self.debug_stage,
            "window_id": window_id,
        }
        with self._ui_state_lock:
            self._dialog_events.append(event)
            self._dialog_events = self._dialog_events[-400:]
            self._ui_state["dialog_events"] = list(self._dialog_events)
            self._ui_state["updated_at"] = time.time()

    def _get_ui_state(self) -> Dict[str, Any]:
        """构建发送给 Web UI 的完整状态快照（包含当前窗口的实时数据）。"""
        with self._lock:
            try:
                stage_index = self.stage_sequence.index(self.debug_stage) + 1
            except ValueError:
                stage_index = 0
            stage_total = len(self.stage_sequence)
            completed_windows_total = max(0, self._window_id - 1)
            current_stage_round = self._stage_window_count + 1
            current_window = {
                "window_id": self._window_id,
                "stage": self.debug_stage,
                "stage_round": current_stage_round,
                "stage_windows_completed": self._stage_window_count,
                "elapsed_sec": time.time() - self._window_started_at,
                "tracking_errors": len(self._tracking_errors),
                "lookahead_distances": len(self._lookahead_distances),
                "cmd_nav": len(self._cmd_nav),
                "cmd_smoothed": len(self._cmd_smoothed),
                "actual_velocities": len(self._actual_velocities),
            }
        with self._ui_state_lock:
            state = self._compact_for_ui(self._ui_state)
            state["dialog_events"] = [
                self._compact_for_ui(event) for event in self._dialog_events[-200:]
            ]
        # current_window is sampled on every request; updated_at must follow it
        # instead of remaining frozen at the last event/analysis update.
        state["updated_at"] = time.time()
        state["progress"] = {
            "stage_index": stage_index,
            "stage_total": stage_total,
            "current_stage_round": current_stage_round,
            "stage_windows_completed": self._stage_window_count,
            "completed_windows_total": completed_windows_total,
            "current_window_id": self._window_id,
            "non_improving_windows": self._non_improving_windows,
            "tuning_complete": self._tuning_complete,
        }
        state["current_window"] = current_window
        return state

    def _compact_for_ui(self, value: Any, depth: int = 0) -> Any:
        """递归截断/省略数据，避免 UI 传输过大负载。

        限制：深度 > 6 截断为字符串、列表最多保留 40 项、字符串最多 2500 字符、
        _raw_response/_raw_content 等大字段直接省略。
        """
        if depth > 6:
            return str(value)[:300]
        if isinstance(value, dict):
            compact: Dict[str, Any] = {}
            for key, item in value.items():
                if str(key) in {"_raw_response", "_raw_content", "reasoning_content"}:
                    compact[str(key)] = "<omitted from UI>"
                    continue
                compact[str(key)] = self._compact_for_ui(item, depth + 1)
            return compact
        if isinstance(value, list):
            if len(value) > 40:
                return [self._compact_for_ui(item, depth + 1) for item in value[-40:]]
            return [self._compact_for_ui(item, depth + 1) for item in value]
        if isinstance(value, str):
            if len(value) > 2500:
                return value[:2500] + "... <truncated>"
            return value
        try:
            json.dumps(value)
            return value
        except TypeError:
            return str(value)

    def _make_ui_handler(self):
        """创建 Web UI 的 HTTP 请求处理器类。

        GET /status → 返回 JSON 状态快照
        GET / → 返回 HTML 页面
        """
        node = self

        class UiHandler(BaseHTTPRequestHandler):
            def do_GET(self):  # noqa: N802
                try:
                    if self.path == "/status":
                        body = json.dumps(node._get_ui_state(), ensure_ascii=True).encode("utf-8")
                        self.send_response(200)
                        self.send_header("Content-Type", "application/json")
                        self.send_header("Cache-Control", "no-store")
                        self.send_header("Content-Length", str(len(body)))
                        self.end_headers()
                        self.wfile.write(body)
                        return
                    body = UI_HTML.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                except Exception as exc:  # noqa: BLE001 - keep UI endpoint alive.
                    node.get_logger().error(f"Tuner UI request failed: {exc}")
                    body = json.dumps({"error": str(exc)}, ensure_ascii=True).encode("utf-8")
                    self.send_response(500)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)

            def log_message(self, format, *args):  # noqa: A002
                return

        return UiHandler

    def _start_ui_server(self) -> None:
        """启动 Web UI HTTP 服务器（后台守护线程）。"""
        self._update_ui_state(
            ui_listening=False,
            ui_url=None,
            ui_error=None,
            ui_host=self.ui_host,
            ui_port=self.ui_port,
        )
        try:
            self._ui_server = ThreadingHTTPServer(
                (self.ui_host, self.ui_port), self._make_ui_handler()
            )
        except OSError as exc:
            if not self.ui_fallback_to_ephemeral_port or self.ui_port == 0:
                self._update_ui_state(ui_error=str(exc))
                self.get_logger().error(
                    f"Failed to start tuner UI on {self.ui_host}:{self.ui_port}: {exc}"
                )
                return
            self.get_logger().warning(
                f"Tuner UI port {self.ui_host}:{self.ui_port} is unavailable ({exc}); "
                "falling back to an ephemeral port."
            )
            try:
                self._ui_server = ThreadingHTTPServer(
                    (self.ui_host, 0), self._make_ui_handler()
                )
            except OSError as fallback_exc:
                self._update_ui_state(ui_error=str(fallback_exc))
                self.get_logger().error(
                    f"Failed to start tuner UI on fallback port: {fallback_exc}"
                )
                return

        bound_host, bound_port = self._ui_server.server_address[:2]
        bound_host = str(bound_host)
        bound_port = int(bound_port)
        display_host = bound_host
        if display_host in {"", "0.0.0.0", "::"}:
            display_host = "127.0.0.1"
        if ":" in display_host and not display_host.startswith("["):
            display_host = f"[{display_host}]"
        ui_url = f"http://{display_host}:{bound_port}"
        self._update_ui_state(
            ui_host=bound_host,
            ui_port=bound_port,
            ui_url=ui_url,
            ui_listening=True,
            ui_error=None,
        )
        self._ui_thread = threading.Thread(target=self._ui_server.serve_forever, daemon=True)
        self._ui_thread.start()
        self.get_logger().info(
            f"Tuner UI: {ui_url} (bound {bound_host}:{bound_port}, "
            f"configured {self.ui_host}:{self.ui_port})"
        )

    def destroy_node(self) -> bool:
        """清理资源：关闭 UI 服务器后调用父类销毁。"""
        if self._ui_server is not None:
            self._ui_server.shutdown()
            self._ui_server.server_close()
            self._ui_server = None
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    """入口函数：初始化 ROS 2 并启动 AI 调参节点。"""
    rclpy.init(args=args)
    node = AiControllerTuner()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
