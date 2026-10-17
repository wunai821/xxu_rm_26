#!/usr/bin/env python3
"""Offline MPPI report; accepts the 2026-09-30 JSON/NPY runs and new CSV runs."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


LEGACY_COLUMNS = ['time', 'x', 'y', 'odom_speed', 'smooth_vx', 'smooth_vy',
                  'path_error', 'collision_vx', 'collision_vy', 'nav_vx', 'nav_vy']
AGE_COLUMNS = ['tf_age', 'odom_age', 'plan_age', 'nav_age', 'smooth_age',
               'collision_age', 'scan_age']
COLUMNS = LEGACY_COLUMNS + AGE_COLUMNS + ['odom_vx', 'odom_vy']


def stats(values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return {'mean': float(values.mean()), 'p95': float(np.percentile(values, 95)),
            'max': float(values.max())}


def analyze_samples(a, columns, goal, max_gap=0.25, max_age=0.3, max_plan_age=2.0):
    if a.ndim != 2 or a.shape[1] != len(columns):
        raise ValueError('Sample shape does not match declared columns')
    c = {key: a[:, i] for i, key in enumerate(columns)}
    if not set(LEGACY_COLUMNS) <= c.keys():
        raise ValueError('Missing required sample columns')
    t = c['time']
    dt = np.diff(t)
    valid = np.isfinite(dt) & (dt > 0) & (dt <= max_gap)
    weights = np.where(valid, dt, 0.0)
    observed = float(weights.sum())
    result = {'samples': len(a), 'observed_seconds': observed,
              'sampled_seconds': float(t[-1] - t[0]) if len(t) > 1 else None,
              'invalid_or_gap_intervals': int((~valid).sum()),
              'speed': {}, 'age': {}, 'chain': {}}
    speeds = {'odom': c['odom_speed']}
    for stage in ['nav', 'smooth', 'collision']:
        speeds[stage] = np.hypot(c[stage + '_vx'], c[stage + '_vy'])

    def fresh(keys):
        mask = np.ones(len(t), dtype=bool)
        for key in keys:
            if key + '_age' in c:
                age = c[key + '_age']
                limit = max_plan_age if key == 'plan' else max_age
                mask &= np.isfinite(age) & (age >= 0) & (age <= limit)
        return mask

    for key, speed in speeds.items():
        result['speed'][key] = stats(speed[fresh([key])])
    e = c['path_error'][fresh(['tf', 'plan'])]
    e = e[np.isfinite(e)]
    result['tracking'] = stats(e)
    if len(e):
        result['tracking']['rms'] = float(np.sqrt(np.mean(e ** 2)))
    position_valid = np.isfinite(c['x']) & np.isfinite(c['y']) & fresh(['tf'])
    distance = np.hypot(c['x'] - goal[0], c['y'] - goal[1])
    result['end_error'] = float(distance[-1]) if len(a) and position_valid[-1] else None
    result['terminal_zone_seconds'] = float(weights[
        position_valid[:-1] & position_valid[1:] & (distance[:-1] <= 0.5)].sum())
    # Rebound after entering the 0.5 m zone, within contiguous valid samples.
    # This is map-frame motion, not proof of physical overshoot (SLAM can move).
    minimum = None
    rebound = None
    for i, d in enumerate(distance):
        if not position_valid[i] or (i and not valid[i - 1]):
            minimum = None
        if not position_valid[i]:
            continue
        if minimum is None and d <= 0.5:
            minimum = float(d)
            rebound = 0.0 if rebound is None else rebound
        if minimum is not None:
            rebound = max(rebound, float(d) - minimum)
            minimum = min(minimum, float(d))
    result['terminal_rebound_m'] = rebound
    for key in AGE_COLUMNS:
        if key in c:
            age = c[key]
            limit = max_plan_age if key == 'plan_age' else max_age
            result['age'][key] = {'stats': stats(age), 'threshold': limit, 'invalid_or_stale_samples':
                                 int((~np.isfinite(age) | (age < 0) | (age > limit)).sum())}
    for upstream, downstream in [('nav', 'smooth'), ('smooth', 'collision'),
                                 ('collision', 'odom')]:
        u, d = speeds[upstream], speeds[downstream]
        eligible = fresh([upstream, downstream]) & np.isfinite(u) & np.isfinite(d) & (u > 0.1)
        interval_mask = eligible[:-1] & eligible[1:]
        seconds = float(weights[interval_mask].sum())
        result['chain'][upstream + '_to_' + downstream] = {
            'eligible_seconds': seconds,
            'slower_fraction': float(weights[interval_mask & (d[:-1] < 0.8 * u[:-1])].sum() / seconds) if seconds else None,
            'stopped_fraction': float(weights[interval_mask & (d[:-1] < 0.01)].sum() / seconds) if seconds else None,
        }
    acceleration = np.linalg.norm(np.diff(a[:, [columns.index('smooth_vx'),
                                               columns.index('smooth_vy')]], axis=0), axis=1)
    accel_valid = valid & fresh(['smooth'])[:-1] & fresh(['smooth'])[1:]
    result['smooth_acceleration'] = stats(acceleration[accel_valid] / dt[accel_valid])
    return result


def load_samples(path):
    if path.suffix == '.npy':
        return np.load(path, allow_pickle=False), LEGACY_COLUMNS
    with path.open(newline='') as stream:
        reader = csv.reader(stream)
        columns = next(reader)
        rows = list(reader)
    return np.array(rows, dtype=float).reshape(-1, len(columns)), columns


def analyze_run(path, max_gap=0.25, max_age=0.3, max_plan_age=2.0):
    raw = json.loads(path.read_text())
    records = raw if isinstance(raw, list) else raw['legs']
    expected = None if isinstance(raw, list) else len(raw['goals'])
    legs = []
    for record in records:
        sample_path = path.parent / record.get('samples_file', f'{path.stem}_leg{record["leg"]}.npy')
        a, columns = load_samples(sample_path)
        metrics = analyze_samples(a, columns, record['goal'], max_gap, max_age, max_plan_age)
        legs.append({**record, 'metrics': metrics})
    successes = sum(r['status'] == 4 and not r.get('timeout', False) for r in legs)
    return {'run': path.parent.name if path.stem == 'run' and not isinstance(raw, list) else path.stem,
            'expected_goals': expected,
            'attempted_goals': len(legs), 'successes': successes,
            'error': None if isinstance(raw, list) else raw.get('error'),
            'complete': (successes == expected and not raw.get('error') and
                         not raw.get('cancellation_error')) if expected is not None else None,
            'successful_recorded_legs': successes == len(legs) and bool(legs),
            'observed_seconds': sum(r['metrics']['observed_seconds'] for r in legs),
            'legs': legs}


def markdown(report):
    lines = ['# MPPI 基准分析', '',
             '耗时采用正向且不超过阈值的采样间隔；缺测时间不计入。速度、误差分位数按样本统计。',
             '路径误差是 map 位姿到最新路径采样点的距离，不是定位真值误差。',
             '速度链使用同时采到的速度模：只能筛查损失，不能据此证明因果或端到端延迟。',
             '旧数据无消息时间戳，无法剔除陈旧消息，也无法确认路线目标总数。', '',
             '平滑降速：上游速度 >0.1 m/s 时，下游低于其 80% 的有效时间占比；停车阈值 0.01 m/s。', '',
             '| 运行 | 成功/尝试 | 有效观测秒 | 最大末端误差 m |',
             '|---|---:|---:|---:|']
    for run in report['runs']:
        ends = [r['metrics']['end_error'] for r in run['legs'] if r['metrics']['end_error'] is not None]
        end = f'{max(ends):.3f}' if ends else '—'
        lines.append(f'| {run["run"]} | {run["successes"]}/{run["attempted_goals"]} | {run["observed_seconds"]:.2f} | {end} |')
    for run in report['runs']:
        lines += ['', f'## {run["run"]}', '',
                  '| 段 | 路径 RMS/P95 m | MPPI/平滑/碰撞/里程计峰值 m/s | 终点 0.5 m 内秒 | 平滑降速占比 | 碰撞停车占比 |',
                  '|---|---:|---|---:|---:|---:|']
        for leg in run['legs']:
            m = leg['metrics']
            def fmt(value):
                return '—' if value is None else f'{value:.3f}'
            track = m['tracking'] or {}
            peaks = '/'.join(fmt((m['speed'][k] or {}).get('max')) for k in ['nav', 'smooth', 'collision', 'odom'])
            lines.append(f'| {leg["leg"]} | {fmt(track.get("rms"))}/{fmt(track.get("p95"))} | {peaks} | {m["terminal_zone_seconds"]:.2f} | {fmt(m["chain"]["nav_to_smooth"]["slower_fraction"])} | {fmt(m["chain"]["smooth_to_collision"]["stopped_fraction"])} |')
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path, help='Run JSON files')
    parser.add_argument('--output', required=True, type=Path, help='New report directory')
    parser.add_argument('--max-gap', type=float, default=0.25)
    parser.add_argument('--max-age', type=float, default=0.3)
    parser.add_argument('--max-plan-age', type=float, default=2.0, help='Separate age limit for slower global replanning')
    args = parser.parse_args()
    if any(not np.isfinite(v) or v <= 0 for v in [args.max_gap, args.max_age, args.max_plan_age]):
        parser.error('Thresholds must be positive')
    report = {'max_gap': args.max_gap, 'max_age': args.max_age, 'max_plan_age': args.max_plan_age,
              'runs': [analyze_run(p, args.max_gap, args.max_age, args.max_plan_age) for p in args.runs]}
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    (args.output / 'report.md').write_text(markdown(report))
    print(args.output / 'report.md')


if __name__ == '__main__':
    main()
