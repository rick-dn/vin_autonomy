#!/usr/bin/env python3
"""
score_run.py - Scores one run saved by record_run.py

Usage: python3 score_run.py square_01 [--start T] [--end T] [--rpe-dt 1.0]

Steps:
  1. Crop to the scored section: from the first OpenVINS pose (initialisation)
     to the disarm after landing (from events.csv). --start / --end override.
  2. Interpolate ground truth at each estimate time.
  3. Align the estimate to ground truth with yaw + translation (4 DoF). Gravity
     makes roll and pitch observable for VIO, so they are not aligned away.
     A separate similarity (Sim3) fit is done only to report the scale error.
  4. ATE: position and rotation error after alignment.
     RPE: error of the relative motion over --rpe-dt seconds (no alignment needed).

Writes metrics.json and score.png into the run folder and prints a summary.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

RUNS_DIR = Path(__file__).resolve().parents[3] / 'runs' / 'eval'


def load_tum(path):
    d = np.loadtxt(path, ndmin=2)
    return d[:, 0], d[:, 1:4], Rotation.from_quat(d[:, 4:8])


def disarm_time(events_path):
    """Time of the first disarm that follows an arm, or None."""
    was_armed = False
    with open(events_path) as f:
        for row in csv.DictReader(f):
            armed = row['armed'] == '1'
            if armed:
                was_armed = True
            elif was_armed:
                return float(row['t'])
    return None


def align_yaw(p_est, p_gt):
    """Yaw rotation and translation that best map p_est onto p_gt (least squares)."""
    me, mg = p_est.mean(axis=0), p_gt.mean(axis=0)
    e, g = p_est - me, p_gt - mg
    yaw = np.arctan2(np.sum(e[:, 0] * g[:, 1] - e[:, 1] * g[:, 0]),
                     np.sum(e[:, 0] * g[:, 0] + e[:, 1] * g[:, 1]))
    R = Rotation.from_euler('z', yaw)
    t = mg - R.apply(me)
    return R, t, yaw


def umeyama_scale(p_est, p_gt):
    """Scale of the best similarity transform p_gt ~ s R p_est + t (Umeyama 1991)."""
    me, mg = p_est.mean(axis=0), p_gt.mean(axis=0)
    e, g = p_est - me, p_gt - mg
    U, D, Vt = np.linalg.svd(g.T @ e / len(e))
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    var_e = np.mean(np.sum(e ** 2, axis=1))
    return np.trace(np.diag(D) @ S) / var_e


def rpe(t, p_est, R_est, p_gt, R_gt, dt):
    """Relative pose error over pairs of poses dt seconds apart."""
    j = np.searchsorted(t, t + dt)
    ok = j < len(t)
    i, j = np.nonzero(ok)[0], j[ok]
    if len(i) == 0:
        return np.array([]), np.array([])
    # relative motion in the frame of pose i
    d_est = R_est[i].inv().apply(p_est[j] - p_est[i])
    d_gt = R_gt[i].inv().apply(p_gt[j] - p_gt[i])
    dR_est = R_est[i].inv() * R_est[j]
    dR_gt = R_gt[i].inv() * R_gt[j]
    trans = np.linalg.norm(d_est - d_gt, axis=1)
    rot = np.degrees((dR_gt.inv() * dR_est).magnitude())
    return trans, rot


def rmse(x):
    return float(np.sqrt(np.mean(x ** 2)))


def main():
    parser = argparse.ArgumentParser(description='Score one recorded VIO run')
    parser.add_argument('name', help='run folder under runs/eval (or a path)')
    parser.add_argument('--start', type=float, help='start of scored section, sim time [s]')
    parser.add_argument('--end', type=float, help='end of scored section, sim time [s]')
    parser.add_argument('--rpe-dt', type=float, default=1.0, help='RPE interval [s], default 1.0')
    args = parser.parse_args()

    run = Path(args.name) if Path(args.name).is_dir() else RUNS_DIR / args.name
    if not run.is_dir():
        sys.exit(f'no run folder {run}')

    t_est, p_est, R_est = load_tum(run / 'estimate.tum')
    t_gt, p_gt_all, R_gt_all = load_tum(run / 'ground_truth.tum')

    # --- scored section
    t0 = args.start if args.start is not None else t_est[0]
    t1 = args.end
    if t1 is None:
        t1 = disarm_time(run / 'events.csv')
        if t1 is None:
            t1 = t_est[-1]
            print('no disarm in events.csv, scoring to the last estimate')
    keep = (t_est >= max(t0, t_gt[0])) & (t_est <= min(t1, t_gt[-1]))
    if keep.sum() < 10:
        sys.exit(f'only {keep.sum()} estimate poses in [{t0:.2f}, {t1:.2f}]')
    t, p_est, R_est = t_est[keep], p_est[keep], R_est[keep]

    # --- ground truth at estimate times
    p_gt = np.column_stack([np.interp(t, t_gt, p_gt_all[:, k]) for k in range(3)])
    R_gt = Slerp(t_gt, R_gt_all)(t)

    # --- alignment
    R_al, t_al, yaw = align_yaw(p_est, p_gt)
    p_al = R_al.apply(p_est) + t_al
    R_est_al = R_al * R_est
    scale = umeyama_scale(p_est, p_gt)

    # --- errors
    err = np.linalg.norm(p_al - p_gt, axis=1)
    err_rot = np.degrees((R_gt.inv() * R_est_al).magnitude())
    rpe_t, rpe_r = rpe(t, p_est, R_est, p_gt, R_gt, args.rpe_dt)
    path_len = float(np.sum(np.linalg.norm(np.diff(p_gt, axis=0), axis=1)))

    metrics = {
        'run': run.name,
        'scored_from_s': float(t[0]),
        'scored_to_s': float(t[-1]),
        'duration_s': float(t[-1] - t[0]),
        'num_poses': int(len(t)),
        'path_length_m': path_len,
        'align_yaw_deg': float(np.degrees(yaw)),
        'ate_rmse_m': rmse(err),
        'ate_mean_m': float(err.mean()),
        'ate_median_m': float(np.median(err)),
        'ate_max_m': float(err.max()),
        'ate_rot_rmse_deg': rmse(err_rot),
        'rpe_dt_s': args.rpe_dt,
        'rpe_trans_rmse_m': rmse(rpe_t) if len(rpe_t) else None,
        'rpe_rot_rmse_deg': rmse(rpe_r) if len(rpe_r) else None,
        'scale': float(scale),
        'scale_error_pct': float((scale - 1.0) * 100.0),
        'final_error_m': float(err[-1]),
        'final_error_pct_of_path': float(100.0 * err[-1] / path_len) if path_len > 0 else None,
    }
    with open(run / 'metrics.json', 'w') as f:
        json.dump(metrics, f, indent=2)

    # --- position 1-sigma from OpenVINS covariance, at estimate times
    sigma = None
    cov_path = run / 'estimate_cov.csv'
    if cov_path.exists():
        c = np.loadtxt(cov_path, delimiter=',', skiprows=1, ndmin=2)
        if len(c):
            s = np.sqrt(np.clip(c[:, 1:4].sum(axis=1), 0, None))
            sigma = np.interp(t, c[:, 0], s)

    # --- plot
    tr = t - t[0]
    fig, ax = plt.subplots(2, 2, figsize=(12, 9))
    a = ax[0, 0]
    a.plot(p_gt[:, 0], p_gt[:, 1], 'k-', label='ground truth')
    a.plot(p_al[:, 0], p_al[:, 1], 'C0-', label='OpenVINS (aligned)')
    a.plot(p_gt[0, 0], p_gt[0, 1], 'go', label='start')
    a.set_xlabel('x [m]'); a.set_ylabel('y [m]'); a.set_title('Top view')
    a.axis('equal'); a.grid(True); a.legend()

    a = ax[0, 1]
    a.plot(tr, p_gt[:, 2], 'k-', label='ground truth')
    a.plot(tr, p_al[:, 2], 'C0-', label='OpenVINS (aligned)')
    a.set_xlabel('t [s]'); a.set_ylabel('z [m]'); a.set_title('Height')
    a.grid(True); a.legend()

    a = ax[1, 0]
    a.plot(tr, err, 'C3-', label='position error')
    if sigma is not None:
        a.plot(tr, 3 * sigma, 'C7--', label='OpenVINS 3$\\sigma$')
    a.set_xlabel('t [s]'); a.set_ylabel('[m]'); a.set_title('Absolute trajectory error')
    a.grid(True); a.legend()

    a = ax[1, 1]
    a.plot(tr, err_rot, 'C2-')
    a.set_xlabel('t [s]'); a.set_ylabel('[deg]'); a.set_title('Rotation error')
    a.grid(True)

    fig.suptitle(f'{run.name}: ATE {metrics["ate_rmse_m"]:.3f} m RMSE, '
                 f'scale error {metrics["scale_error_pct"]:+.1f}%, '
                 f'path {path_len:.1f} m')
    fig.tight_layout()
    fig.savefig(run / 'score.png', dpi=110)

    # --- summary
    m = metrics
    print(f'run            {m["run"]}')
    print(f'scored         {m["scored_from_s"]:.2f} -> {m["scored_to_s"]:.2f} s sim '
          f'({m["duration_s"]:.1f} s, {m["num_poses"]} poses, path {m["path_length_m"]:.2f} m)')
    print(f'alignment yaw  {m["align_yaw_deg"]:.1f} deg')
    print(f'ATE  position  rmse {m["ate_rmse_m"]:.3f}  mean {m["ate_mean_m"]:.3f}  '
          f'median {m["ate_median_m"]:.3f}  max {m["ate_max_m"]:.3f} m')
    print(f'ATE  rotation  rmse {m["ate_rot_rmse_deg"]:.2f} deg')
    if m['rpe_trans_rmse_m'] is not None:
        print(f'RPE  {m["rpe_dt_s"]:.1f} s      {m["rpe_trans_rmse_m"]:.3f} m  {m["rpe_rot_rmse_deg"]:.2f} deg (rmse)')
    print(f'scale          {m["scale"]:.4f} ({m["scale_error_pct"]:+.2f}%)')
    print(f'final error    {m["final_error_m"]:.3f} m ({m["final_error_pct_of_path"]:.2f}% of path)')
    print(f'saved          {run / "metrics.json"}, {run / "score.png"}')


if __name__ == '__main__':
    main()
