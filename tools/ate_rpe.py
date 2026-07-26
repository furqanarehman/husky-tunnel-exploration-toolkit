#!/usr/bin/env python3
"""
ate_rpe.py

Computes Absolute Trajectory Error (ATE) and Relative Pose Error (RPE)
between a ground-truth trajectory and an estimated trajectory, both
recorded by pose_recorder.py as CSVs with columns:
    t, x, y, z, qx, qy, qz, qw

Method (standard approach, same idea as the TUM RGB-D benchmark tools):
  1. Associate ground-truth and estimate samples by nearest timestamp
     (within --max-time-diff seconds).
  2. Umeyama-align the estimated trajectory onto ground truth (rotation +
     translation, fixed scale = 1) -- this is necessary because the
     estimate's map/odom frame origin and orientation don't necessarily
     match Gazebo's world frame.
  3. ATE = RMSE of per-point translational error after alignment.
  4. RPE = RMSE of translational error in relative motion over a fixed
     time delta (--rpe-delta seconds), which is less sensitive to a single
     bad alignment and better reflects local drift.

Usage:
    python3 ate_rpe.py ground_truth.csv lio_sam.csv
    python3 ate_rpe.py ground_truth.csv slam_toolbox.csv --rpe-delta 1.0
"""

import argparse
import csv

import numpy as np


def load_csv(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append((
                float(row["t"]),
                np.array([float(row["x"]), float(row["y"]), float(row["z"])]),
            ))
    rows.sort(key=lambda r: r[0])
    return rows


def associate(gt, est, max_time_diff):
    """Nearest-timestamp association. Returns list of (gt_xyz, est_xyz) pairs."""
    est_times = np.array([t for t, _ in est])
    pairs = []
    for t_gt, p_gt in gt:
        idx = np.searchsorted(est_times, t_gt)
        candidates = [i for i in (idx - 1, idx) if 0 <= i < len(est)]
        if not candidates:
            continue
        best = min(candidates, key=lambda i: abs(est_times[i] - t_gt))
        if abs(est_times[best] - t_gt) <= max_time_diff:
            pairs.append((t_gt, p_gt, est[best][1]))
    return pairs


def umeyama_alignment(src, dst):
    """
    Find R, t such that dst ~= R @ src + t (fixed scale = 1), minimizing
    sum of squared errors. src, dst: Nx3 arrays.
    """
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    src_c = src - mu_src
    dst_c = dst - mu_dst

    cov = dst_c.T @ src_c / len(src)
    U, S, Vt = np.linalg.svd(cov)
    d = np.sign(np.linalg.det(U @ Vt))
    D = np.diag([1, 1, d])
    R = U @ D @ Vt
    t = mu_dst - R @ mu_src
    return R, t


def compute_ate(pairs):
    gt_pts = np.array([p_gt for _, p_gt, _ in pairs])
    est_pts = np.array([p_est for _, _, p_est in pairs])

    R, t = umeyama_alignment(est_pts, gt_pts)
    est_aligned = (R @ est_pts.T).T + t

    errors = np.linalg.norm(gt_pts - est_aligned, axis=1)
    rmse = np.sqrt(np.mean(errors ** 2))
    return rmse, errors, R, t


def compute_rpe(pairs, delta_s, R):
    """
    RPE over a fixed time delta: compare the relative motion (translation)
    between two timestamps ~delta_s apart, in ground truth vs. estimate.

    The estimate's relative-motion vectors are rotated by the SAME
    alignment rotation R used for ATE before comparing -- relative motion
    is invariant to a fixed global TRANSLATION (it cancels out
    automatically when differencing two points), but NOT invariant to a
    fixed global ROTATION between the two trajectories' frames, so that
    part of the alignment must still be applied here, or a fixed frame
    misalignment gets miscounted as drift error.
    """
    times = np.array([t for t, _, _ in pairs])
    gt_pts = np.array([p_gt for _, p_gt, _ in pairs])
    est_pts = np.array([p_est for _, _, p_est in pairs])

    errors = []
    for i in range(len(pairs)):
        # Find the first later sample at least delta_s ahead.
        j_candidates = np.where(times >= times[i] + delta_s)[0]
        if len(j_candidates) == 0:
            continue
        j = j_candidates[0]

        gt_rel = gt_pts[j] - gt_pts[i]
        est_rel_raw = est_pts[j] - est_pts[i]
        est_rel = R @ est_rel_raw
        errors.append(np.linalg.norm(gt_rel - est_rel))

    if not errors:
        return None, []
    errors = np.array(errors)
    rmse = np.sqrt(np.mean(errors ** 2))
    return rmse, errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ground_truth_csv")
    parser.add_argument("estimate_csv")
    parser.add_argument("--max-time-diff", type=float, default=0.05,
                         help="Max seconds between associated samples (default 0.05)")
    parser.add_argument("--rpe-delta", type=float, default=1.0,
                         help="Time delta in seconds for RPE (default 1.0)")
    args = parser.parse_args()

    gt = load_csv(args.ground_truth_csv)
    est = load_csv(args.estimate_csv)
    print(f"Ground truth samples: {len(gt)}")
    print(f"Estimate samples:     {len(est)}")

    pairs = associate(gt, est, args.max_time_diff)
    print(f"Associated pairs (within {args.max_time_diff}s): {len(pairs)}")
    if len(pairs) < 10:
        print("Too few associated pairs -- check that both CSVs cover the "
              "same time range and use the same clock (sim time vs. wall "
              "time), and that --max-time-diff is reasonable for your "
              "sampling rates.")
        return

    ate_rmse, ate_errors, R, t = compute_ate(pairs)
    print(f"\nATE (RMSE after alignment): {ate_rmse:.4f} m")
    print(f"  mean={np.mean(ate_errors):.4f}  median={np.median(ate_errors):.4f}  "
          f"max={np.max(ate_errors):.4f}  min={np.min(ate_errors):.4f}")

    rpe_rmse, rpe_errors = compute_rpe(pairs, args.rpe_delta, R)
    if rpe_rmse is not None:
        print(f"\nRPE (RMSE over {args.rpe_delta}s intervals): {rpe_rmse:.4f} m")
        print(f"  mean={np.mean(rpe_errors):.4f}  median={np.median(rpe_errors):.4f}  "
              f"max={np.max(rpe_errors):.4f}  min={np.min(rpe_errors):.4f}  "
              f"n={len(rpe_errors)}")
    else:
        print(f"\nRPE: no pairs found with {args.rpe_delta}s separation -- "
              f"try a smaller --rpe-delta or check the trajectory duration.")


if __name__ == "__main__":
    main()
