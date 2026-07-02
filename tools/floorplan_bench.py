#!/usr/bin/env python3
"""Objective map-quality benchmark for ldim_to_floorplan.py.

Runs the same pipeline stages as the floorplan tool (strip -> orientation ->
assemble -> SLAM -> loop closure) and prints metrics that track SLAM quality:

  loop_t / loop_r   pre-correction loop-closure residual (odometry drift).
  tdc_med / tdc_p90 temporally-distant consistency: for each point, distance to
                    the nearest point observed >`gap` scans away. Directly
                    measures doubled walls (a 5 cm ghost reads ~5 cm here).
  thick_med         local PCA wall thickness: sqrt of the minor eigenvalue of
                    the covariance of neighbours within 15 cm, median over a
                    random sample. Crisper walls = thinner.
  cells / ipr       occupied wall cells after the render threshold+declutter
                    (fewer = crisper at equal coverage) and the straighten
                    inverse-participation score (higher = better aligned).

Usage:
  tools/.venv/bin/python tools/floorplan_bench.py recordings/scan_006.ldim [tool flags]
"""
import argparse
import math
import sys

import numpy as np
from scipy.spatial import cKDTree

import ldim_to_floorplan as fp


def tdc(scans, poses, gap=30, cap=0.5):
    pts, sid = [], []
    for i, (P, s) in enumerate(zip(poses, scans)):
        w = fp.apply(P, s)
        pts.append(w)
        sid.append(np.full(len(w), i))
    pts = np.vstack(pts)
    sid = np.concatenate(sid)
    kd = cKDTree(pts)
    # nearest temporally-distant neighbour: take k candidates, keep the first
    # one more than `gap` scans away in time
    d, idx = kd.query(pts, k=24, distance_upper_bound=cap)
    best = np.full(len(pts), cap)
    for k in range(1, d.shape[1]):
        ok = np.isfinite(d[:, k])
        far = np.zeros(len(pts), bool)
        far[ok] = np.abs(sid[idx[ok, k]] - sid[ok]) > gap
        take = far & (best >= cap)
        best[take] = d[take, k]
    valid = best < cap
    return float(np.median(best[valid])), float(np.percentile(best[valid], 90)), pts


def pca_thickness(pts, r=0.15, n=4000, seed=0):
    rng = np.random.default_rng(seed)
    kd = cKDTree(pts)
    samp = pts[rng.choice(len(pts), size=min(n, len(pts)), replace=False)]
    th = []
    for p in samp:
        nb = pts[kd.query_ball_point(p, r)]
        if len(nb) < 8:
            continue
        c = nb - nb.mean(0)
        lam = np.linalg.eigvalsh(c.T @ c / len(nb))
        th.append(math.sqrt(max(lam[0], 0.0)))
    return float(np.median(th))


def render_stats(pts, res=0.025, hits=2):
    xs, ys = -pts[:, 0], pts[:, 1]
    a = fp.straighten_angle(np.column_stack([xs, ys]))
    # recompute the IPR score at the chosen angle
    c, s = math.cos(a), math.sin(a)
    rx, ry = xs * c - ys * s, xs * s + ys * c
    hx, _ = np.histogram(rx, bins=max(1, int((rx.max() - rx.min()) / 0.05)))
    hy, _ = np.histogram(ry, bins=max(1, int((ry.max() - ry.min()) / 0.05)))
    ipr = ((hx / hx.sum()) ** 2).sum() + ((hy / hy.sum()) ** 2).sum()
    mnx, mny = rx.min(), ry.min()
    W = int((rx.max() - mnx) / res) + 1
    H = int((ry.max() - mny) / res) + 1
    grid = np.zeros((H, W), np.int32)
    np.add.at(grid, (((ry - mny) / res).astype(int), ((rx - mnx) / res).astype(int)), 1)
    from scipy.ndimage import label as _label
    lab, n = _label(grid >= 1)
    keep = np.zeros(n + 1, bool)
    keep[np.unique(lab[grid >= hits])] = True
    keep[0] = False
    occ = keep[lab]
    occ = fp.declutter(occ)
    return int(occ.sum()), float(ipr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("--no-kf", action="store_true")
    ap.add_argument("--kf-gate", type=float, default=0.03)
    ap.add_argument("--window", type=int, default=50)
    ap.add_argument("--keep-operator", action="store_true")
    ap.add_argument("--label", default="")
    # feature toggles under test (mirror the tool's flags as they are added)
    ap.add_argument("--p2p", action="store_true", help="point-to-point ICP (legacy)")
    ap.add_argument("--no-deskew-xy", action="store_true")
    ap.add_argument("--no-refine", action="store_true")
    args = ap.parse_args()

    packets, imu_ns, imu_wz, imu_data = fp.load_ldim(args.input)
    if not args.keep_operator:
        packets = fp.strip_operator(packets)
    if args.no_kf:
        get_rot = fp.build_orientation(imu_data)
    else:
        get_rot = fp.build_orientation_kf(imu_data, gate=args.kf_gate)
    yaw_at = lambda t: get_rot(t)[2]
    scans, times = fp.assemble_scans(packets, get_rot)

    kw = {}
    if hasattr(fp, "PLICP_DEFAULT"):
        kw["point_to_line"] = not args.p2p
    poses = fp.run_slam(scans, times, yaw_at, win=args.window, **kw)

    if not args.no_deskew_xy and hasattr(fp, "redeskew_translation"):
        scans, times = fp.redeskew_translation(packets, get_rot, poses, times)
        poses = fp.run_slam(scans, times, yaw_at, win=args.window, **kw)

    poses2, C = fp.close_loop(poses, scans, **kw)
    if C is None:
        loop_t, loop_r = float("nan"), float("nan")
    else:
        loop_t = math.hypot(C[0, 2], C[1, 2])
        loop_r = abs(math.degrees(fp.yaw_of(C)))
    poses = poses2

    if not args.no_refine and hasattr(fp, "refine_poses"):
        poses = fp.refine_poses(poses, scans)

    med, p90, pts = tdc(scans, poses)
    thick = pca_thickness(pts)
    cells, ipr = render_stats(pts)
    tag = args.label or "run"
    print(f"[{tag}] {args.input}")
    print(f"  loop_t={loop_t:.3f} m  loop_r={loop_r:.2f} deg")
    print(f"  tdc_med={med * 100:.2f} cm  tdc_p90={p90 * 100:.2f} cm")
    print(f"  thick_med={thick * 100:.2f} cm")
    print(f"  cells={cells}  ipr={ipr:.4f}")


if __name__ == "__main__":
    main()
