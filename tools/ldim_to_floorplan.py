#!/usr/bin/env python3
"""Build a 2D floorplan PNG directly from a .ldim recording.

This is a self-contained pure-Python pipeline (no ROS, no mcap) that fixes the
two things that wreck a naive overlay of the raw scans:

  1. Motion deskew. The LD19 spins at ~10 Hz; while it completes one 360°
     revolution the rig itself rotates (up to ~45 deg/s in scan_003). Each
     packet in the .ldim carries its own timestamp, so we rotate every packet's
     points by the gyro-integrated heading change relative to the scan's
     reference time. This removes the ~4-5 deg/scan skew that otherwise
     accumulates into runaway yaw drift.

  2. Drift + loop closure. A scan-to-(sliding-window local map) ICP, seeded with
     the IMU yaw as a per-scan prior, tracks the rig. Residual drift is removed
     by matching the final scans back onto the starting room (the capture starts
     and ends in the same place) and distributing that loop-closure correction
     across the trajectory, weighted by how much the rig was moving/turning at
     each step.

Usage:
  tools/.venv/bin/python tools/ldim_to_floorplan.py recordings/scan_003.ldim
  tools/.venv/bin/python tools/ldim_to_floorplan.py recordings/scan_003.ldim \
      -o scan_003_floorplan.png --res 0.025 --no-loop
"""
import argparse
import math
import struct
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from scipy.spatial import cKDTree
    from scipy.linalg import logm, expm
except ImportError:
    sys.exit("missing deps — pip install numpy scipy matplotlib")

MAGIC = 0x4D49444C
TYPE_LD19 = 0
TYPE_IMU = 1
LD19_POINTS = 12
GYR_LSB_PER_DPS = 16.4
DEG = math.pi / 180.0
# This rig records the IMU in a proper +Z-up frame; the LD19 angle sweeps CCW in
# the tool's (x,y) frame, so positive gyro-Z must be negated to match the deskew
# rotation sense. (The Pi rig mounted the IMU inverted, so it used +1.)
GYRO_SIGN = -1.0


# --- .ldim parsing --------------------------------------------------------

def load_ldim(path):
    """Return (packets, imu_ns, imu_wz).

    packets: list of (abs_ns, start_deg, end_deg, [(dist_mm, intensity), ...])
    imu_wz:  yaw rate about z in rad/s.
    """
    packets, imu_ns, imu_wz = [], [], []
    with open(path, "rb") as f:
        hdr = f.read(16)
        if len(hdr) != 16:
            sys.exit("truncated header")
        magic, _, _ = struct.unpack_from("<IHH", hdr)
        base, = struct.unpack_from("<Q", hdr, 8)
        if magic != MAGIC:
            sys.exit(f"bad magic 0x{magic:08x}")
        while True:
            rh = f.read(12)
            if len(rh) < 12:
                break
            ns, type_, _, length = struct.unpack("<QBBH", rh)
            pl = f.read(length)
            if len(pl) < length:
                break
            abs_ns = base + ns
            if type_ == TYPE_LD19:
                a0 = struct.unpack_from("<H", pl, 4)[0] / 100.0
                a1 = struct.unpack_from("<H", pl, 6 + 12 * 3)[0] / 100.0
                pts = [struct.unpack_from("<HB", pl, 6 + i * 3) for i in range(LD19_POINTS)]
                packets.append((abs_ns, a0, a1, pts))
            elif type_ == TYPE_IMU:
                _, _, gz, _, _, _ = struct.unpack("<hhhhhh", pl)
                imu_ns.append(abs_ns)
                imu_wz.append(GYRO_SIGN * gz / GYR_LSB_PER_DPS * DEG)
    imu_ns = np.array(imu_ns, float)
    imu_wz = np.array(imu_wz)
    order = np.argsort(imu_ns)
    return packets, imu_ns[order], imu_wz[order]


# --- deskew + scan assembly ----------------------------------------------

def build_yaw(imu_ns, imu_wz):
    """Cumulative gyro-integrated heading; returns yaw_at(t_ns) interpolator."""
    if len(imu_ns) < 2:
        return lambda t: 0.0
    cum = np.concatenate(
        [[0.0], np.cumsum(np.diff(imu_ns) / 1e9 * (imu_wz[:-1] + imu_wz[1:]) * 0.5)])
    return lambda t: float(np.interp(t, imu_ns, cum))


def assemble_scans(packets, yaw_at, rng=(0.05, 12.0), min_pts=50):
    """Group packets into revolutions (start-angle wrap) and deskew each one
    into the rig frame at the revolution's mid-time. Returns (scans, times)."""
    scans, times, cur, last_a0 = [], [], [], -1.0

    def flush(group):
        if len(group) < 20:
            return None
        t_ref = group[len(group) // 2][0]
        y_ref = yaw_at(t_ref)
        out = []
        for (t_ns, a0, a1, pts) in group:
            span = a1 - a0
            if span < 0:
                span += 360.0
            dy = yaw_at(t_ns) - y_ref           # rig rotation since reference
            c, s = math.cos(dy), math.sin(dy)
            for i, (d, _inten) in enumerate(pts):
                if d <= 0:
                    continue
                ang = (a0 + span * i / (LD19_POINTS - 1)) * DEG
                d_m = d / 1000.0
                x = -math.sin(ang) * d_m
                y = math.cos(ang) * d_m
                out.append((c * x - s * y, s * x + c * y))   # deskew rotate by +dy
        if not out:
            return None
        return np.array(out), t_ref

    for p in packets:
        a0 = p[1]
        if a0 < last_a0 - 180.0 and cur:
            r = flush(cur)
            if r is not None:
                scans.append(r[0]); times.append(r[1])
            cur = []
        cur.append(p); last_a0 = a0
    r = flush(cur)
    if r is not None:
        scans.append(r[0]); times.append(r[1])

    kept_s, kept_t = [], []
    for s, t in zip(scans, times):
        r = np.hypot(s[:, 0], s[:, 1])
        s = s[(r > rng[0]) & (r < rng[1])]
        if len(s) >= min_pts:
            kept_s.append(s); kept_t.append(t)
    return kept_s, kept_t


# --- ICP ------------------------------------------------------------------

def voxel(p, leaf):
    if len(p) == 0:
        return p
    key = np.round(p / leaf).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return p[idx]


def mat(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def apply(M, p):
    return p @ M[:2, :2].T + M[:2, 2]


def _icp_pass(src, kd, tgt, M, iters, max_dist, trim):
    for _ in range(iters):
        T = apply(M, src)
        d, idx = kd.query(T, distance_upper_bound=max_dist)
        ok = np.isfinite(d)
        if ok.sum() < 10:
            break
        ok &= d <= np.percentile(d[ok], trim * 100)
        sp = T[ok]; tp = tgt[idx[ok]]
        cs = sp.mean(0); ct = tp.mean(0)
        H = (sp - cs).T @ (tp - ct)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1
            R = Vt.T @ U.T
        dM = np.eye(3)
        dM[:2, :2] = R
        dM[:2, 2] = ct - R @ cs
        M = dM @ M
    return M


def icp(src, tgt, M0, schedule):
    if len(src) < 10 or len(tgt) < 10:
        return M0
    kd = cKDTree(tgt)
    M = M0
    for step in schedule:
        M = _icp_pass(src, kd, tgt, M, *step)
    return M


def yaw_of(M):
    return math.atan2(M[1, 0], M[0, 0])


# --- SLAM + loop closure --------------------------------------------------

def run_slam(scans, times, yaw_at, win=50, voxel_map=0.03, voxel_src=0.04):
    poses = [mat(0, 0, 0)]
    recent = [apply(poses[0], scans[0])]
    for i in range(1, len(scans)):
        px, py = poses[-1][0, 2], poses[-1][1, 2]
        dyaw = yaw_at(times[i]) - yaw_at(times[i - 1])
        M0 = mat(px, py, yaw_of(poses[-1]) + dyaw)
        local = voxel(np.vstack(recent[-win:]), voxel_map)
        M = icp(voxel(scans[i], voxel_src), local, M0,
                [(30, 0.4, 0.85), (25, 0.15, 0.92)])
        poses.append(M)
        recent.append(apply(M, scans[i]))
    return poses


def close_loop(poses, scans, k=45, leaf=0.025):
    N = len(poses)
    if N < 2 * k:
        return poses, None
    start_ref = voxel(np.vstack([apply(poses[j], scans[j]) for j in range(k)]), leaf)
    end_glob = voxel(np.vstack([apply(poses[j], scans[j]) for j in range(N - k, N)]), leaf)
    C = icp(end_glob, start_ref, np.eye(3),
            [(60, 2.0, 0.9), (50, 0.7, 0.92), (40, 0.25, 0.95)])
    # weight correction by per-step motion (drift concentrates where it moves)
    trans = [0.0] + [np.hypot(poses[i][0, 2] - poses[i - 1][0, 2],
                              poses[i][1, 2] - poses[i - 1][1, 2]) for i in range(1, N)]
    rot = [0.0] + [abs((yaw_of(poses[i]) - yaw_of(poses[i - 1]) + math.pi)
                       % (2 * math.pi) - math.pi) for i in range(1, N)]
    w = np.array(rot) * 2.0 + np.array(trans)
    f = np.cumsum(w)
    f = f / f[-1] if f[-1] > 0 else f
    logC = logm(C).real
    return [expm(f[i] * logC) @ poses[i] for i in range(N)], C


# --- render ---------------------------------------------------------------

def straighten_angle(xy, res=0.05):
    """Manhattan-frame estimate: find the rotation in [0, 90) that makes the
    point projections onto x and y as concentrated as possible (i.e. aligns the
    dominant walls with the image axes)."""
    best_a, best_score = 0.0, -1.0
    for a in np.arange(0.0, 90.0, 0.5):
        r = math.radians(a)
        c, s = math.cos(r), math.sin(r)
        rx = xy[:, 0] * c - xy[:, 1] * s
        ry = xy[:, 0] * s + xy[:, 1] * c
        hx, _ = np.histogram(rx, bins=max(1, int((rx.max() - rx.min()) / res)))
        hy, _ = np.histogram(ry, bins=max(1, int((ry.max() - ry.min()) / res)))
        # inverse participation ratio: peaks high when mass sits in few bins
        score = ((hx / hx.sum()) ** 2).sum() + ((hy / hy.sum()) ** 2).sum()
        if score > best_score:
            best_score, best_a = score, a
    return math.radians(best_a)


def declutter(occ, min_neighbors=4, min_blob=15):
    """Drop the path trail: thin 1-px streaks and isolated specks left by the
    rig's trajectory / sparse reflections, keeping solid wall structure."""
    from scipy.ndimage import convolve, label
    cnt = convolve(occ.astype(np.int32), np.ones((3, 3), np.int32), mode="constant")
    occ = occ & (cnt >= min_neighbors)
    lab, n = label(occ)
    if n:
        sizes = np.bincount(lab.ravel())
        sizes[0] = 0
        occ = (sizes >= min_blob)[lab]
    return occ


def prune_spurs(skel, min_len):
    """Remove short skeleton branches that dead-end (spurs). A branch is the run
    of skeleton pixels between junctions; one that touches a free endpoint and is
    shorter than `min_len` pixels is deleted. Iterates because pruning one spur
    can expose another."""
    from scipy.ndimage import convolve, label
    K = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]], np.int32)
    s8 = np.ones((3, 3), np.int32)
    skel = skel.copy()
    while True:
        deg = convolve(skel.astype(np.int32), K, mode="constant") * skel
        junctions = skel & (deg >= 3)
        endpoints = skel & (deg == 1)
        segs = skel & ~junctions
        lab, n = label(segs, structure=s8)
        if not n:
            break
        sizes = np.bincount(lab.ravel())
        touches_end = np.zeros(n + 1, bool)
        ep_labels = lab[endpoints]
        touches_end[ep_labels[ep_labels > 0]] = True
        remove_ids = [i for i in range(1, n + 1)
                      if touches_end[i] and sizes[i] < min_len]
        if not remove_ids:
            break
        skel[np.isin(lab, remove_ids)] = False
    return skel


def render(poses, scans, out_png, res=0.025, hits=2, title="Floorplan",
           mirror=True, straighten=True, clean=True, thin=False):
    allp = np.vstack([apply(P, s) for P, s in zip(poses, scans)])
    xs, ys = allp[:, 0], allp[:, 1]
    # The LD19 polar->cartesian convention comes out mirrored vs reality; flip x
    # to match the live viewer's g_mirror toggle (lidar_view.c: wx = -wx).
    if mirror:
        xs = -xs
    if straighten:
        a = straighten_angle(np.column_stack([xs, ys]))
        c, s = math.cos(a), math.sin(a)
        xs, ys = xs * c - ys * s, xs * s + ys * c
        print(f"straighten: rotated map by {math.degrees(a):.1f} deg")
    mnx, mny = xs.min(), ys.min()
    W = int((xs.max() - mnx) / res) + 1
    H = int((ys.max() - mny) / res) + 1
    grid = np.zeros((H, W), np.int32)
    np.add.at(grid, (((ys - mny) / res).astype(int), ((xs - mnx) / res).astype(int)), 1)
    occ = grid >= hits
    if clean:
        occ = declutter(occ)
    if thin:
        # collapse the drift-thickened wall bands to single-pixel centerlines,
        # so walls look as thin as in one scan frame. Close first to consolidate
        # the rough/holey bands, otherwise the skeleton comes out beaded.
        from skimage.morphology import skeletonize
        from scipy.ndimage import binary_closing
        occ = binary_closing(occ, np.ones((3, 3), bool), iterations=2)
        occ = skeletonize(occ)
        occ = prune_spurs(occ, min_len=int(round(0.25 / res)))
    # crop to the occupied content (+ small margin) so the corner is well defined
    rows, cols = np.any(occ, axis=1), np.any(occ, axis=0)
    if rows.any() and cols.any():
        m = int(round(0.3 / res))
        r0, r1 = np.where(rows)[0][[0, -1]]
        c0, c1 = np.where(cols)[0][[0, -1]]
        r0, r1 = max(0, r0 - m), min(H - 1, r1 + m)
        c0, c1 = max(0, c0 - m), min(W - 1, c1 + m)
        occ = occ[r0:r1 + 1, c0:c1 + 1]
    Hc, Wc = occ.shape
    img = np.where(occ, 0, 255).astype(np.uint8)
    plt.figure(figsize=(12, 12))
    # origin (0,0) at the lower-left corner: x runs [0, width], y runs [0, height]
    plt.imshow(img, cmap="gray", origin="lower",
               extent=[0.0, Wc * res, 0.0, Hc * res])
    from matplotlib.ticker import MultipleLocator
    ax = plt.gca()
    ax.set_aspect("equal")
    ax.xaxis.set_major_locator(MultipleLocator(0.5))
    ax.yaxis.set_major_locator(MultipleLocator(0.5))
    ax.tick_params(labelsize=7)
    plt.setp(ax.get_xticklabels(), rotation=90)
    ax.grid(True, which="major", linewidth=0.3, color="0.8")
    plt.title(title)
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.savefig(out_png, dpi=130, bbox_inches="tight")
    plt.close()


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="input .ldim file")
    ap.add_argument("-o", "--output", help="output PNG (default: <input>_floorplan.png)")
    ap.add_argument("--res", type=float, default=0.025, help="metres/pixel (default 0.025)")
    ap.add_argument("--hits", type=int, default=2,
                    help="min point hits per cell to mark a wall (default 2)")
    ap.add_argument("--window", type=int, default=50, help="local-map window in scans")
    ap.add_argument("--no-loop", action="store_true", help="skip loop closure")
    ap.add_argument("--no-mirror", action="store_true",
                    help="do not mirror x (default mirrors to match the live viewer)")
    ap.add_argument("--no-straighten", action="store_true",
                    help="do not rotate the map to axis-align the walls")
    ap.add_argument("--keep-trail", action="store_true",
                    help="keep the path trail (skip the declutter / streak removal)")
    ap.add_argument("--thin", action="store_true",
                    help="thin walls to single-pixel centerlines (like one scan frame)")
    args = ap.parse_args()

    out = args.output or args.input.rsplit(".", 1)[0] + "_floorplan.png"

    packets, imu_ns, imu_wz = load_ldim(args.input)
    print(f"{len(packets)} LD19 packets, {len(imu_ns)} IMU samples, "
          f"{(packets[-1][0] - packets[0][0]) / 1e9:.1f}s")
    yaw_at = build_yaw(imu_ns, imu_wz)
    scans, times = assemble_scans(packets, yaw_at)
    print(f"{len(scans)} deskewed scans")

    poses = run_slam(scans, times, yaw_at, win=args.window)
    print(f"forward SLAM net yaw: {math.degrees(yaw_of(poses[-1])):.1f} deg")

    if not args.no_loop:
        poses, C = close_loop(poses, scans)
        if C is not None:
            print(f"loop closure: dx={C[0,2]:+.3f} dy={C[1,2]:+.3f} "
                  f"dtheta={math.degrees(yaw_of(C)):+.2f} deg")

    render(poses, scans, out, res=args.res, hits=args.hits,
           mirror=not args.no_mirror, straighten=not args.no_straighten,
           clean=not args.keep_trail, thin=args.thin,
           title=f"Floorplan — {args.input}")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
