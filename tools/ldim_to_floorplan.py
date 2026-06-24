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
    packets, imu_ns, imu_wz, imu_data = [], [], [], []
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
                gx, gy, gz, ax, ay, az = struct.unpack("<hhhhhh", pl)
                imu_ns.append(abs_ns)
                w = np.array([gx, gy, gz]) / 16.4 * (math.pi / 180.0)
                a = np.array([ax, ay, az])
                imu_data.append((abs_ns, w, a))
                imu_wz.append(GYRO_SIGN * gz / 16.4 * (math.pi / 180.0))
    imu_ns = np.array(imu_ns, float)
    imu_wz = np.array(imu_wz)
    order = np.argsort(imu_ns)
    return packets, imu_ns[order], imu_wz[order], sorted(imu_data)


# --- operator removal -----------------------------------------------------

def strip_operator(packets, rng=1.1, frac=0.5, nbins=72, sector=None, margin=0.3):
    """Blank the operator from the scan, in memory. The operator walks close
    behind the scanner, so their body is a persistent close-range return in a
    fixed *sensor-frame* angular sector (it keeps the same bearing/range to the
    rig as the room sweeps past).

    The discriminator vs. a real wall is *temporal persistence*: a wall is only
    close while you walk past it (a minority of revolutions), but the operator
    is close in nearly every revolution. So we split the recording into
    revolutions and flag the bins that hold a near (< `rng`) return in more than
    `frac` of them -- a far stronger signal than the old "what fraction of this
    bin's returns are close" test, which a wall you walk alongside also trips.
    The flagged sector is dilated one bin each side (the body's angular edges
    sit just under threshold and otherwise leak a fringe), and points are
    blanked only within `margin` of the operator's *own* per-bin range. That
    range-band cull keeps a wall that later appears farther out in the same
    bearing, which a flat near-cull would have eaten. The torso sits at
    0.3-0.8 m, so a plain global <0.3 m cull misses it entirely.

    Pass `sector`=(lo,hi) deg to force the bearing instead of auto-detecting.
    Returns new packets with blanked points set to distance 0 (assemble skips
    d<=0)."""
    w = nbins / 360.0
    def ang_of(a0, span, i):
        return (a0 + span * i / (LD19_POINTS - 1)) % 360.0
    def bin_of(a):
        return int(a * w) % nbins

    if sector is None:
        # Segment into revolutions on the a0 wrap (mirrors assemble_scans).
        revs, cur, last_a0 = [], [], -1.0
        for p in packets:
            if p[1] < last_a0 - 180.0 and cur:
                revs.append(cur); cur = []
            cur.append(p); last_a0 = p[1]
        if cur:
            revs.append(cur)
        # Per revolution, the closest near return seen in each bin = the
        # operator candidate for that revolution.
        present = np.zeros(nbins, int)
        op_ranges = [[] for _ in range(nbins)]
        for rev in revs:
            closest = np.full(nbins, np.inf)
            for (_t, a0, a1, pts) in rev:
                span = a1 - a0
                if span < 0:
                    span += 360.0
                for i, (d, _v) in enumerate(pts):
                    dm = d / 1000.0
                    if d > 0 and dm < rng:
                        b = bin_of(ang_of(a0, span, i))
                        if dm < closest[b]:
                            closest[b] = dm
            for b in np.nonzero(np.isfinite(closest))[0]:
                present[b] += 1
                op_ranges[b].append(closest[b])
        occ = present / max(1, len(revs))
        core = occ > frac
        mask = core | np.roll(core, 1) | np.roll(core, -1)   # dilate +/-1 bin
        edge = mask & ~core
        # Core bins ARE the body: it sits at point-blank in every revolution and
        # occludes whatever is behind it at that bearing, so there is no real
        # wall to keep -- blank the whole sector out to rng. That clears the body
        # at *every* range it appears, including the larger stand-off at the
        # start of the capture, which a median-range band leaves as a "shadow".
        # Dilated edge bins only clip the body's fringe and may catch a real
        # wall, so there blank just within margin of the operator's own range.
        thr = np.where(core, rng, 0.0)
        for b in np.nonzero(edge)[0]:
            thr[b] = min(rng, float(np.median(op_ranges[b])) + margin) if op_ranges[b] else 0.0
        in_self = lambda a: mask[bin_of(a)]
        thr_of = lambda a: thr[bin_of(a)]
        label = f"auto {int(mask.sum())}/{nbins} bins, {len(revs)} revs"
    else:
        lo, hi = sector
        in_self = lambda a: (lo <= a <= hi) if lo <= hi else (a >= lo or a <= hi)
        thr_of = lambda a: rng
        label = f"sector {lo}-{hi} deg"

    out, dropped, total = [], 0, 0
    for (t_ns, a0, a1, pts) in packets:
        span = a1 - a0
        if span < 0:
            span += 360.0
        new = []
        for i, (d, v) in enumerate(pts):
            total += 1
            a = ang_of(a0, span, i)
            if d > 0 and d / 1000.0 < thr_of(a) and in_self(a):
                new.append((0, v)); dropped += 1
            else:
                new.append((d, v))
        out.append((t_ns, a0, a1, new))
    print(f"operator strip: blanked {dropped}/{total} returns (< {rng} m, {label})")
    return out


# --- deskew + scan assembly ----------------------------------------------

def build_orientation(imu_data):
    orientations = []
    last_t = imu_data[0][0]
    roll, pitch, yaw = 0.0, 0.0, 0.0
    for t_ns, w, a in imu_data:
        dt = (t_ns - last_t) / 1e9
        last_t = t_ns
        roll  += w[0] * dt
        pitch += w[1] * dt
        yaw   += GYRO_SIGN * w[2] * dt
        ax, ay, az = a
        a_norm = math.sqrt(ax*ax + ay*ay + az*az)
        if a_norm > 0:
            a_roll  = math.atan2(ay, az)
            a_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
            alpha = 0.02
            roll  = (1 - alpha) * roll  + alpha * a_roll
            pitch = (1 - alpha) * pitch + alpha * a_pitch
        orientations.append((t_ns, roll, pitch, yaw))
    ts = np.array([o[0] for o in orientations], float)
    rs = np.array([o[1] for o in orientations])
    ps = np.array([o[2] for o in orientations])
    ys = np.array([o[3] for o in orientations])
    def get_rot(t_ns):
        return (float(np.interp(t_ns, ts, rs)),
                float(np.interp(t_ns, ts, ps)),
                float(np.interp(t_ns, ts, ys)))
    return get_rot


# Accel full-scale is +/-2 g at 16384 LSB/g (see imu-hardware notes), so a
# sample at rest has |a| ~ ACC_LSB_PER_G. We only need this for the KF's
# "is the accel currently just gravity?" test; the complementary filter uses
# atan2 ratios where the scale cancels.
ACC_LSB_PER_G = 16384.0


def build_orientation_kf(imu_data, gate=0.03, q_ang=20.0, r_ang=1.0):
    """Attitude (roll/pitch) via a Kalman filter that fuses gyro and accel.

    This is the motion-noise-rejecting alternative to build_orientation's
    fixed-gain complementary filter. State = [roll, pitch] in rad; the gyro
    drives the prediction and the accelerometer supplies a gravity-direction
    measurement that corrects integration drift.

    The handheld catch is that walking acceleration adds to gravity, so the
    accel-derived tilt is only trustworthy when |a| ~ 1 g. Rather than a fixed
    complementary gain (which trusts a fake tilt while walking) or a hard gate,
    the filter inflates the measurement noise R smoothly with how far |a|
    strays from gravity, so motion noise is down-weighted in proportion to how
    much motion there is. `gate` sets the deviation (fraction of g) at which R
    has doubled; samples well past it are effectively ignored.

    Yaw has no absolute reference on this rig (no magnetometer), so it stays a
    plain gyro integration, sign-matched to the deskew convention.
    """
    x = np.zeros(2)              # [roll, pitch]
    P = np.eye(2) * 1e-2
    I = np.eye(2)
    yaw = 0.0
    last_t = imu_data[0][0]
    ts, rs, ps, ys = [], [], [], []
    # Seed roll/pitch from the first usable accel sample so we don't spend the
    # first second of the scan converging up from a flat-zero guess.
    for _t, _w, a in imu_data:
        ax, ay, az = a
        if ax or ay or az:
            x[0] = math.atan2(ay, az)
            x[1] = math.atan2(-ax, math.sqrt(ay*ay + az*az))
            break
    for t_ns, w, a in imu_data:
        dt = (t_ns - last_t) / 1e9
        last_t = t_ns
        if dt <= 0:
            dt = 1e-3
        # Predict: integrate the GYRO body rates (w); covariance grows with the
        # gyro process noise over the step.
        x[0] += w[0] * dt                       # roll  from gyro X
        x[1] += w[1] * dt                       # pitch from gyro Y
        P = P + I * (q_ang * dt * dt)
        yaw += GYRO_SIGN * w[2] * dt            # yaw   from gyro Z (gyro-only)
        # Correct: ACCELEROMETER (a) gravity direction, weighted by |a|-vs-1g.
        ax, ay, az = a
        a_norm = math.sqrt(ax*ax + ay*ay + az*az)
        if a_norm > 0:
            z = np.array([math.atan2(ay, az),                 # accel roll
                          math.atan2(-ax, math.sqrt(ay*ay + az*az))])  # accel pitch
            dev = abs(a_norm / ACC_LSB_PER_G - 1.0)
            R = I * (r_ang * (1.0 + (dev / gate) ** 2))
            K = P @ np.linalg.inv(P + R)        # H = I
            x = x + K @ (z - x)                 # fuse accel tilt into roll/pitch
            P = (I - K) @ P
        ts.append(t_ns); rs.append(x[0]); ps.append(x[1]); ys.append(yaw)
    ts = np.array(ts, float); rs = np.array(rs); ps = np.array(ps); ys = np.array(ys)

    def get_rot(t_ns):
        return (float(np.interp(t_ns, ts, rs)),
                float(np.interp(t_ns, ts, ps)),
                float(np.interp(t_ns, ts, ys)))
    return get_rot


def rotate_3d(x, y, z, roll, pitch, yaw):
    c, s = math.cos(roll), math.sin(roll)
    y, z = c*y - s*z, s*y + c*z
    c, s = math.cos(pitch), math.sin(pitch)
    x, z = c*x + s*z, -s*x + c*z
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = c*x - s*y, s*x + c*y
    return x, y, z


def assemble_scans(packets, get_rot, rng=(0.25, 12.0), min_pts=50):
    # rng[0] is a global near floor: nothing real sits within 0.25 m of a
    # handheld scanner, so this mops up point-blank clutter (a hand, or the
    # operator's setup pose at the start) that the sensor-frame persistence
    # strip can't catch because it only occupies that bearing for a moment.
    scans, times, cur, last_a0 = [], [], [], -1.0
    PKT_NS = 3.33e6

    def flush(group):
        if len(group) < 20:
            return None
        t_ref = group[len(group) // 2][0]
        r_ref, p_ref, y_ref = get_rot(t_ref)
        out = []
        for (t_ns, a0, a1, pts) in group:
            span = a1 - a0
            if span < 0:
                span += 360.0
            for i, (d, _inten) in enumerate(pts):
                if d <= 0:
                    continue
                t_pt = t_ns + (i / (LD19_POINTS - 1)) * PKT_NS
                r, p, y = get_rot(t_pt)
                dy = y - y_ref
                ang = (a0 + span * i / (LD19_POINTS - 1)) * DEG
                d_m = d / 1000.0
                lx, ly, lz = -math.sin(ang) * d_m, math.cos(ang) * d_m, 0.0
                rx, ry, rz = rotate_3d(lx, ly, lz, r, p, dy)
                if -0.5 < rz < 1.0:
                    out.append((rx, ry))
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


def declutter(occ, min_neighbors=3, min_blob=15):
    """Drop the path trail: thin 1-px streaks and isolated specks left by the
    rig's trajectory / sparse reflections, keeping solid wall structure.

    min_neighbors counts the cell + its 8-neighbours: 1=lone speck, 2=a pair,
    3=an interior pixel of a straight 1-px line. Keep >=3 so genuine thin walls
    (a far/grazing wall that only painted a single-pixel line) survive; the
    connected-component min_blob then removes the short scattered runs that a
    4-cull would have caught. (A 4-cull erodes every 1-px wall, which deleted
    real short wall segments.)"""
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
    # Hysteresis threshold: a faint/grazing wall paints a sparse line of mostly
    # 1-hit cells with only a few >= hits cells, which a flat `grid >= hits` cull
    # guts (and declutter then finishes off). Instead keep every >=1-hit cell
    # whose connected blob contains at least one strong (>= hits) cell -- the
    # faint parts of a real wall are recovered, but isolated 1-hit specks (no
    # strong evidence anywhere in their blob) are still dropped.
    if hits > 1:
        from scipy.ndimage import label as _label
        lab, n = _label(grid >= 1)
        keep = np.zeros(n + 1, bool)
        keep[np.unique(lab[grid >= hits])] = True
        keep[0] = False
        occ = keep[lab]
    else:
        occ = grid >= hits

    if clean:
        occ = declutter(occ)
        
    if thin:
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
    ap.add_argument("--keep-operator", action="store_true",
                    help="do not strip the operator (close fixed-bearing returns)")
    ap.add_argument("--self-range", type=float, default=1.1,
                    help="operator strip max range in metres (default 1.1; the body sits "
                         "at 0.3-0.8 m, so walls beyond ~1 m are kept)")
    ap.add_argument("--self-frac", type=float, default=0.5,
                    help="operator persistence: blank a bearing held near in more than this "
                         "fraction of revolutions (default 0.5; lower = strip more aggressively)")
    ap.add_argument("--self-ang", default="",
                    help="force operator sector 'lo,hi' deg (wrap-aware) instead of auto-detect")
    ap.add_argument("--no-kf", action="store_true",
                    help="use the fixed-gain complementary filter for tilt instead of the "
                         "motion-adaptive Kalman filter (default)")
    ap.add_argument("--kf-gate", type=float, default=0.03,
                    help="Kalman accel-trust gate: |a|-vs-1g deviation (fraction of g) at "
                         "which the tilt measurement noise doubles (default 0.03, tuned on scan_006)")
    args = ap.parse_args()

    out = args.output or args.input.rsplit(".", 1)[0] + "_floorplan.png"

    packets, imu_ns, imu_wz, imu_data = load_ldim(args.input)
    print(f"{len(packets)} LD19 packets, {len(imu_ns)} IMU samples, "
          f"{(packets[-1][0] - packets[0][0]) / 1e9:.1f}s")
    if not args.keep_operator:
        sector = tuple(float(v) for v in args.self_ang.split(",")) if args.self_ang else None
        packets = strip_operator(packets, rng=args.self_range, frac=args.self_frac, sector=sector)
    if args.no_kf:
        get_rot = build_orientation(imu_data)
        print("tilt: fixed-gain complementary filter")
    else:
        get_rot = build_orientation_kf(imu_data, gate=args.kf_gate)
        print(f"tilt: motion-adaptive Kalman filter (gate={args.kf_gate})")
    yaw_at = lambda t: get_rot(t)[2]
    scans, times = assemble_scans(packets, get_rot)
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
