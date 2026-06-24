#!/usr/bin/env python3
"""Convert a .ldim recording to a unified 3D PointCloud2 in an MCAP file.

This tool merges all deskewed and SLAM-aligned scans into a sequence of
3D frames (or a single merged frame), correctly compensating for the 
10Hz lidar spin and rig motion using the IMU data.

Usage:
  make viz3d LDIM=recordings/scan_007.ldim
"""
import argparse
import math
import struct
import sys
from pathlib import Path

import numpy as np

try:
    from scipy.spatial import cKDTree
    from scipy.linalg import logm, expm
    from mcap_ros2.writer import Writer as Ros2Writer
except ImportError:
    sys.exit("missing deps — tools/.venv/bin/pip install numpy scipy mcap-ros2-support")

MAGIC = 0x4D49444C
TYPE_LD19 = 0
TYPE_IMU = 1
LD19_POINTS = 12
GYR_LSB_PER_DPS = 16.4
DEG = math.pi / 180.0
# The firmware already negates gz to match CCW floorplan logic.
GYRO_SIGN = -1.0 

POINT_CLOUD2_SCHEMA = """\
std_msgs/Header header
uint32 height
uint32 width
sensor_msgs/PointField[] fields
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: sensor_msgs/PointField
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8
string name
uint32 offset
uint8 datatype
uint32 count
"""

# --- .ldim parsing --------------------------------------------------------

def load_ldim(path):
    packets, imu_ns, imu_wz, imu_data = [], [], [], []
    with open(path, "rb") as f:
        hdr = f.read(16)
        if len(hdr) != 16: sys.exit("truncated header")
        magic, _, _ = struct.unpack_from("<IHH", hdr)
        base, = struct.unpack_from("<Q", hdr, 8)
        if magic != MAGIC: sys.exit(f"bad magic 0x{magic:08x}")
        while True:
            rh = f.read(12)
            if len(rh) < 12: break
            ns, type_, _, length = struct.unpack("<QBBH", rh)
            pl = f.read(length)
            if len(pl) < length: break
            abs_ns = base + ns
            if type_ == TYPE_LD19:
                a0 = struct.unpack_from("<H", pl, 4)[0] / 100.0
                a1 = struct.unpack_from("<H", pl, 6 + 12 * 3)[0] / 100.0
                pts = [struct.unpack_from("<HB", pl, 6 + i * 3) for i in range(LD19_POINTS)]
                packets.append((abs_ns, a0, a1, pts))
            elif type_ == TYPE_IMU:
                gx, gy, gz, ax, ay, az = struct.unpack("<hhhhhh", pl)
                imu_ns.append(abs_ns)
                # Firmware frame (mount-corrected): X=forward, Y=left, Z=up
                w = np.array([gx, gy, gz]) / GYR_LSB_PER_DPS * DEG
                a = np.array([ax, ay, az])
                imu_data.append((abs_ns, w, a))
                imu_wz.append(GYRO_SIGN * gz / GYR_LSB_PER_DPS * DEG)
    imu_ns = np.array(imu_ns, float)
    imu_wz = np.array(imu_wz)
    order = np.argsort(imu_ns)
    return packets, imu_ns[order], imu_wz[order], sorted(imu_data)

def build_yaw(imu_ns, imu_wz):
    if len(imu_ns) < 2: return lambda t: 0.0
    cum = np.concatenate([[0.0], np.cumsum(np.diff(imu_ns) / 1e9 * (imu_wz[:-1] + imu_wz[1:]) * 0.5)])
    return lambda t: float(np.interp(t, imu_ns, cum))

def build_orientation(imu_data):
    """Estimate tilt (pitch/roll) using accelerometer and integrate yaw."""
    orientations = []
    last_t = imu_data[0][0]
    # Rig frame: X=forward, Y=left, Z=up
    # Rotation around X = Roll, Y = Pitch, Z = Yaw
    roll, pitch, yaw = 0.0, 0.0, 0.0
    for t_ns, w, a in imu_data:
        dt = (t_ns - last_t) / 1e9
        last_t = t_ns
        # Gyro integration
        roll  += w[0] * dt
        pitch += w[1] * dt
        yaw   += GYRO_SIGN * w[2] * dt
        # Accel correction
        ax, ay, az = a
        a_norm = math.sqrt(ax*ax + ay*ay + az*az)
        if a_norm > 0:
            # Gravity-based tilt
            a_roll  = math.atan2(ay, az)
            a_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
            alpha = 0.02
            roll  = (1 - alpha) * roll  + alpha * a_roll
            pitch = (1 - alpha) * pitch + alpha * a_pitch
        orientations.append((t_ns, roll, pitch, yaw))

    # Precompute arrays once so per-point lookups are O(log n), not O(n).
    ts = np.array([o[0] for o in orientations], float)
    rs = np.array([o[1] for o in orientations])
    ps = np.array([o[2] for o in orientations])
    ys = np.array([o[3] for o in orientations])

    def get_rot(t_ns):
        return (float(np.interp(t_ns, ts, rs)),
                float(np.interp(t_ns, ts, ps)),
                float(np.interp(t_ns, ts, ys)))
    return get_rot

ACC_LSB_PER_G = 16384.0  # +/-2 g full-scale; |a| ~ this at rest (for the KF gate)

def build_orientation_kf(imu_data, gate=0.03, q_ang=20.0, r_ang=1.0):
    """Attitude (roll/pitch) via a Kalman filter that fuses gyro and accel.

    Opt-in (`--kf`) alternative to build_orientation's fixed-gain complementary
    filter. State = [roll, pitch] in rad; the gyro drives the predict, the accel
    gravity-direction is the measurement, and the measurement noise R is inflated
    smoothly with |a|/16384 - 1 so walking acceleration (which masquerades as
    tilt) is down-weighted in proportion to motion. `gate` is the deviation
    (fraction of g) at which R doubles. Yaw stays plain gyro integration."""
    x = np.zeros(2)
    P = np.eye(2) * 1e-2
    I = np.eye(2)
    yaw = 0.0
    last_t = imu_data[0][0]
    ts, rs, ps, ys = [], [], [], []
    for _t, _w, a in imu_data:          # seed from first usable accel sample
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
        # --- Predict: integrate the GYRO body rates (w) ---
        x[0] += w[0] * dt                       # roll  from gyro X
        x[1] += w[1] * dt                       # pitch from gyro Y
        P = P + I * (q_ang * dt * dt)
        yaw += GYRO_SIGN * w[2] * dt            # yaw   from gyro Z (gyro-only)
        # --- Correct: ACCELEROMETER (a) gravity direction, motion-weighted ---
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
    """Rigid 3D rotation: Roll(x), Pitch(y), Yaw(z)."""
    # Roll (X)
    c, s = math.cos(roll), math.sin(roll)
    y, z = c*y - s*z, s*y + c*z
    # Pitch (Y)
    c, s = math.cos(pitch), math.sin(pitch)
    x, z = c*x + s*z, -s*x + c*z
    # Yaw (Z)
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = c*x - s*y, s*x + c*y
    return x, y, z

def assemble_scans_3d(packets, get_rot, rng=(0.05, 12.0), min_pts=50):
    scans, times, cur, last_a0 = [], [], [], -1.0
    # Packet duration is ~3.3ms (300Hz)
    PKT_NS = 3.33e6 
    
    def flush(group):
        if len(group) < 20: return None
        t_ref = group[len(group) // 2][0]
        r_ref, p_ref, y_ref = get_rot(t_ref)
        out = []
        for (t_ns, a0, a1, pts) in group:
            span = a1 - a0
            if span < 0: span += 360.0
            for i, (d, inten) in enumerate(pts):
                if d <= 0: continue
                # Exact time of this point within the packet
                t_pt = t_ns + (i / (LD19_POINTS - 1)) * PKT_NS
                r, p, y = get_rot(t_pt)
                # Deskew relative to scan mid-time
                dr, dp, dy = r - r_ref, p - p_ref, y - y_ref
                
                ang = (a0 + span * i / (LD19_POINTS - 1)) * DEG
                d_m = d / 1000.0
                # Sensor raw: ang=0 is Forward (+X? No, floorplan uses ang=0 -> +Y)
                # Let's match floorplan: lx = -sin(ang), ly = cos(ang)
                lx, ly, lz = -math.sin(ang) * d_m, math.cos(ang) * d_m, 0.0
                
                # Apply 3D deskew
                rx, ry, rz = rotate_3d(lx, ly, lz, dr, dp, dy)
                out.append((rx, ry, rz, float(inten)))
        if not out: return None
        return np.array(out), t_ref

    for p in packets:
        a0 = p[1]
        if a0 < last_a0 - 180.0 and cur:
            r = flush(cur); (scans.append(r[0]), times.append(r[1])) if r else None
            cur = []
        cur.append(p); last_a0 = a0
    r = flush(cur); (scans.append(r[0]), times.append(r[1])) if r else None
    kept_s, kept_t = [], []
    for s, t in zip(scans, times):
        r = np.hypot(s[:, 0], s[:, 1])
        s = s[(r > rng[0]) & (r < rng[1])]
        if len(s) >= min_pts: kept_s.append(s); kept_t.append(t)
    return kept_s, kept_t

def assemble_static_3d(packets, get_rot, rng=(0.05, 12.0), mount=(0.0, 0.0, 0.0),
                       pivot=(0.0, 0.0, 0.0)):
    """Stationary tilt+pan scan: place every point in 3D using only the IMU
    orientation (roll, pitch, yaw) at the instant it was measured. The rig
    never translates, so there is no SLAM to do — the head spin lives in `ang`,
    the rig pan/tilt lives in the IMU:

        world = R_body(t) . (R_mount . p_lidar - pivot)

    `mount` (roll, pitch, yaw radians) is the fixed IMU<->lidar rotation. The
    BMI160 is mounted yawed ~90 deg from the LD19, so the tilt the IMU reports
    as roll-about-X is physically the lidar tilting about its left-right axis;
    pre-rotating the lidar points by the mount fixes the geometry (scans stack
    as parallel horizontal lines instead of fanning into a twist). We rotate the
    *points* rather than remapping the IMU so the tilt stays on the roll axis,
    which integrates cleanly to 90 deg — remapping it onto pitch would hit Euler
    gimbal lock at vertical.
    `pivot` (x, y, z metres, lidar frame) is the lever arm: the point the rig
    actually rotates about, correcting any lateral slide if the lidar centre
    sits off the tilt hinge."""
    out = []
    for p in packets:
        out.extend(_project_static(p, get_rot, mount, pivot, rng))
    return np.array(out)

PKT_NS = 3.33e6  # ~3.3ms per 12-point packet (300Hz)

def _project_static(packet, get_rot, mount, pivot, rng):
    """Place one LD19 packet's 12 points into the static world frame:
    world = R_body(t) . (R_mount . p_lidar - pivot). Returns a list of (x,y,z,i)."""
    mr, mp, my = mount
    px, py, pz = pivot
    t_ns, a0, a1, pts = packet
    span = a1 - a0
    if span < 0: span += 360.0
    out = []
    for i, (d, inten) in enumerate(pts):
        if d <= 0: continue
        d_m = d / 1000.0
        if not (rng[0] < d_m < rng[1]): continue
        # Per-point timestamp -> absolute orientation of the rig.
        t_pt = t_ns + (i / (LD19_POINTS - 1)) * PKT_NS
        roll, pitch, yaw = get_rot(t_pt)
        ang = (a0 + span * i / (LD19_POINTS - 1)) * DEG
        lx, ly, lz = -math.sin(ang) * d_m, math.cos(ang) * d_m, 0.0
        # Fixed IMU<->lidar mount rotation, then subtract the pivot lever arm.
        mx, myy, mz = rotate_3d(lx, ly, lz, mr, mp, my)
        mx, myy, mz = mx - px, myy - py, mz - pz
        gx, gy, gz = rotate_3d(mx, myy, mz, roll, pitch, yaw)
        out.append((gx, gy, gz, float(inten)))
    return out

def assemble_static_scans(packets, get_rot, rng=(0.05, 12.0), mount=(0.0, 0.0, 0.0),
                          pivot=(0.0, 0.0, 0.0), min_pts=80):
    """Group packets into per-revolution scans (split on the a0 wrap) and project
    each to the static world frame. Returns a list of ((N,4) cloud, t_ref) for
    the 6-DoF registration path."""
    scans, cur, last_a0 = [], [], -1.0
    def flush(group):
        if len(group) < 20: return
        t_ref = group[len(group) // 2][0]
        out = []
        for p in group:
            out.extend(_project_static(p, get_rot, mount, pivot, rng))
        if len(out) >= min_pts:
            scans.append((np.array(out), t_ref))
    for p in packets:
        a0 = p[1]
        if a0 < last_a0 - 180.0 and cur:
            flush(cur); cur = []
        cur.append(p); last_a0 = a0
    flush(cur)
    return scans

# --- SLAM logic ----------------------------------------------------------

def voxel(p, leaf):
    if len(p) == 0: return p
    key = np.round(p[:, :2] / leaf).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return p[idx]

def mat(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])

def apply_3d(M, p):
    xy = p[:, :2] @ M[:2, :2].T + M[:2, 2]
    return np.column_stack([xy, p[:, 2], p[:, 3]])

def icp(src, tgt, M0, schedule):
    if len(src) < 10 or len(tgt) < 10: return M0
    kd = cKDTree(tgt[:, :2]); M = M0
    for iters, max_dist, trim in schedule:
        for _ in range(iters):
            T = apply_3d(M, src); d, idx = kd.query(T[:, :2], distance_upper_bound=max_dist)
            ok = np.isfinite(d)
            if ok.sum() < 10: break
            ok &= d <= np.percentile(d[ok], trim * 100)
            sp, tp = T[ok, :2], tgt[idx[ok], :2]; cs, ct = sp.mean(0), tp.mean(0)
            H = (sp - cs).T @ (tp - ct); U, _, Vt = np.linalg.svd(H); R = Vt.T @ U.T
            if np.linalg.det(R) < 0: Vt[-1] *= -1; R = Vt.T @ U.T
            dM = np.eye(3); dM[:2, :2], dM[:2, 2] = R, ct - R @ cs; M = dM @ M
    return M

def yaw_of(M): return math.atan2(M[1, 0], M[0, 0])

def run_slam(scans, times, yaw_at):
    poses = [mat(0, 0, 0)]
    recent = [apply_3d(poses[0], scans[0])]
    for i in range(1, len(scans)):
        px, py = poses[-1][0, 2], poses[-1][1, 2]
        dyaw = yaw_at(times[i]) - yaw_at(times[i - 1])
        M0 = mat(px, py, yaw_of(poses[-1]) + dyaw)
        local = voxel(np.vstack(recent[-50:]), 0.03)
        M = icp(voxel(scans[i], 0.04), local, M0, [(30, 0.4, 0.85), (25, 0.15, 0.92)])
        poses.append(M)
        recent.append(apply_3d(M, scans[i]))
    return poses

def close_loop(poses, scans, passes=4):
    """Distribute the end->start yaw+xy drift along the trajectory when the scan
    returns near where it started. A single distribution under-corrects: the
    spread is weighted by cumulative motion, which is only approximate, so one
    pass of a 16 deg loop error leaves a few degrees behind. Re-measure the
    residual end->start transform and redistribute until it converges (~3
    passes). Gated on actual end/start overlap, so a non-returning walk is left
    untouched instead of being bent by a meaningless correction."""
    N, k = len(poses), 45
    if N < 2 * k: return poses
    for _ in range(passes):
        start_ref = voxel(np.vstack([apply_3d(poses[j], scans[j]) for j in range(k)]), 0.025)
        end_glob = voxel(np.vstack([apply_3d(poses[j], scans[j]) for j in range(N - k, N)]), 0.025)
        C = icp(end_glob, start_ref, np.eye(3), [(60, 2.0, 0.9), (50, 0.7, 0.92), (40, 0.25, 0.95)])
        d, _ = cKDTree(start_ref[:, :2]).query(end_glob[:, :2] @ C[:2, :2].T + C[:2, 2])
        if np.mean(d < 0.25) <= 0.3: break          # gate: end must overlap start
        relyaw = math.atan2(C[1, 0], C[0, 0])
        if abs(relyaw) < 0.01 and math.hypot(C[0, 2], C[1, 2]) < 0.02: break   # converged
        trans = [0.0] + [np.hypot(poses[i][0, 2] - poses[i - 1][0, 2], poses[i][1, 2] - poses[i - 1][1, 2]) for i in range(1, N)]
        rot = [0.0] + [abs((yaw_of(poses[i]) - yaw_of(poses[i - 1]) + math.pi) % (2 * math.pi) - math.pi) for i in range(1, N)]
        w = np.array(rot) * 2.0 + np.array(trans); f = np.cumsum(w); f = f / f[-1] if f[-1] > 0 else f
        logC = logm(C).real
        poses = [expm(f[i] * logC) @ poses[i] for i in range(N)]
    return poses

# --- 6-DoF point-to-plane registration (handheld tilt scans) -------------
# The --merge path projects every point with the IMU orientation only and
# assumes the rig never translates. Handheld scans drift 0.5-1 m, which smears
# walls to 10-20 cm. This path keeps the (validated) static per-point geometry
# but treats each lidar revolution as a frame and corrects its residual 6-DoF
# pose against the growing map with point-to-plane ICP. A single tilted ring is
# thin, but the map it registers against is fully 3D, so the fit is well posed
# (KISS-ICP scan-to-map idea). Revisited walls (pan out and back) snap onto the
# existing map instead of doubling.

def voxel3d(p, leaf):
    """Downsample to one point per leaf-sized cube (p is (N,3) or (N,>=3))."""
    if len(p) == 0: return p
    key = np.round(p[:, :3] / leaf).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return p[idx]

def estimate_normals(pts, k=12):
    """Per-point surface normal: smallest-eigenvector of the local kNN scatter."""
    n = len(pts)
    if n < 3: return np.tile([0.0, 0.0, 1.0], (n, 1))
    k = min(k, n)
    _, idx = cKDTree(pts).query(pts, k=k)
    nb = pts[idx]                              # (N, k, 3)
    q = nb - nb.mean(1, keepdims=True)
    C = np.einsum('nki,nkj->nij', q, q)        # (N, 3, 3) scatter matrices
    _, vecs = np.linalg.eigh(C)                # ascending eigenvalues
    return vecs[:, :, 0]                        # normal = smallest-variance axis

def rodrigues(w):
    """Rotation matrix from a small rotation vector (axis * angle)."""
    th = float(np.linalg.norm(w))
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0.0]])
    if th < 1e-12: return np.eye(3) + K
    return np.eye(3) + math.sin(th) / th * K + (1 - math.cos(th)) / (th * th) * (K @ K)

# Free DoF for the registration solve, indexing the 6-vector [wx,wy,wz,tx,ty,tz].
# We lock roll/pitch (wx, wy) and correct only yaw + translation. Two reasons:
# (1) the IMU tilt is trustworthy here (the floor comes out flat), so the real
# error to remove is the handheld translation drift that smears/doubles walls;
# (2) leaving roll/pitch free is unstable. A keyframe is dominated by the floor
# and ceiling, so a free 6-DoF fit tilts the whole keyframe a few degrees to
# overlap the (already smeared) map better, which inflates room height rather
# than sharpening it. Wall normals are horizontal and pin tx/ty; the floor pins
# tz.
DOF_DRIFT = [2, 3, 4, 5]   # wz, tx, ty, tz

def icp_point_to_plane(src, tgt, n_tgt, T0, schedule, axes=DOF_DRIFT):
    """Point-to-plane ICP solving only the `axes` of [wx,wy,wz,tx,ty,tz].
    src/tgt are (N,3); n_tgt are tgt normals. schedule entries are
    (max_dist, iters, trim_fraction). Returns a 4x4 pose."""
    if len(src) < 20 or len(tgt) < 20: return T0
    kd = cKDTree(tgt); T = T0.copy()
    for max_dist, iters, trim in schedule:
        for _ in range(iters):
            S = src @ T[:3, :3].T + T[:3, 3]
            d, idx = kd.query(S, distance_upper_bound=max_dist)
            ok = np.isfinite(d)
            if ok.sum() < 20: break
            ok &= d <= np.percentile(d[ok], trim * 100)
            s, t, nn = S[ok], tgt[idx[ok]], n_tgt[idx[ok]]
            # Linearise R ~ I + [w]_x: residual ((Rs+tr)-t).n -> [s x n | n].[w;tr]
            A = np.hstack([np.cross(s, nn), nn])[:, axes]   # (m, len(axes))
            b = -np.einsum('ij,ij->i', s - t, nn)           # (m,)
            sol, *_ = np.linalg.lstsq(A, b, rcond=None)
            x = np.zeros(6); x[axes] = sol
            dT = np.eye(4); dT[:3, :3] = rodrigues(x[:3]); dT[:3, 3] = x[3:]
            T = dT @ T
    return T

SCHED_ODOM = [(0.6, 15, 0.9), (0.3, 12, 0.92), (0.12, 10, 0.95)]

def merge_icp6(scans, get_rot, key_dt=0.7e9, win=8, anchor_tilt_deg=30.0,
               leaf_model=0.04, leaf_src=0.05):
    """Reconstruct a single coherent room from a handheld tilt+pan scan and return
    the merged (N,4) world cloud plus the per-keyframe poses.

    The rig translates while scanning (~2 m for scan_013), so the plain --merge
    nests later/closer scans inside earlier ones ("room in a room"). We fix it in
    three stages:

    1. Incremental odometry. Group ~`key_dt` ns of revolutions into a keyframe (a
       thin single ring is degenerate; a keyframe spans a slab of the tilt sweep
       and carries real 3D structure). Register each keyframe to the last `win`
       keyframes with point-to-plane ICP, solving yaw + xyz translation and
       keeping roll/pitch from the IMU (gravity is trustworthy; freeing tilt just
       lets keyframes rotate to overlap a smeared map and inflates room height).

    2. Loop closure. If the rig pans out and back, gyro yaw drifts (~39 deg on
       scan_013), so the same walls land twice at a yaw offset ("two rooms, one
       rotated"). When the end submap revisits the start submap, register them
       (wall band only) and distribute the yaw+xy correction along the trajectory
       by cumulative motion. Gated on actual overlap, so a non-returning walk is
       left untouched.

    3. xy anchor smoothing. While the lidar points up at the (flat) ceiling, the
       horizontal pose is unconstrained, so that stretch of the ICP xy is
       garbage. The xy is only trustworthy while walls are in view (low tilt), so
       we interpolate the rig's xy across the high-tilt stretches from the
       low-tilt (< `anchor_tilt_deg`) keyframes, a smooth-motion prior. z and yaw
       are kept from ICP (the ceiling plane constrains height well). Scans with
       little tilt are all anchors, so this is a no-op for non-tilt recordings."""
    keys, cur = [], [0]
    for i in range(1, len(scans)):
        if scans[i][1] - scans[cur[0]][1] > key_dt:
            keys.append(cur); cur = [i]
        else:
            cur.append(i)
    keys.append(cur)
    kf = [np.vstack([scans[j][0] for j in g]) for g in keys]
    kt = np.array([scans[g[len(g) // 2]][1] for g in keys], float)
    ktilt = np.array([np.median([abs(get_rot(scans[j][1])[0]) for j in g]) for g in keys])

    # Stage 1: incremental yaw+translation odometry against a recent local map.
    poses = [np.eye(4)]
    world = [kf[0]]
    for i in range(1, len(kf)):
        local = voxel3d(np.vstack(world[-win:]), leaf_model)
        normals = estimate_normals(local[:, :3])
        T = icp_point_to_plane(voxel3d(kf[i][:, :3], leaf_src), local[:, :3],
                               normals, poses[-1].copy(), SCHED_ODOM)
        poses.append(T)
        w = kf[i][:, :3] @ T[:3, :3].T + T[:3, 3]
        world.append(np.column_stack([w, kf[i][:, 3]]))

    # Stage 2: loop closure when the end revisits the start (wall band only).
    # Distributing the correction by cumulative motion is approximate, so iterate:
    # re-place, re-measure the residual end->start transform, redistribute. This
    # converges the loop yaw to ~0 in 2-3 passes (single pass left ~7 deg).
    K = min(5, len(kf) // 3)
    wb = lambda a: a[(a[:, 2] > 0.3) & (a[:, 2] < 1.6)]
    place = lambda: [np.column_stack([kf[i][:, :3] @ poses[i][:3, :3].T + poses[i][:3, 3], kf[i][:, 3]])
                     for i in range(len(kf))]
    for _ in range(4):
        if K < 2: break
        placed = place()
        start = voxel3d(wb(np.vstack(placed[:K]))[:, :3], leaf_model)
        end = voxel3d(wb(np.vstack(placed[-K:]))[:, :3], leaf_src)
        if len(start) < 50 or len(end) < 50: break
        C = icp_point_to_plane(end, start, estimate_normals(start), np.eye(4),
                               [(2.0, 25, 0.9), (1.0, 25, 0.92), (0.4, 25, 0.95)])
        d, _ = cKDTree(start).query(end @ C[:3, :3].T + C[:3, 3])
        if np.mean(d < 0.25) <= 0.3: break          # gate: end must overlap start
        relyaw = math.atan2(C[1, 0], C[0, 0])
        if abs(relyaw) < 0.01 and math.hypot(C[0, 3], C[1, 3]) < 0.02: break   # converged
        yaws = np.array([math.atan2(p[1, 0], p[0, 0]) for p in poses])
        txy = np.array([[p[0, 3], p[1, 3]] for p in poses])
        dy = np.abs(np.diff(yaws)); dy = np.minimum(dy, 2 * math.pi - dy)
        f = np.concatenate([[0.0], np.cumsum(dy * 2.0 + np.hypot(*np.diff(txy, axis=0).T))])
        f = f / f[-1] if f[-1] > 0 else f
        logC = logm(mat(C[0, 3], C[1, 3], relyaw)).real
        for i in range(len(poses)):
            S = expm(f[i] * logC).real
            Ci = np.eye(4); Ci[:2, :2] = S[:2, :2]; Ci[:2, 3] = S[:2, 2]
            poses[i] = Ci @ poses[i]

    # Stage 3: replace high-tilt xy with an interpolation of the low-tilt anchors.
    tx = np.array([p[0, 3] for p in poses]); ty = np.array([p[1, 3] for p in poses])
    anc = ktilt < math.radians(anchor_tilt_deg)
    if anc.sum() >= 2:
        tx = np.interp(kt, kt[anc], tx[anc])
        ty = np.interp(kt, kt[anc], ty[anc])
    out, smoothed = [kf[0]], [poses[0]]
    for i in range(1, len(kf)):
        T = poses[i].copy(); T[0, 3], T[1, 3] = tx[i], ty[i]
        smoothed.append(T)
        w = kf[i][:, :3] @ T[:3, :3].T + T[:3, 3]
        out.append(np.column_stack([w, kf[i][:, 3]]))
    return np.vstack(out), smoothed

# --- viz3d-stacked merge -------------------------------------------------

def manhattan_yaw(merged, zlo=0.2, zhi=1.6, res=0.05, step=0.5):
    """Yaw (radians) that snaps the dominant walls onto the X/Y axes.

    Scan candidate rotations 0..90 deg; for each, rotate the wall-band points and
    score how tightly their x and y coordinates pile onto a few lines (sum of
    squared histogram-bin counts — tall, sparse peaks mean axis-aligned walls).
    The best angle aligns the dominant wall pair to the grid. Cosmetic: it makes
    the main walls look straight, it does not remove turn-to-turn drift."""
    band = merged[(merged[:, 2] > zlo) & (merged[:, 2] < zhi)][:, :2]
    if len(band) < 100: return 0.0
    band = band - band.mean(0)
    reach = np.abs(band).max() + res
    edges = np.arange(-reach, reach + res, res)
    best_deg, best_score = 0.0, -1.0
    for deg in np.arange(0.0, 90.0, step):
        th = math.radians(deg); c, s = math.cos(th), math.sin(th)
        xr = band[:, 0] * c - band[:, 1] * s
        yr = band[:, 0] * s + band[:, 1] * c
        hx = np.histogram(xr, bins=edges)[0].astype(float)
        hy = np.histogram(yr, bins=edges)[0].astype(float)
        score = (hx * hx).sum() + (hy * hy).sum()
        if score > best_score: best_score, best_deg = score, deg
    return -math.radians(best_deg)

def rotate_yaw(merged, yaw):
    """Rotate an (N,4) cloud about Z by `yaw` radians (keep z, intensity)."""
    c, s = math.cos(yaw), math.sin(yaw)
    x = merged[:, 0] * c - merged[:, 1] * s
    y = merged[:, 0] * s + merged[:, 1] * c
    return np.column_stack([x, y, merged[:, 2], merged[:, 3]])

def merge_viz3d(packets, get_rot, imu_ns, imu_wz):
    """Collapse the viz3d frame sequence into a single cloud.

    Reuses the viz3d geometry verbatim: each lidar revolution is deskewed against
    its own mid-time orientation, tilted into the world by the absolute IMU
    roll/pitch (yaw=0; the 2D SLAM owns yaw), and placed by the 2D SLAM pose
    (xy + yaw). It then stacks every frame instead of emitting them one per
    message. viz3d's rotation handling is clean because each frame's correction
    is tiny and self-contained, and the 2D SLAM (scan-to-map + loop close)
    absorbs the handheld translation/yaw drift — so there's none of merge_icp6's
    3D tilt-vs-translation coupling. Returns (merged (N,4) cloud, t_ns)."""
    scans, times = assemble_scans_3d(packets, get_rot)
    yaw_at = build_yaw(imu_ns, imu_wz)
    poses = close_loop(run_slam(scans, times, yaw_at), scans)
    out = []
    for P, s, t in zip(poses, scans, times):
        roll, pitch, _ = get_rot(t)
        # Absolute IMU tilt, constant across one frame: R = Pitch(Y) . Roll(X)
        # (matches rotate_3d's Roll->Pitch->Yaw order with yaw=0).
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        tilted = np.column_stack([s[:, :3] @ (Ry @ Rx).T, s[:, 3]])
        out.append(apply_3d(P, tilted))
    merged = np.vstack(out)
    # Snap the dominant walls to the X/Y grid so they read as straight/aligned.
    yaw = manhattan_yaw(merged)
    print(f"  Manhattan snap: rotating cloud {math.degrees(yaw):+.1f} deg to align walls")
    return rotate_yaw(merged, yaw), times[0]

# --- ROS 2 formatting ----------------------------------------------------

def ns_to_stamp(ns): return {'sec': int(ns // 1e9), 'nanosec': int(ns % 1e9)}

def build_pc2(points, t_ns):
    data = bytearray()
    for p in points: data += struct.pack('<ffff', p[0], p[1], p[2], p[3])
    return {
        'header': {'stamp': ns_to_stamp(t_ns), 'frame_id': 'map'},
        'height': 1, 'width': len(points),
        'fields': [
            {'name': 'x', 'offset': 0, 'datatype': 7, 'count': 1},
            {'name': 'y', 'offset': 4, 'datatype': 7, 'count': 1},
            {'name': 'z', 'offset': 8, 'datatype': 7, 'count': 1},
            {'name': 'intensity', 'offset': 12, 'datatype': 7, 'count': 1},
        ],
        'is_bigendian': False, 'point_step': 16, 'row_step': 16 * len(points),
        'data': bytes(data), 'is_dense': True
    }

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input", help="input .ldim file")
    ap.add_argument("-o", "--output", help="output .mcap file")
    ap.add_argument("--merge", action="store_true", help="merge all scans into one frame (no SLAM; tripod/fixed-pivot only)")
    ap.add_argument("--merge6", action="store_true",
                    help="merge into one frame with 6-DoF point-to-plane registration "
                         "(corrects handheld translation/drift; use for handheld tilt scans)")
    ap.add_argument("--debug", action="store_true", help="print the IMU orientation trajectory and exit")
    ap.add_argument("--kf", action="store_true",
                    help="use the motion-adaptive Kalman tilt filter instead of the default "
                         "fixed-gain complementary filter")
    ap.add_argument("--mount", default="0,0,-90",
                    help="IMU<->lidar mount rotation 'roll,pitch,yaw' in deg. Default 0,0,-90 "
                         "(BMI160 yawed 90 deg from LD19). Flip to 0,0,90 if the cloud is "
                         "turned 180 deg left-right.")
    ap.add_argument("--pivot", default="0,0,0",
                    help="lever arm 'x,y,z' in metres (lidar frame); tune to remove lateral slide")
    ap.add_argument("--rotate", default="0,0,0",
                    help="view orientation, stage 1: yaw 'rx,ry,rz' in deg (order Rz Ry Rx) in the "
                         "gravity-levelled frame, e.g. '0,0,90' to face a different wall")
    ap.add_argument("--mirror", default="x", choices=["none", "x", "y", "z"],
                    help="view orientation, stage 2: mirror this axis (left-right handedness)")
    ap.add_argument("--postrotate", default="-90,180,0",
                    help="view orientation, stage 3: convert gravity-up (+Z) to the viewer's Y-up "
                         "convention (-90 about X maps +Z->+Y), then the 180 about Y turns the "
                         "room to face the camera. Floor stays as ground, not top-down.")
    args = ap.parse_args()
    suffix = "_merged6.mcap" if args.merge6 else "_static.mcap" if args.merge else "_3d.mcap"
    out = args.output or args.input.rsplit(".", 1)[0] + suffix
    packets, imu_ns, imu_wz, imu_data = load_ldim(args.input)
    print(f"Processing {args.input} with precise 3D deskewing...")
    get_rot = build_orientation_kf(imu_data) if args.kf else build_orientation(imu_data)
    print(f"tilt: {'motion-adaptive Kalman filter' if args.kf else 'complementary filter (default)'}")

    if args.debug:
        t0 = imu_data[0][0]
        rs, ps, ys = [], [], []
        for k in range(0, len(imu_data), max(1, len(imu_data) // 20)):
            t = imu_data[k][0]; r, p, y = get_rot(t)
            rs.append(r); ps.append(p); ys.append(y)
            print(f"  t={(t-t0)/1e9:5.1f}s  roll={math.degrees(r):7.1f}  "
                  f"pitch={math.degrees(p):7.1f}  yaw={math.degrees(y):7.1f}")
        span = lambda a: math.degrees(max(a) - min(a))
        print(f"  -> roll span {span(rs):.0f}°, pitch span {span(ps):.0f}°, yaw span {span(ys):.0f}°")
        return

    def view_transform(merged):
        # View orientation pipeline: rotate -> mirror -> postrotate.
        x, y, z, inten = merged[:, 0], merged[:, 1], merged[:, 2], merged[:, 3]
        x, y, z = rotate_3d(x, y, z, *(float(v) * DEG for v in args.rotate.split(",")))
        if args.mirror != "none":
            x, y, z = (-v if ax == args.mirror else v
                       for ax, v in zip("xyz", (x, y, z)))
        x, y, z = rotate_3d(x, y, z, *(float(v) * DEG for v in args.postrotate.split(",")))
        return np.column_stack([x, y, z, inten])

    def write_single(merged, t_ns):
        print(f"Writing {len(merged)} points to {out}...")
        with open(out, 'wb') as f:
            w = Ros2Writer(f)
            schema = w.register_msgdef('sensor_msgs/msg/PointCloud2', POINT_CLOUD2_SCHEMA)
            msg = build_pc2(merged, t_ns)
            w.write_message(topic="/points", schema=schema, message=msg, log_time=t_ns, publish_time=t_ns)
            w.finish()

    mount = tuple(float(v) * DEG for v in args.mount.split(","))
    pivot = tuple(float(v) for v in args.pivot.split(","))

    if args.merge6:
        # Handheld tilt scan: stack the viz3d frame sequence into one cloud.
        # Each revolution keeps viz3d's clean per-frame rotation (relative deskew
        # + absolute IMU tilt) and is placed by the 2D SLAM pose, which absorbs
        # the handheld translation/yaw drift the plain --merge can't. (The older
        # 3D point-to-plane path, merge_icp6, is kept available but coupled tilt
        # into translation and distorted the cloud.)
        print("Stacking viz3d frames into a single cloud (2D SLAM)...")
        merged, t_ns = merge_viz3d(packets, get_rot, imu_ns, imu_wz)
        write_single(view_transform(merged), t_ns)
    elif args.merge:
        # Stationary tilt+pan scan: pure IMU-orientation projection, no SLAM.
        # The rig doesn't translate, so 2D ICP on tilted slices would only
        # invent bogus motion and smear the cloud.
        print("Merging all points by IMU orientation (no SLAM)...")
        merged = assemble_static_3d(packets, get_rot, mount=mount, pivot=pivot)
        write_single(view_transform(merged), packets[0][0])
    else:
        scans, times = assemble_scans_3d(packets, get_rot)
        print("SLAM alignment...")
        yaw_at = build_yaw(imu_ns, imu_wz)
        poses = close_loop(run_slam(scans, times, yaw_at), scans)
        print("Writing scans to MCAP sequence...")
        with open(out, 'wb') as f:
            w = Ros2Writer(f)
            schema = w.register_msgdef('sensor_msgs/msg/PointCloud2', POINT_CLOUD2_SCHEMA)
            for i, (P, s) in enumerate(zip(poses, scans)):
                r, p, _ = get_rot(times[i])
                scan_global_tilt = []
                for pt in s:
                    gx, gy, gz = rotate_3d(pt[0], pt[1], pt[2], r, p, 0.0)
                    scan_global_tilt.append((gx, gy, gz, pt[3]))
                aligned_scan = apply_3d(P, np.array(scan_global_tilt))
                msg = build_pc2(aligned_scan, times[i])
                w.write_message(topic="/points", schema=schema, message=msg, 
                                log_time=times[i], publish_time=times[i])
            w.finish()
        print(f"Done. Wrote {len(poses)} frames to {out}")
    print("Done.")

if __name__ == "__main__":
    main()
