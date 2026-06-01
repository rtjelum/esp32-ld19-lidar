#!/usr/bin/env python3
"""Quick viewer for a .ldim recording — overlays every LD19 scan point in the
sensor frame. Good for eyeballing a capture before running the full floorplan
pipeline (tools/ldim_to_floorplan.py), which adds gyro deskew + ICP.

Examples:
  tools/view_ldim.py recordings/scan_001.ldim
  tools/view_ldim.py recordings/scan_001.ldim --out scan_001.png --max-m 6
"""
import argparse
import math
import struct
import sys

import numpy as np

MAGIC = 0x4D49444C
TYPE_LD19 = 0
LD19_POINTS = 12


def load_points(path, max_m):
    """Return Nx2 array of (x, y) metres from all LD19 packets, plus IMU count."""
    xs, ys = [], []
    imu_count = 0
    with open(path, "rb") as f:
        hdr = f.read(16)
        if len(hdr) != 16 or struct.unpack_from("<I", hdr, 0)[0] != MAGIC:
            sys.exit("not a .ldim file (bad magic)")
        while True:
            rh = f.read(12)
            if len(rh) < 12:
                break
            ns, typ, _, length = struct.unpack("<QBBH", rh)
            payload = f.read(length)
            if len(payload) < length:
                break
            if typ != TYPE_LD19 or length != 47:
                if typ == 1:
                    imu_count += 1
                continue
            start = struct.unpack_from("<H", payload, 4)[0] / 100.0
            end = struct.unpack_from("<H", payload, 42)[0] / 100.0
            span = (end - start) % 360.0
            for i in range(LD19_POINTS):
                dist = struct.unpack_from("<H", payload, 6 + i * 3)[0]
                if dist == 0 or dist >= 5000:
                    continue
                a = math.radians(start + span * i / (LD19_POINTS - 1))
                d = dist / 1000.0
                if d > max_m:
                    continue
                xs.append(d * math.cos(a))
                ys.append(d * math.sin(a))
    return np.column_stack([xs, ys]) if xs else np.empty((0, 2)), imu_count


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="input .ldim file")
    ap.add_argument("--out", help="save PNG instead of opening a window")
    ap.add_argument("--max-m", type=float, default=8.0, help="clip range in metres (default 8)")
    args = ap.parse_args()

    import matplotlib
    if args.out:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    pts, imu_count = load_points(args.input, args.max_m)
    print(f"{args.input}: {len(pts)} points, {imu_count} IMU samples")
    if len(pts) == 0:
        sys.exit("no points to show")

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(pts[:, 0], pts[:, 1], s=1, c="#3cf", alpha=0.4, linewidths=0)
    ax.scatter([0], [0], c="#f44", s=40, marker="^", label="sensor")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title(args.input)
    ax.grid(True, alpha=0.2)
    ax.legend(loc="upper right")
    if args.out:
        fig.savefig(args.out, dpi=130, bbox_inches="tight")
        print(f"wrote {args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
