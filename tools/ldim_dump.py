#!/usr/bin/env python3
"""Inspect a .ldim recording and optionally dump points / IMU samples to CSV.

The .ldim format is documented in lidar_view.c. Summary:
  16-byte header (magic 'LDIM', version, start_unix_ns)
  repeated 12-byte record header (monotonic ns, type, len) + payload
    type 0 → raw 47-byte LD19 packet
    type 1 → 12-byte mount-corrected IMU sample (gyro then accel, int16 LE)

Example:
  tools/ldim_dump.py recordings/scan_001.ldim
  tools/ldim_dump.py recordings/scan_001.ldim --points points.csv --imu imu.csv
"""
import argparse, struct, sys
from datetime import datetime, timezone

MAGIC          = 0x4D49444C
TYPE_LD19      = 0
TYPE_IMU       = 1
LD19_PKT_LEN   = 47
LD19_POINTS    = 12

# IMU LSBs (default BMI160 ranges set by lidar_view.c).
ACC_LSB_PER_G   = 16384.0
GYR_LSB_PER_DPS = 16.4


def parse_ld19(payload):
    speed, start = struct.unpack_from('<HH', payload, 2)
    end,   ts    = struct.unpack_from('<HH', payload, 6 + 12 * 3)
    pts = []
    for i in range(LD19_POINTS):
        d, inten = struct.unpack_from('<HB', payload, 6 + i * 3)
        pts.append((d, inten))
    return speed, start / 100.0, end / 100.0, ts, pts


def parse_imu(payload):
    return struct.unpack('<hhhhhh', payload)


def iter_records(fp):
    hdr = fp.read(16)
    if len(hdr) != 16:
        raise SystemExit('truncated header')
    magic, ver, _ = struct.unpack_from('<IHH', hdr, 0)
    start_unix_ns, = struct.unpack_from('<Q', hdr, 8)
    if magic != MAGIC:
        raise SystemExit(f'bad magic 0x{magic:08x} (expected {MAGIC:#x})')
    yield 'header', ver, start_unix_ns
    while True:
        h = fp.read(12)
        if not h:
            return
        if len(h) < 12:
            print(f'warning: truncated record header at EOF ({len(h)} bytes) — stopping',
                  file=sys.stderr)
            return
        ns, type_, _, length = struct.unpack('<QBBH', h)
        payload = fp.read(length)
        if len(payload) < length:
            print(f'warning: truncated payload at EOF ({len(payload)}/{length}) — stopping',
                  file=sys.stderr)
            return
        yield 'rec', ns, type_, payload


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('file')
    ap.add_argument('--points', metavar='CSV',
                    help='write per-point CSV: ns,scan,angle_deg,dist_mm,intensity')
    ap.add_argument('--imu', metavar='CSV',
                    help='write per-IMU CSV: ns,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g')
    args = ap.parse_args()

    pt_csv = open(args.points, 'w') if args.points else None
    if pt_csv:
        pt_csv.write('ns,scan,angle_deg,dist_mm,intensity\n')
    imu_csv = open(args.imu, 'w') if args.imu else None
    if imu_csv:
        imu_csv.write('ns,gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g\n')

    n_pkt = n_imu = 0
    last_ns = 0
    start_unix_ns = 0
    ver = 0
    with open(args.file, 'rb') as fp:
        for rec in iter_records(fp):
            if rec[0] == 'header':
                _, ver, start_unix_ns = rec
                continue
            _, ns, type_, payload = rec
            last_ns = ns
            if type_ == TYPE_LD19:
                n_pkt += 1
                if pt_csv:
                    _, a0, a1, _, pts = parse_ld19(payload)
                    span = a1 - a0
                    if span < 0:
                        span += 360.0
                    for i, (d, inten) in enumerate(pts):
                        a = a0 + span * i / (LD19_POINTS - 1)
                        pt_csv.write(f'{ns},{n_pkt},{a:.2f},{d},{inten}\n')
            elif type_ == TYPE_IMU:
                n_imu += 1
                if imu_csv:
                    gx, gy, gz, ax, ay, az = parse_imu(payload)
                    imu_csv.write(
                        f'{ns},'
                        f'{gx/GYR_LSB_PER_DPS:.3f},{gy/GYR_LSB_PER_DPS:.3f},{gz/GYR_LSB_PER_DPS:.3f},'
                        f'{ax/ACC_LSB_PER_G:.4f},{ay/ACC_LSB_PER_G:.4f},{az/ACC_LSB_PER_G:.4f}\n')

    if pt_csv:  pt_csv.close()
    if imu_csv: imu_csv.close()

    dur_s = last_ns / 1e9
    started = datetime.fromtimestamp(start_unix_ns / 1e9, tz=timezone.utc).astimezone()
    print(f'file:     {args.file}')
    print(f'version:  {ver}')
    print(f'started:  {started.isoformat(timespec="seconds")}')
    print(f'duration: {dur_s:.2f} s')
    print(f'LD19:     {n_pkt} packets ({n_pkt/dur_s:.1f}/s)' if dur_s > 0 else f'LD19:     {n_pkt} packets')
    print(f'IMU:      {n_imu} samples  ({n_imu/dur_s:.1f}/s)' if dur_s > 0 else f'IMU:      {n_imu} samples')
    print(f'~LD19 points: {n_pkt * LD19_POINTS}')
    if args.points: print(f'wrote {args.points}')
    if args.imu:    print(f'wrote {args.imu}')


if __name__ == '__main__':
    main()
