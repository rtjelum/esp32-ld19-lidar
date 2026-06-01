#!/usr/bin/env python3
"""Convert a .ldim recording to MCAP with ROS 2 LaserScan + PointCloud2 + Imu messages.

Output topics:
  /scan    sensor_msgs/msg/LaserScan    one per LD19 revolution (~10 Hz)
  /points  sensor_msgs/msg/PointCloud2  same data as /scan, projected to (x,y,0,intensity)
  /imu     sensor_msgs/msg/Imu          one per IMU sample (~70 Hz)

LaserScan is what SLAM tools (slam_toolbox, Cartographer) consume.
PointCloud2 is what generic point-cloud viewers (PRBonn lidar-visualizer,
RViz PointCloud2 display) consume.

Frames:
  lidar  the LD19 sensor frame
  imu    the BMI160 mount-corrected frame (per IMU_MOUNT_*_SIGN in lidar_view.c)

Suitable for ingestion by Foxglove Studio (drag-drop) or ROS 2 SLAM stacks
(slam_toolbox, Cartographer) once you publish a static transform between
the two frames matching your physical mount.

Requires: mcap, mcap-ros2-support
Usage:    tools/.venv/bin/python tools/ldim_to_mcap.py recordings/scan_001.ldim
"""
import argparse
import math
import struct
import sys
from pathlib import Path

try:
    from mcap_ros2.writer import Writer as Ros2Writer
except ImportError:
    sys.exit("missing dependency — run: "
             "python3 -m venv tools/.venv && tools/.venv/bin/pip install mcap mcap-ros2-support")

# --- .ldim format --------------------------------------------------------

MAGIC        = 0x4D49444C
TYPE_LD19    = 0
TYPE_IMU     = 1
LD19_POINTS  = 12

# Matches lidar_view.c IMU range setup (BMI160 defaults: ±2 g, ±2000 °/s).
ACC_LSB_PER_G   = 16384.0
GYR_LSB_PER_DPS = 16.4
G_TO_MS2        = 9.80665
DEG_TO_RAD      = math.pi / 180.0

# LD19 datasheet ranges.
LD19_RANGE_MIN_M = 0.02
LD19_RANGE_MAX_M = 12.0

FRAME_LIDAR  = "lidar"
FRAME_IMU    = "imu"
TOPIC_SCAN   = "/scan"
TOPIC_POINTS = "/points"
TOPIC_IMU    = "/imu"

# --- ROS 2 schema text (with all dependencies inlined) -------------------

LASER_SCAN_SCHEMA = """\
std_msgs/Header header
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""

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

IMU_SCHEMA = """\
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
"""


def iter_records(fp):
    hdr = fp.read(16)
    if len(hdr) != 16:
        raise SystemExit("truncated header")
    magic, _, _ = struct.unpack_from('<IHH', hdr)
    start_unix_ns, = struct.unpack_from('<Q', hdr, 8)
    if magic != MAGIC:
        raise SystemExit(f"bad magic 0x{magic:08x}")
    yield 'header', start_unix_ns
    while True:
        h = fp.read(12)
        if not h:
            return
        if len(h) < 12:
            print(f"warning: truncated record header at EOF ({len(h)})", file=sys.stderr)
            return
        ns, type_, _, length = struct.unpack('<QBBH', h)
        payload = fp.read(length)
        if len(payload) < length:
            print(f"warning: truncated payload at EOF ({len(payload)}/{length})", file=sys.stderr)
            return
        yield 'rec', ns, type_, payload


def parse_ld19(payload):
    """Returns (start_deg, end_deg, [(dist_mm, intensity), ...])."""
    start = struct.unpack_from('<H', payload, 4)[0] / 100.0
    end   = struct.unpack_from('<H', payload, 6 + 12 * 3)[0] / 100.0
    pts = [struct.unpack_from('<HB', payload, 6 + i * 3) for i in range(LD19_POINTS)]
    return start, end, pts


def ns_to_stamp(ns):
    return {'sec': int(ns // 1_000_000_000), 'nanosec': int(ns % 1_000_000_000)}


def build_laserscan(bins_mm, bins_int, t_start_ns, t_end_ns):
    n = len(bins_mm)
    ranges = []
    intensities = []
    for d_mm, inten in zip(bins_mm, bins_int):
        if d_mm <= 0:
            ranges.append(float('inf'))
            intensities.append(0.0)
        else:
            ranges.append(d_mm / 1000.0)
            intensities.append(float(inten))
    scan_time = max(0.0, (t_end_ns - t_start_ns) / 1e9)
    return {
        'header': {'stamp': ns_to_stamp(t_start_ns), 'frame_id': FRAME_LIDAR},
        'angle_min':       0.0,
        'angle_max':       2.0 * math.pi - (2.0 * math.pi / n),
        'angle_increment': 2.0 * math.pi / n,
        'time_increment':  scan_time / n if n else 0.0,
        'scan_time':       scan_time,
        'range_min':       LD19_RANGE_MIN_M,
        'range_max':       LD19_RANGE_MAX_M,
        'ranges':          ranges,
        'intensities':     intensities,
    }


def build_pointcloud2(bins_mm, bins_int, t_start_ns):
    """Project per-degree polar bins to (x, y, z=0, intensity) points.

    Matches lidar_view.c's convention: angle 0° → +y (forward), 90° → -x (left).
    PointField datatype 7 = FLOAT32. Only valid bins (d > 0) are emitted.
    """
    pts_bytes = bytearray()
    n = 0
    for bin_deg in range(len(bins_mm)):
        d_mm = bins_mm[bin_deg]
        if d_mm <= 0:
            continue
        ang = bin_deg * math.pi / 180.0
        d_m = d_mm / 1000.0
        x = -math.sin(ang) * d_m
        y =  math.cos(ang) * d_m
        pts_bytes += struct.pack('<ffff', x, y, 0.0, float(bins_int[bin_deg]))
        n += 1
    return {
        'header': {'stamp': ns_to_stamp(t_start_ns), 'frame_id': FRAME_LIDAR},
        'height':         1,
        'width':          n,
        'fields': [
            {'name': 'x',         'offset': 0,  'datatype': 7, 'count': 1},
            {'name': 'y',         'offset': 4,  'datatype': 7, 'count': 1},
            {'name': 'z',         'offset': 8,  'datatype': 7, 'count': 1},
            {'name': 'intensity', 'offset': 12, 'datatype': 7, 'count': 1},
        ],
        'is_bigendian':   False,
        'point_step':     16,
        'row_step':       16 * n,
        'data':           bytes(pts_bytes),
        'is_dense':       True,
    }


def build_imu(gx, gy, gz, ax, ay, az, abs_ns):
    return {
        'header': {'stamp': ns_to_stamp(abs_ns), 'frame_id': FRAME_IMU},
        'orientation':                  {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        # ROS convention: -1 in [0,0] means "orientation not provided".
        'orientation_covariance':        [-1.0] + [0.0] * 8,
        'angular_velocity': {
            'x': gx / GYR_LSB_PER_DPS * DEG_TO_RAD,
            'y': gy / GYR_LSB_PER_DPS * DEG_TO_RAD,
            'z': gz / GYR_LSB_PER_DPS * DEG_TO_RAD,
        },
        'angular_velocity_covariance':   [0.0] * 9,
        'linear_acceleration': {
            'x': ax / ACC_LSB_PER_G * G_TO_MS2,
            'y': ay / ACC_LSB_PER_G * G_TO_MS2,
            'z': az / ACC_LSB_PER_G * G_TO_MS2,
        },
        'linear_acceleration_covariance': [0.0] * 9,
    }


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('input', help='input .ldim file')
    ap.add_argument('-o', '--output', help='output .mcap (default: input.mcap)')
    args = ap.parse_args()

    out_path = args.output or str(Path(args.input).with_suffix('.mcap'))

    bins_mm  = [0] * 360
    bins_int = [0] * 360
    scan_first_ns = None
    scan_last_ns  = None
    last_start    = -1.0   # for revolution-wrap detection

    n_scan = 0
    n_imu  = 0
    start_unix_ns = 0

    with open(args.input, 'rb') as fp, open(out_path, 'wb') as out:
        w = Ros2Writer(out)
        scan_schema   = w.register_msgdef('sensor_msgs/msg/LaserScan',   LASER_SCAN_SCHEMA)
        points_schema = w.register_msgdef('sensor_msgs/msg/PointCloud2', POINT_CLOUD2_SCHEMA)
        imu_schema    = w.register_msgdef('sensor_msgs/msg/Imu',         IMU_SCHEMA)

        for rec in iter_records(fp):
            if rec[0] == 'header':
                start_unix_ns = rec[1]
                continue
            _, ns, type_, payload = rec
            abs_ns = start_unix_ns + ns

            if type_ == TYPE_LD19:
                a0, a1, pts = parse_ld19(payload)
                if scan_first_ns is None:
                    scan_first_ns = abs_ns
                # New revolution when start_angle wraps backwards.
                if a0 < last_start - 180.0:
                    scan_msg = build_laserscan(bins_mm, bins_int, scan_first_ns, abs_ns)
                    w.write_message(topic=TOPIC_SCAN, schema=scan_schema,
                                    message=scan_msg, log_time=scan_first_ns,
                                    publish_time=scan_first_ns)
                    pc_msg = build_pointcloud2(bins_mm, bins_int, scan_first_ns)
                    w.write_message(topic=TOPIC_POINTS, schema=points_schema,
                                    message=pc_msg, log_time=scan_first_ns,
                                    publish_time=scan_first_ns)
                    n_scan += 1
                    bins_mm  = [0] * 360
                    bins_int = [0] * 360
                    scan_first_ns = abs_ns
                last_start = a0
                span = a1 - a0
                if span < 0:
                    span += 360.0
                for i, (d, inten) in enumerate(pts):
                    a = a0 + span * i / (LD19_POINTS - 1)
                    a %= 360.0
                    b = int(a)
                    if 0 <= b < 360 and d > 0:
                        bins_mm[b]  = d
                        bins_int[b] = inten
                scan_last_ns = abs_ns

            elif type_ == TYPE_IMU:
                gx, gy, gz, ax, ay, az = struct.unpack('<hhhhhh', payload)
                msg = build_imu(gx, gy, gz, ax, ay, az, abs_ns)
                w.write_message(topic=TOPIC_IMU, schema=imu_schema,
                                message=msg, log_time=abs_ns,
                                publish_time=abs_ns)
                n_imu += 1

        # Final partial revolution, if any.
        if scan_first_ns is not None and any(bins_mm):
            t_end = scan_last_ns or scan_first_ns
            scan_msg = build_laserscan(bins_mm, bins_int, scan_first_ns, t_end)
            w.write_message(topic=TOPIC_SCAN, schema=scan_schema,
                            message=scan_msg, log_time=scan_first_ns,
                            publish_time=scan_first_ns)
            pc_msg = build_pointcloud2(bins_mm, bins_int, scan_first_ns)
            w.write_message(topic=TOPIC_POINTS, schema=points_schema,
                            message=pc_msg, log_time=scan_first_ns,
                            publish_time=scan_first_ns)
            n_scan += 1

        w.finish()

    print(f"wrote {out_path}")
    print(f"  /scan:   {n_scan} LaserScan messages")
    print(f"  /points: {n_scan} PointCloud2 messages")
    print(f"  /imu:    {n_imu} Imu messages")


if __name__ == '__main__':
    main()
