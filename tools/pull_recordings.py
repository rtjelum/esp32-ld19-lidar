#!/usr/bin/env python3
"""Pull .ldim recordings off the ESP32 LiDAR over WiFi.

The firmware serves them from the onboard FAT partition:
  GET    /rec            -> {"recording":bool,"free":N,"files":[{name,size}]}
  GET    /rec/<file>     -> the .ldim bytes
  DELETE /rec/<file>     -> delete it

By default this downloads every file that's missing locally (or whose size
differs) into ./recordings/, skipping the one currently being recorded.

Examples:
  tools/pull_recordings.py                      # from http://lidar.local
  tools/pull_recordings.py --host 192.168.86.21
  tools/pull_recordings.py --delete             # delete each file after a verified pull
  tools/pull_recordings.py --list               # just show what's on the device
"""
import argparse
import sys
import urllib.request
import urllib.error
import json
from pathlib import Path


def api(host, path, method="GET", timeout=30):
    url = f"http://{host}{path}"
    req = urllib.request.Request(url, method=method)
    return urllib.request.urlopen(req, timeout=timeout)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="lidar.local", help="device host/IP (default lidar.local)")
    ap.add_argument("--dest", default="recordings", help="local dir (default ./recordings)")
    ap.add_argument("--list", action="store_true", help="only list what's on the device")
    ap.add_argument("--delete", action="store_true",
                    help="delete each file on the device after a size-verified download")
    ap.add_argument("--force", action="store_true", help="re-download even if local copy matches")
    args = ap.parse_args()

    try:
        with api(args.host, "/rec") as r:
            info = json.load(r)
    except urllib.error.URLError as e:
        sys.exit(f"cannot reach {args.host}: {e}")

    files = info.get("files", [])
    print(f"{args.host}: {len(files)} file(s), {info.get('free', 0)/1e6:.1f} MB free"
          + ("  [RECORDING IN PROGRESS]" if info.get("recording") else ""))
    for f in files:
        print(f"  {f['name']:<20} {f['size']:>10} bytes")
    if args.list:
        return

    dest = Path(args.dest)
    dest.mkdir(parents=True, exist_ok=True)
    pulled = 0
    for f in files:
        name, size = f["name"], f["size"]
        local = dest / name
        if local.exists() and local.stat().st_size == size and not args.force:
            print(f"  = {name} (up to date)")
            continue
        print(f"  ↓ {name} ...", end="", flush=True)
        try:
            with api(args.host, f"/rec/{name}", timeout=120) as r:
                data = r.read()
        except urllib.error.HTTPError as e:
            print(f" skipped ({e.code} {e.reason})")
            continue
        if len(data) != size:
            print(f" SIZE MISMATCH (got {len(data)}, expected {size}) — not saving")
            continue
        local.write_bytes(data)
        pulled += 1
        print(f" {len(data)} bytes -> {local}")
        if args.delete:
            try:
                api(args.host, f"/rec/{name}", method="DELETE").read()
                print(f"    deleted {name} on device")
            except urllib.error.URLError as e:
                print(f"    delete failed: {e}")

    print(f"done — {pulled} file(s) pulled into {dest}/")


if __name__ == "__main__":
    main()
