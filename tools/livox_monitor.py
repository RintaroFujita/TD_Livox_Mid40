"""Connect to a Livox Mid-40 and print device info + live point statistics.

    python tools/livox_monitor.py                 # discover + stream for 10 s
    python tools/livox_monitor.py --discover      # only list LiDARs on the network
    python tools/livox_monitor.py --ip 192.168.1.180 --seconds 0 --save scan.ply
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from livox_mid40 import Mid40, discover, protocol as P  # noqa: E402
from livox_mid40.dhcp import MiniDHCPServer  # noqa: E402


def save_ply(path: str, pts: np.ndarray) -> None:
    with open(path, "wb") as f:
        f.write((f"ply\nformat binary_little_endian 1.0\nelement vertex {len(pts)}\n"
                 "property float x\nproperty float y\nproperty float z\n"
                 "property float intensity\nend_header\n").encode())
        f.write(pts.astype("<f4").tobytes())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--discover", action="store_true", help="only listen for broadcasts")
    ap.add_argument("--ip", help="LiDAR IP (default: auto-discover)")
    ap.add_argument("--host-ip", help="this PC's IP on the LiDAR subnet (default: auto)")
    ap.add_argument("--seconds", type=float, default=10, help="stream duration, 0 = until Ctrl+C")
    ap.add_argument("--buffer", type=int, default=10_000, help="points kept in the rolling buffer")
    ap.add_argument("--save", help="write the last buffer to a .ply file on exit")
    ap.add_argument("--dhcp", action="store_true",
                    help="run a mini DHCP server (LiDAR in dynamic-IP mode, direct cable)")
    ap.add_argument("--dhcp-server-ip", default="192.168.1.50", help="this PC's wired IP")
    ap.add_argument("--dhcp-lease-ip", default="192.168.1.100", help="IP handed to the LiDAR")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(message)s", datefmt="%H:%M:%S")

    dhcp = None
    if a.dhcp:
        dhcp = MiniDHCPServer(a.dhcp_server_ip, a.dhcp_lease_ip, mac_prefix="34:d2:62").start()
    try:
        return run(a)
    finally:
        if dhcp:
            dhcp.stop()


def run(a: argparse.Namespace) -> int:
    discover_timeout = 15 if a.dhcp else 5  # a freshly leased LiDAR needs a few seconds
    if a.discover:
        devs = discover(timeout=discover_timeout)
        if not devs:
            print(f"No LiDAR broadcast received in {discover_timeout} s."
                  + ("" if a.dhcp else " If the LiDAR is in dynamic-IP mode, add --dhcp."))
            return 1
        for d in devs:
            print(f"{d.device_name:8s}  code={d.broadcast_code}  ip={d.ip}")
        return 0

    lidar = Mid40(lidar_ip=a.ip, host_ip=a.host_ip, buffer_points=a.buffer,
                  discover_timeout=discover_timeout)
    try:
        lidar.connect()
        st = lidar.status
        print(f"\nConnected: {P.DEVICE_TYPES.get(st.device_type, '-')}  code={st.broadcast_code or '-'}"
              f"  ip={lidar.lidar_ip}  host={lidar.host_ip}  firmware={st.firmware or '?'}")
        try:
            print("Extrinsic:", lidar.read_extrinsic())
        except Exception as e:  # not critical
            print("Extrinsic: n/a", e)
        lidar.set_cartesian(True)
        lidar.start()
        print("Sampling started (motor spin-up can take a few seconds)\n")

        t0 = time.time()
        last_pts, last_t = 0, t0
        while a.seconds <= 0 or time.time() - t0 < a.seconds:
            time.sleep(1.0)
            now = time.time()
            rate = (st.points - last_pts) / (now - last_t)
            last_pts, last_t = st.points, now
            pts = lidar.get_points()
            if len(pts):
                d = np.linalg.norm(pts[:, :3], axis=1)
                rng = f"range {d.min():5.2f}-{d.max():6.2f} m  median {np.median(d):5.2f} m"
                c = pts[:, :3].mean(axis=0)
                ctr = f"centroid ({c[0]:+.2f},{c[1]:+.2f},{c[2]:+.2f})"
            else:
                rng, ctr = "no valid points yet", ""
            sc = P.decode_status_code(st.status_code)
            warn = {k: v for k, v in sc.items() if v and k not in ("time_sync",)}
            print(f"[{now - t0:5.1f}s] state={st.work_state_name:12s} {rate:8.0f} pts/s  "
                  f"pkts={st.packets:7d}  valid={len(pts):6d}  {rng}  {ctr}"
                  + (f"  WARN {warn}" if warn else ""))
    except KeyboardInterrupt:
        pass
    finally:
        if a.save:
            pts = lidar.get_points()
            save_ply(a.save, pts)
            print(f"saved {len(pts)} points -> {a.save}")
        lidar.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
