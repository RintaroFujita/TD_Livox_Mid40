"""LiDAR server process: Mid-40 connection + point processing -> shared memory.

Runs outside TouchDesigner so that packet reception (1000 pkt/s) and grid
building never compete with TD for Python's GIL. TouchDesigner launches it
automatically (see td_bridge.py); it can also be started by hand:

    python -m livox_mid40.server --dhcp --client-timeout 0

Processing parameters (mode, resolution, ...) are read every tick from the
control block written by the client. With ``--client-timeout N`` the server
exits N seconds after the client stops sending heartbeats (TD closed).
"""

from __future__ import annotations

import argparse
import logging
import os
import threading
import time

import numpy as np

from . import engine as E
from . import shm as S
from .dhcp import MiniDHCPServer
from .lidar import Mid40

log = logging.getLogger("livox_mid40.server")

RING_SECONDS = 2.0


class Connection:
    """Keeps a Mid-40 connected (DHCP, discovery, handshake, auto-reconnect)."""

    def __init__(self, lidar_ip: str | None, host_ip: str, use_dhcp: bool,
                 data_port: int = 56001, cmd_port: int = 56000):
        self.lidar_ip, self.host_ip, self.use_dhcp = lidar_ip, host_ip, use_dhcp
        self.data_port, self.cmd_port = data_port, cmd_port
        self.lidar: Mid40 | None = None
        self.dhcp: MiniDHCPServer | None = None
        self.state, self.error = "starting", ""
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="livox-conn", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            pending = None
            try:
                if self.use_dhcp and self.dhcp is None:
                    self.dhcp = MiniDHCPServer(self.host_ip, mac_prefix="34:d2:62").start()
                self.state = "connecting"
                pending = Mid40(lidar_ip=self.lidar_ip, host_ip=self.host_ip,
                                buffer_points=int(RING_SECONDS * E.POINTS_PER_SEC),
                                discover_timeout=5.0, data_port=self.data_port,
                                cmd_port=self.cmd_port, imu_port=self.data_port + 1)
                pending.connect()
                pending.set_cartesian(True)
                pending.start()
                self.lidar = pending
                self.state, self.error = "streaming", ""
                log.info("streaming from %s", pending.lidar_ip)
                while not self._stop.wait(1.0):
                    if time.time() - pending.status.last_heartbeat > 3.0:
                        raise RuntimeError("heartbeat lost")
            except Exception as e:
                self.state, self.error = "retrying", str(e)
                log.warning("%s", e)
            finally:
                self.lidar = None
                if pending is not None:
                    try:
                        pending.close()
                    except Exception:
                        pass
            self._stop.wait(2.0)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3.0)
        if self.dhcp:
            self.dhcp.stop()


DEFAULT_CTL = dict(mode="grid", res=128, fov=E.MID40_FOV_DEG, fill=1, coord="td",
                   seconds=0.5, color="depth", near=1.0, far=10.0)


def serve(lidar_ip: str | None, host_ip: str, use_dhcp: bool, rate: float,
          client_timeout: float, data_port: int = 56001, cmd_port: int = 56000,
          shm_prefix: str = S.DEFAULT_PREFIX) -> None:
    ctl_shm = S.open_or_create(S.ctl_name(shm_prefix), S.CTL_SIZE)
    data_shm = S.open_or_create(S.data_name(shm_prefix), S.DATA_SIZE)
    # always reset: a reused block may still hold quit=1 from the previous server
    ctl_shm.buf[: S._CTL.size] = S.ctl_pack(heartbeat=time.time(), **DEFAULT_CTL)
    writer = S.DataWriter(data_shm, os.getpid())
    conn = Connection(lidar_ip, host_ip, use_dhcp, data_port, cmd_port)
    acc = E.GridAccumulator()
    hz = 0.0
    log.info("server pid %d, shared memory ready (%s_ctl/_data)", os.getpid(), shm_prefix)
    try:
        while True:
            t0 = time.perf_counter()
            c = S.ctl_unpack(ctl_shm.buf) or dict(DEFAULT_CTL, heartbeat=time.time(), quit=False)
            if c["quit"]:
                log.info("quit requested by client")
                break
            if client_timeout > 0 and time.time() - c["heartbeat"] > client_timeout:
                log.info("client heartbeat lost for %.0fs, exiting", client_timeout)
                break

            lidar = conn.lidar
            st = writer.status
            st.update(state=conn.state, error=conn.error, grid_hz=hz)
            if lidar is not None:
                st.update(packets=lidar.status.packets, code=lidar.status.broadcast_code,
                          ip=lidar.lidar_ip or "", fw=lidar.status.firmware)
                grid, valid, cols = writer.begin()
                if c["mode"] == "grid":
                    res = max(8, min(int(c["res"]), S.MAX_RES))
                    acc.update(lidar, res, c["fov"], c["seconds"], RING_SECONDS)
                    g, v = acc.snapshot(c["seconds"], c["fill"], c["coord"])
                    grid[: res * res] = g.reshape(-1, 4)
                    valid[: res * res] = v.reshape(-1)
                    n = E.chop_columns(g[v], cols, c["color"], c["near"], c["far"])
                else:
                    res = 0
                    sec = min(c["seconds"], RING_SECONDS)
                    p = lidar.get_points(nonzero=True, count=int(sec * E.POINTS_PER_SEC))
                    p = E.convert(p[-S.MAX_PTS:], c["coord"])
                    n = E.chop_columns(p, cols, c["color"], c["near"], c["far"])
                writer.commit(c["mode"], res, n)
            else:
                writer.write_header()

            dt = time.perf_counter() - t0
            hz = 0.9 * hz + 0.1 * (1.0 / max(dt, 1.0 / rate))
            time.sleep(max(0.0, 1.0 / rate - dt))
    finally:
        conn.close()
        writer.status.update(state="stopped")
        writer.write_header()
        ctl_shm.close()
        data_shm.close()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lidar-ip", default="", help="LiDAR IP (default: auto-discover)")
    ap.add_argument("--host-ip", default="192.168.1.50", help="this PC's wired IP")
    ap.add_argument("--dhcp", action="store_true", help="hand the LiDAR an IP (direct cable / switch)")
    ap.add_argument("--rate", type=float, default=30.0, help="processing rate [Hz]")
    ap.add_argument("--client-timeout", type=float, default=10.0,
                    help="exit when the client heartbeat is older than this (0 = never)")
    ap.add_argument("--data-port", type=int, default=56001)
    ap.add_argument("--cmd-port", type=int, default=56000)
    ap.add_argument("--shm-prefix", default=S.DEFAULT_PREFIX,
                    help="shared-memory name prefix (one per LiDAR / test)")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s", datefmt="%H:%M:%S")
    serve(a.lidar_ip or None, a.host_ip, a.dhcp, a.rate, a.client_timeout,
          a.data_port, a.cmd_port, a.shm_prefix)


if __name__ == "__main__":
    main()
