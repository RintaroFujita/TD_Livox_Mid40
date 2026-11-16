"""High level Mid-40 client: discovery, handshake, heartbeat and point streaming.

Only depends on the standard library + numpy, so the same code can later run
inside TouchDesigner's bundled Python.

    from livox_mid40 import Mid40
    with Mid40() as lidar:          # auto-discovers the first LiDAR
        lidar.start()
        pts = lidar.get_points()    # (N, 4) float32: x, y, z [m], reflectivity
"""

from __future__ import annotations

import ipaddress
import logging
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass, field

import numpy as np

from . import protocol as P

log = logging.getLogger("livox_mid40")


class LivoxError(RuntimeError):
    pass


def _udp_socket(port: int, bind_ip: str = "", rcvbuf: int | None = None,
                broadcast: bool = False) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if broadcast:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    if rcvbuf:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
    if sys.platform == "win32" and hasattr(socket, "SIO_UDP_CONNRESET"):
        # Stop Windows from raising WSAECONNRESET after ICMP port-unreachable.
        s.ioctl(socket.SIO_UDP_CONNRESET, False)
    s.bind((bind_ip, port))
    return s


def discover(timeout: float = 3.0, first_only: bool = False) -> list[P.BroadcastInfo]:
    """Listen for LiDAR discovery broadcasts (UDP 55000).

    A LiDAR only broadcasts while it is *not* connected to a host.
    """
    found: dict[str, P.BroadcastInfo] = {}
    sock = _udp_socket(P.BROADCAST_PORT, broadcast=True)
    sock.settimeout(0.2)
    end = time.time() + timeout
    try:
        while time.time() < end:
            try:
                buf, (ip, _port) = sock.recvfrom(1024)
            except socket.timeout:
                continue
            try:
                fr = P.parse_frame(buf)
            except P.ProtocolError as e:
                log.warning("ignored packet from %s on UDP %d: %s (%s)",
                            ip, P.BROADCAST_PORT, e, buf[:40].hex())
                continue
            if fr.cmd_type == P.CMD_TYPE_MSG and fr.key == (P.SET_GENERAL, P.GEN_BROADCAST):
                info = P.parse_broadcast(fr, ip)
                if info.broadcast_code not in found:
                    log.info("found %s  code=%s  ip=%s", info.device_name, info.broadcast_code, ip)
                found[info.broadcast_code] = info
                if first_only:
                    break
    finally:
        sock.close()
    return list(found.values())


def _local_ip_for(target_ip: str) -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((target_ip, P.LIDAR_CMD_PORT))
        return s.getsockname()[0]
    finally:
        s.close()


@dataclass
class LidarStatus:
    connected: bool = False
    device_type: int = -1
    broadcast_code: str = ""
    firmware: str = ""
    work_state: int = -1
    status_code: int = 0
    data_type: int = -1
    packets: int = 0
    points: int = 0
    dropped_packets: int = 0
    last_heartbeat: float = 0.0
    extra: dict = field(default_factory=dict)

    @property
    def work_state_name(self) -> str:
        return P.WORK_STATES.get(self.work_state, f"unknown({self.work_state})")


class Mid40:
    """Livox Mid-40 client.

    Args:
        lidar_ip: LiDAR IP. ``None`` = auto-discover via broadcast.
        broadcast_code: pick a specific unit when several are on the network.
        host_ip: this PC's IP on the LiDAR subnet. ``None`` = auto.
        buffer_points: size of the rolling point buffer returned by
            :meth:`get_points` (Mid-40 = 100k pts/s, so 10_000 ~= 100 ms,
            similar to one "frame").
    """

    HEARTBEAT_INTERVAL = 0.5

    def __init__(self, lidar_ip: str | None = None, broadcast_code: str | None = None,
                 host_ip: str | None = None, data_port: int = 56001, cmd_port: int = 56000,
                 imu_port: int = 56002, buffer_points: int = 10_000,
                 discover_timeout: float = 5.0):
        self.lidar_ip = lidar_ip
        self.broadcast_code = broadcast_code
        self.host_ip = host_ip
        self.data_port = data_port
        self.cmd_port = cmd_port
        self.imu_port = imu_port
        self.discover_timeout = discover_timeout
        self.status = LidarStatus()

        self._seq = 0
        self._cmd_sock: socket.socket | None = None
        self._data_sock: socket.socket | None = None
        self._threads: list[threading.Thread] = []
        self._stop = threading.Event()
        self._pending: dict[tuple[int, int], tuple[threading.Event, list]] = {}
        self._pending_lock = threading.Lock()
        self._send_lock = threading.Lock()

        self._buf = np.zeros((buffer_points, 4), dtype=np.float32)
        self._buf_idx = 0
        self._buf_full = False
        self._written = 0  # total points ever written into the ring buffer
        self._buf_lock = threading.Lock()
        self._last_hdr: P.DataHeader | None = None

    # ------------------------------------------------------------ lifecycle
    def __enter__(self) -> "Mid40":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def connect(self) -> None:
        if self.lidar_ip is None:
            log.info("discovering LiDAR (%.0fs)...", self.discover_timeout)
            devices = discover(self.discover_timeout, first_only=self.broadcast_code is None)
            if self.broadcast_code:
                devices = [d for d in devices if d.broadcast_code == self.broadcast_code]
            if not devices:
                raise LivoxError(
                    "No Livox broadcast received on UDP 55000. Check cable/power, that this PC "
                    "has an IP on the LiDAR subnet, and that Livox Viewer is not already "
                    "connected. Mid-40 ships in dynamic-IP (DHCP) mode: on a direct cable run "
                    "livox_mid40.dhcp.MiniDHCPServer (or livox_monitor.py --dhcp).")
            dev = devices[0]
            self.lidar_ip = dev.ip
            self.status.device_type = dev.device_type
            self.status.broadcast_code = dev.broadcast_code

        if self.host_ip is None:
            self.host_ip = _local_ip_for(self.lidar_ip)
        same_net = (ipaddress.ip_network(f"{self.host_ip}/24", strict=False)
                    == ipaddress.ip_network(f"{self.lidar_ip}/24", strict=False))
        if not same_net:
            log.warning("host IP %s is not in the LiDAR subnet of %s; data may not arrive",
                        self.host_ip, self.lidar_ip)

        self._stop.clear()
        self._cmd_sock = _udp_socket(self.cmd_port)
        self._data_sock = _udp_socket(self.data_port, rcvbuf=8 * 1024 * 1024)
        self._cmd_sock.settimeout(0.2)
        self._data_sock.settimeout(0.2)
        self._spawn(self._cmd_loop, "livox-cmd")
        self._spawn(self._data_loop, "livox-data")

        log.info("handshake -> %s (host %s, data %d, cmd %d)",
                 self.lidar_ip, self.host_ip, self.data_port, self.cmd_port)
        ack = self._request(P.SET_GENERAL, P.GEN_HANDSHAKE,
                            P.handshake_payload(self.host_ip, self.data_port,
                                                self.cmd_port, self.imu_port))
        if ack.payload[:1] != b"\x00":
            raise LivoxError(f"handshake rejected (ret={ack.payload[:1].hex()})")
        self.status.connected = True
        self._spawn(self._heartbeat_loop, "livox-heartbeat")

        try:
            self.query_info()
        except LivoxError as e:
            log.warning("query device info failed: %s", e)

    def close(self) -> None:
        if self.status.connected:
            try:
                self.stop()
                self._request(P.SET_GENERAL, P.GEN_DISCONNECT, timeout=0.5)
            except LivoxError:
                pass
        self.status.connected = False
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        self._threads.clear()
        for s in (self._cmd_sock, self._data_sock):
            if s:
                s.close()
        self._cmd_sock = self._data_sock = None

    # ------------------------------------------------------------ commands
    def query_info(self) -> str:
        ack = self._request(P.SET_GENERAL, P.GEN_QUERY_INFO)
        if len(ack.payload) >= 5:
            self.status.firmware = ".".join(f"{b:02d}" for b in ack.payload[1:5])
        return self.status.firmware

    def start(self) -> None:
        """Spin up (if needed) and start streaming points."""
        self._expect_ok(self._request(P.SET_GENERAL, P.GEN_SAMPLING, b"\x01"), "start sampling")

    def stop(self) -> None:
        self._expect_ok(self._request(P.SET_GENERAL, P.GEN_SAMPLING, b"\x00"), "stop sampling")

    def set_cartesian(self, cartesian: bool = True) -> None:
        self._expect_ok(self._request(P.SET_GENERAL, P.GEN_COORDINATE,
                                      b"\x00" if cartesian else b"\x01"), "set coordinate")

    def read_extrinsic(self) -> dict[str, float]:
        ack = self._request(P.SET_LIDAR, P.LIDAR_READ_EXTRINSIC)
        roll, pitch, yaw, x, y, z = struct.unpack_from("<fffiii", ack.payload, 1)
        return {"roll": roll, "pitch": pitch, "yaw": yaw, "x_mm": x, "y_mm": y, "z_mm": z}

    # ------------------------------------------------------------ data access
    def get_points(self, nonzero: bool = True, count: int | None = None) -> np.ndarray:
        """Latest points, oldest first, (N, 4) float32 [x, y, z, r].

        ``count`` limits to the newest ``count`` points (default: whole buffer).
        """
        cap = self._buf.shape[0]
        with self._buf_lock:
            avail = cap if self._buf_full else self._buf_idx
            n = avail if count is None else max(0, min(int(count), avail))
            start = self._buf_idx - n
            if start >= 0:
                pts = self._buf[start:self._buf_idx].copy()
            else:
                pts = np.concatenate((self._buf[start:], self._buf[:self._buf_idx]))
        if nonzero:
            pts = pts[np.any(pts[:, :3] != 0, axis=1)]
        return pts

    @property
    def written(self) -> int:
        """Total number of points written so far (sequence for get_points_since)."""
        return self._written

    def get_points_since(self, seq: int) -> tuple[np.ndarray, int]:
        """Points written after sequence number ``seq`` (oldest first, zeros
        included) and the new sequence number. Start with ``seq=0``; if more
        than the buffer size arrived in between, only the newest are returned."""
        cap = self._buf.shape[0]
        with self._buf_lock:
            total = self._written
            n = min(max(0, total - seq), cap)
            start = self._buf_idx - n
            if start >= 0:
                pts = self._buf[start:self._buf_idx].copy()
            else:
                pts = np.concatenate((self._buf[start:], self._buf[:self._buf_idx]))
        return pts, total

    @property
    def last_header(self) -> P.DataHeader | None:
        return self._last_hdr

    # ------------------------------------------------------------ internals
    def _spawn(self, fn, name: str) -> None:
        t = threading.Thread(target=fn, name=name, daemon=True)
        t.start()
        self._threads.append(t)

    @staticmethod
    def _expect_ok(ack: P.Frame, what: str) -> None:
        if ack.payload[:1] != b"\x00":
            raise LivoxError(f"{what} failed (ret={ack.payload[:1].hex()})")

    def _request(self, cmd_set: int, cmd_id: int, payload: bytes = b"",
                 timeout: float = 1.0, retries: int = 3) -> P.Frame:
        if self._cmd_sock is None:
            raise LivoxError("not connected")
        key = (cmd_set, cmd_id)
        for _ in range(retries):
            ev, slot = threading.Event(), []
            with self._pending_lock:
                self._pending[key] = (ev, slot)
            with self._send_lock:
                self._seq = (self._seq + 1) & 0xFFFF
                pkt = P.build_frame(cmd_set, cmd_id, payload, self._seq)
                self._cmd_sock.sendto(pkt, (self.lidar_ip, P.LIDAR_CMD_PORT))
            if ev.wait(timeout):
                return slot[0]
        with self._pending_lock:
            self._pending.pop(key, None)
        raise LivoxError(f"no ACK for cmd 0x{cmd_set:02X}/0x{cmd_id:02X} from {self.lidar_ip}")

    def _cmd_loop(self) -> None:
        while not self._stop.is_set():
            try:
                buf, _addr = self._cmd_sock.recvfrom(1024)
            except (socket.timeout, OSError):
                continue
            try:
                fr = P.parse_frame(buf)
            except P.ProtocolError as e:
                log.debug("bad cmd packet: %s", e)
                continue
            if fr.cmd_type == P.CMD_TYPE_ACK:
                with self._pending_lock:
                    waiter = self._pending.pop(fr.key, None)
                if waiter:
                    waiter[1].append(fr)
                    waiter[0].set()
            elif fr.key == (P.SET_GENERAL, P.GEN_ABNORMAL_STATUS) and len(fr.payload) >= 4:
                (code,) = struct.unpack_from("<I", fr.payload)
                self.status.status_code = code
                log.warning("abnormal status pushed: 0x%08X %s", code, P.decode_status_code(code))

    def _heartbeat_loop(self) -> None:
        while not self._stop.wait(self.HEARTBEAT_INTERVAL):
            try:
                ack = self._request(P.SET_GENERAL, P.GEN_HEARTBEAT, timeout=0.5, retries=1)
            except LivoxError:
                log.warning("heartbeat timeout")
                continue
            p = ack.payload
            if len(p) >= 2:
                self.status.work_state = p[1]
            if len(p) >= 7:
                (self.status.status_code,) = struct.unpack_from("<I", p, 3)
            self.status.last_heartbeat = time.time()

    def _data_loop(self) -> None:
        cap = self._buf.shape[0]
        while not self._stop.is_set():
            try:
                buf = self._data_sock.recv(2048)
            except (socket.timeout, OSError):
                continue
            try:
                hdr, pts = P.parse_points_xyz(buf)
            except (P.ProtocolError, struct.error, ValueError) as e:
                self.status.dropped_packets += 1
                log.debug("bad data packet: %s", e)
                continue
            self._last_hdr = hdr
            self.status.data_type = hdr.data_type
            self.status.packets += 1
            self.status.points += len(pts)
            n = len(pts)
            with self._buf_lock:
                i = self._buf_idx
                end = i + n
                if end <= cap:
                    self._buf[i:end] = pts
                else:
                    k = cap - i
                    self._buf[i:] = pts[:k]
                    self._buf[: n - k] = pts[k:]
                self._buf_idx = end % cap
                if end >= cap:
                    self._buf_full = True
                self._written += n
