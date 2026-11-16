"""Livox SDK1 communication protocol (Mid-40 / Mid-70 / Horizon / Tele-15 / Avia).

Pure-Python implementation of the wire format so that no C++ SDK is needed.
Reference: "Livox SDK Communication Protocol" (SDK1, protocol v1).

Control frame layout (little endian):

    sof(1)=0xAA | version(1)=1 | length(2) | cmd_type(1) | seq_num(2) | crc16(2)
    cmd_set(1) | cmd_id(1) | payload(N) | crc32(4)

    crc16 covers the first 7 bytes, crc32 covers everything before it.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass

import numpy as np

# ---------------------------------------------------------------- ports
BROADCAST_PORT = 55000  # LiDAR -> host, 1 Hz discovery broadcast
LIDAR_CMD_PORT = 65000  # host -> LiDAR, control commands

# ---------------------------------------------------------------- frame
SOF = 0xAA
PROTOCOL_VERSION = 1
HEADER_LEN = 9          # sof .. crc16
MIN_FRAME_LEN = HEADER_LEN + 2 + 4

CMD_TYPE_CMD = 0x00
CMD_TYPE_ACK = 0x01
CMD_TYPE_MSG = 0x02

# cmd_set 0x00: general
SET_GENERAL = 0x00
GEN_BROADCAST = 0x00
GEN_HANDSHAKE = 0x01
GEN_QUERY_INFO = 0x02
GEN_HEARTBEAT = 0x03
GEN_SAMPLING = 0x04
GEN_COORDINATE = 0x05
GEN_DISCONNECT = 0x06
GEN_ABNORMAL_STATUS = 0x07

# cmd_set 0x01: LiDAR
SET_LIDAR = 0x01
LIDAR_SET_MODE = 0x00
LIDAR_WRITE_EXTRINSIC = 0x01
LIDAR_READ_EXTRINSIC = 0x02

DEVICE_TYPES = {
    0: "Livox Hub",
    1: "Mid-40",
    2: "Tele-15",
    3: "Horizon",
    6: "Mid-70",
    7: "Avia",
}

WORK_STATES = {
    0: "initializing",
    1: "normal",
    2: "power-saving",
    3: "standby",
    4: "error",
}

# ---------------------------------------------------------------- CRC
_CRC16_SEED = 0x4C49
_CRC32_SEED = 0x564F580A


def _make_crc16_table() -> list[int]:
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0x8408 if c & 1 else c >> 1
        table.append(c)
    return table


_CRC16_TABLE = _make_crc16_table()


def crc16(data: bytes) -> int:
    """CRC-16/MCRF4XX (poly 0x1021 reflected) with Livox seed 0x4C49."""
    crc = _CRC16_SEED
    for b in data:
        crc = (crc >> 8) ^ _CRC16_TABLE[(crc ^ b) & 0xFF]
    return crc


def crc32(data: bytes) -> int:
    """CRC-32 (IEEE, reflected) with Livox seed 0x564F580A."""
    return zlib.crc32(data, _CRC32_SEED) & 0xFFFFFFFF


# ---------------------------------------------------------------- frames
@dataclass
class Frame:
    cmd_type: int
    seq: int
    cmd_set: int
    cmd_id: int
    payload: bytes

    @property
    def key(self) -> tuple[int, int]:
        return self.cmd_set, self.cmd_id


class ProtocolError(ValueError):
    pass


def build_frame(cmd_set: int, cmd_id: int, payload: bytes = b"", seq: int = 0,
                cmd_type: int = CMD_TYPE_CMD) -> bytes:
    length = HEADER_LEN + 2 + len(payload) + 4
    head = struct.pack("<BBHBH", SOF, PROTOCOL_VERSION, length, cmd_type, seq & 0xFFFF)
    head += struct.pack("<H", crc16(head))
    body = head + struct.pack("<BB", cmd_set, cmd_id) + payload
    return body + struct.pack("<I", crc32(body))


def parse_frame(buf: bytes) -> Frame:
    if len(buf) < MIN_FRAME_LEN:
        raise ProtocolError(f"frame too short ({len(buf)} bytes)")
    sof, ver, length, cmd_type, seq, hcrc = struct.unpack_from("<BBHBHH", buf, 0)
    if sof != SOF:
        raise ProtocolError(f"bad SOF 0x{sof:02X}")
    if length > len(buf):
        raise ProtocolError(f"truncated frame ({len(buf)} < {length})")
    if crc16(buf[:7]) != hcrc:
        raise ProtocolError("header CRC16 mismatch")
    (fcrc,) = struct.unpack_from("<I", buf, length - 4)
    if crc32(buf[: length - 4]) != fcrc:
        raise ProtocolError("frame CRC32 mismatch")
    cmd_set, cmd_id = buf[9], buf[10]
    return Frame(cmd_type, seq, cmd_set, cmd_id, bytes(buf[11 : length - 4]))


# ---------------------------------------------------------------- payloads
@dataclass
class BroadcastInfo:
    broadcast_code: str
    device_type: int
    ip: str

    @property
    def device_name(self) -> str:
        return DEVICE_TYPES.get(self.device_type, f"unknown({self.device_type})")


def parse_broadcast(frame: Frame, ip: str) -> BroadcastInfo:
    code = frame.payload[:16].split(b"\x00", 1)[0].decode("ascii", "replace")
    dev_type = frame.payload[16] if len(frame.payload) > 16 else -1
    return BroadcastInfo(code, dev_type, ip)


def handshake_payload(host_ip: str, data_port: int, cmd_port: int, imu_port: int) -> bytes:
    ip_bytes = bytes(int(p) for p in host_ip.split("."))
    return ip_bytes + struct.pack("<HHH", data_port, cmd_port, imu_port)


def decode_status_code(code: int) -> dict[str, int]:
    """Bit fields of the 32-bit LiDAR status code (Mid-40)."""
    return {
        "temp": code & 0x3,               # 0 normal, 1 high/low, 2 extremely high/low
        "voltage": (code >> 2) & 0x3,
        "motor": (code >> 4) & 0x3,       # 0 normal, 1 warning, 2 error
        "dirty": (code >> 6) & 0x3,
        "firmware": (code >> 8) & 0x1,    # 1 = firmware abnormal, upgrade needed
        "pps": (code >> 9) & 0x1,
        "device": (code >> 10) & 0x1,     # 1 = end of service life warning
        "fan": (code >> 11) & 0x1,
        "self_heating": (code >> 12) & 0x1,
        "ptp": (code >> 13) & 0x1,
        "time_sync": (code >> 14) & 0x7,
        "system": (code >> 30) & 0x3,     # 0 normal, 1 warning, 2 error
    }


# ---------------------------------------------------------------- point data
DATA_HEADER_LEN = 18
# data_type -> numpy dtype of one point
POINT_DTYPES = {
    0: np.dtype([("x", "<i4"), ("y", "<i4"), ("z", "<i4"), ("r", "u1")]),           # cartesian, mm
    1: np.dtype([("depth", "<u4"), ("theta", "<u2"), ("phi", "<u2"), ("r", "u1")]),  # spherical
}


@dataclass
class DataHeader:
    version: int
    slot_id: int
    lidar_id: int
    status_code: int
    timestamp_type: int
    data_type: int
    timestamp: int  # ns (for PTP / no-sync) or raw 8 bytes as uint64


def parse_data_header(buf: bytes) -> DataHeader:
    ver, slot, lid, _rsv, status, ts_type, dtype, ts = struct.unpack_from("<BBBBIBBQ", buf, 0)
    return DataHeader(ver, slot, lid, status, ts_type, dtype, ts)


def parse_points_xyz(buf: bytes) -> tuple[DataHeader, np.ndarray]:
    """Return (header, Nx4 float32 array [x, y, z, reflectivity]) in metres.

    Zero points (no return) are kept; filter with ``pts[:, :3].any(axis=1)``.
    Livox frame: +X forward, +Y left, +Z up.
    """
    hdr = parse_data_header(buf)
    dt = POINT_DTYPES.get(hdr.data_type)
    if dt is None:
        raise ProtocolError(f"unsupported data_type {hdr.data_type}")
    raw = np.frombuffer(buf, dtype=dt, offset=DATA_HEADER_LEN,
                        count=(len(buf) - DATA_HEADER_LEN) // dt.itemsize)
    out = np.empty((raw.shape[0], 4), dtype=np.float32)
    if hdr.data_type == 0:
        out[:, 0] = raw["x"] * 1e-3
        out[:, 1] = raw["y"] * 1e-3
        out[:, 2] = raw["z"] * 1e-3
    else:
        d = raw["depth"] * 1e-3
        theta = np.radians(raw["theta"] * 0.01)  # zenith angle
        phi = np.radians(raw["phi"] * 0.01)      # azimuth
        out[:, 0] = d * np.sin(theta) * np.cos(phi)
        out[:, 1] = d * np.sin(theta) * np.sin(phi)
        out[:, 2] = d * np.cos(theta)
    out[:, 3] = raw["r"]
    return hdr, out
