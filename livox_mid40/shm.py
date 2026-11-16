"""Shared-memory layout between the LiDAR server process and TouchDesigner.

Two blocks, both created by the server:

``livox_mid40_ctl``  (client -> server): requested processing parameters and a
    client heartbeat. The server exits when the heartbeat goes stale.

``livox_mid40_data`` (server -> client): status header + two data slots
    (double buffering). The server writes the inactive slot, then flips
    ``active``. A reader copies the active slot and checks its ``seq`` did not
    change while copying.
"""

from __future__ import annotations

import struct
import time
from multiprocessing import shared_memory

import numpy as np

DEFAULT_PREFIX = "livox_mid40"  # use another prefix per LiDAR / for tests


def ctl_name(prefix: str = DEFAULT_PREFIX) -> str:
    return prefix + "_ctl"


def data_name(prefix: str = DEFAULT_PREFIX) -> str:
    return prefix + "_data"


CTL_NAME, DATA_NAME = ctl_name(), data_name()
MAGIC = 0x4C4D3430  # 'LM40'

MAX_RES = 512
MAX_PIX = MAX_RES * MAX_RES
MAX_PTS = 262_144  # >= max(grid pixels, raw ring buffer)
N_CHANS = 8

MODES = ("grid", "raw")
COORDS = ("td", "livox")
COLORS = ("depth", "reflectivity", "white")
STATES = ("starting", "connecting", "streaming", "retrying", "stopped")

# ---------------------------------------------------------------- control block
# magic, seq, heartbeat, mode, res, fov, fill, coord, seconds, color, near, far, quit
_CTL = struct.Struct("<IId I I f I I f I f f I")
CTL_SIZE = 256


def ctl_pack(*, heartbeat: float, mode: str, res: int, fov: float, fill: int, coord: str,
             seconds: float, color: str, near: float, far: float, quit: bool = False,
             seq: int = 0) -> bytes:
    return _CTL.pack(MAGIC, seq & 0xFFFFFFFF, heartbeat, MODES.index(mode),
                     min(int(res), MAX_RES), float(fov), int(fill), COORDS.index(coord),
                     float(seconds), COLORS.index(color), float(near), float(far), int(quit))


def ctl_unpack(buf) -> dict | None:
    v = _CTL.unpack_from(buf, 0)
    if v[0] != MAGIC:
        return None
    return dict(seq=v[1], heartbeat=v[2], mode=MODES[v[3]], res=v[4], fov=v[5], fill=v[6],
                coord=COORDS[v[7]], seconds=v[8], color=COLORS[v[9]], near=v[10], far=v[11],
                quit=bool(v[12]))


# ---------------------------------------------------------------- data block
# magic, seq, active, state, packets, server_beat, grid_hz, pid, error[256], code[32], ip[16], fw[16]
_HDR = struct.Struct("<I Q I I Q d f I 256s 32s 16s 16s")
HDR_SIZE = 1024
# per slot: seq, mode, res, n
_SLOT = struct.Struct("<Q I I I")
SLOT_HDR = 64
GRID_BYTES = MAX_PIX * 4 * 4
VALID_BYTES = MAX_PIX
COLS_BYTES = N_CHANS * MAX_PTS * 4
SLOT_SIZE = SLOT_HDR + GRID_BYTES + VALID_BYTES + COLS_BYTES
DATA_SIZE = HDR_SIZE + 2 * SLOT_SIZE


def _slot_off(i: int) -> int:
    return HDR_SIZE + i * SLOT_SIZE


def _slot_arrays(buf, i: int):
    o = _slot_off(i) + SLOT_HDR
    grid = np.ndarray((MAX_PIX, 4), np.float32, buf, o)
    valid = np.ndarray((MAX_PIX,), np.uint8, buf, o + GRID_BYTES)
    cols = np.ndarray((N_CHANS, MAX_PTS), np.float32, buf, o + GRID_BYTES + VALID_BYTES)
    return grid, valid, cols


def _s(b: bytes) -> str:
    return b.split(b"\0", 1)[0].decode("utf-8", "replace")


def _b(s: str, n: int) -> bytes:
    return s.encode("utf-8", "replace")[: n - 1]


def open_or_create(name: str, size: int) -> shared_memory.SharedMemory:
    try:
        return shared_memory.SharedMemory(name=name, create=True, size=size)
    except FileExistsError:  # stale block still held by a client: reuse it
        shm = shared_memory.SharedMemory(name=name)
        if shm.size < size:
            raise RuntimeError(f"shared memory {name} exists with a smaller size")
        return shm


class DataWriter:
    """Server side of the data block."""

    def __init__(self, shm: shared_memory.SharedMemory, pid: int):
        self.shm, self.buf = shm, shm.buf
        self.seq, self.active = 0, 0
        self.status = dict(state="starting", packets=0, grid_hz=0.0, error="", code="",
                           ip="", fw="")
        self.pid = pid
        self.write_header()

    def write_header(self) -> None:
        st = self.status
        _HDR.pack_into(self.buf, 0, MAGIC, self.seq, self.active, STATES.index(st["state"]),
                       int(st["packets"]), time.time(), float(st["grid_hz"]), self.pid,
                       _b(st["error"], 256), _b(st["code"], 32), _b(st["ip"], 16),
                       _b(st["fw"], 16))

    def begin(self):
        """Arrays of the inactive slot to fill: (grid (MAX_PIX,4), valid, cols)."""
        return _slot_arrays(self.buf, 1 - self.active)

    def commit(self, mode: str, res: int, n: int) -> None:
        slot = 1 - self.active
        self.seq += 1
        _SLOT.pack_into(self.buf, _slot_off(slot), self.seq, MODES.index(mode), res, n)
        self.active = slot
        self.write_header()


class Frame:
    __slots__ = ("seq", "mode", "res", "n", "grid", "valid", "cols", "slot")


class DataReader:
    """Client side of the data block (zero-copy views; copy what you keep)."""

    def __init__(self, shm: shared_memory.SharedMemory):
        self.shm, self.buf = shm, shm.buf

    def header(self) -> dict | None:
        v = _HDR.unpack_from(self.buf, 0)
        if v[0] != MAGIC:
            return None
        return dict(seq=v[1], active=v[2], state=STATES[v[3]], packets=v[4], server_beat=v[5],
                    grid_hz=round(v[6], 1), pid=v[7], error=_s(v[8]), code=_s(v[9]),
                    ip=_s(v[10]), firmware=_s(v[11]))

    def slot_seq(self, slot: int) -> int:
        return _SLOT.unpack_from(self.buf, _slot_off(slot))[0]

    def frame(self) -> Frame | None:
        h = self.header()
        if h is None or h["seq"] == 0:
            return None
        slot = h["active"]
        seq, mode, res, n = _SLOT.unpack_from(self.buf, _slot_off(slot))
        grid, valid, cols = _slot_arrays(self.buf, slot)
        f = Frame()
        f.seq, f.mode, f.res, f.n = seq, MODES[mode], res, n
        f.grid = grid[: res * res].reshape(res, res, 4) if res else grid[:0].reshape(0, 0, 4)
        f.valid = valid[: res * res].reshape(res, res) if res else valid[:0].reshape(0, 0)
        f.cols = cols[:, :n]
        f.slot = slot
        return f
