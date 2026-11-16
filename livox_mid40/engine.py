"""Point-cloud processing shared by the server process (no TouchDesigner code).

- coordinate conversion (Livox -> TouchDesigner)
- Kinect-style grid: pinhole projection, incremental accumulation, hole fill
- turbo colormap and CHOP column packing
"""

from __future__ import annotations

import time

import numpy as np

POINTS_PER_SEC = 100_000  # Mid-40
MID40_FOV_DEG = 38.4

CHOP_CHANNELS = ("tx", "ty", "tz", "r", "d", "cr", "cg", "cb")


# ---------------------------------------------------------------- coordinates
def convert(p: np.ndarray, coord: str) -> np.ndarray:
    """Livox (..., 4) [x,y,z,r 0-255] -> requested coords with r in 0-1.

    coord="td":    TouchDesigner (+X right, +Y up, -Z forward: the LiDAR looks
                   down -Z like TD's default camera).
    coord="livox": raw Livox (+X forward, +Y left, +Z up).
    """
    out = np.empty_like(p)
    if coord == "td":
        out[..., 0] = -p[..., 1]
        out[..., 1] = p[..., 2]
        out[..., 2] = -p[..., 0]
    else:
        out[..., :3] = p[..., :3]
    out[..., 3] = p[..., 3] * (1.0 / 255.0)
    return out


# ---------------------------------------------------------------- grid
# A res x res pinhole "depth camera" covering fov_deg. Row 0 is the *bottom*
# of the image (TouchDesigner texture order), column 0 the LiDAR's left.

def project(p: np.ndarray, res: int, t: float) -> tuple[np.ndarray, np.ndarray]:
    """Livox points (x > 0) -> (flat pixel index, mask of points inside the grid)."""
    inv_x = 1.0 / p[:, 0]
    col = ((0.5 - p[:, 1] * inv_x / (2 * t)) * res).astype(np.int32)  # +Y is left
    row = ((0.5 + p[:, 2] * inv_x / (2 * t)) * res).astype(np.int32)  # +Z is up
    ok = (col >= 0) & (col < res) & (row >= 0) & (row < res)
    return row[ok] * res + col[ok], ok


_NEIGHBOURS = ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1))


def fill_holes(xyzr: np.ndarray, valid: np.ndarray, t: float, passes: int) -> None:
    """In-place hole fill on a (res, res, 4) Livox-frame grid. Filled pixels sit
    on their own pixel ray at the neighbour's forward depth."""
    res = valid.shape[0]
    filled = np.zeros_like(valid)
    for _ in range(max(0, int(passes))):
        if valid.all():
            break
        src_x, src_v = xyzr.copy(), valid.copy()
        for dy, dx in _NEIGHBOURS:
            ys = slice(max(dy, 0), res + min(dy, 0))
            yd = slice(max(-dy, 0), res + min(-dy, 0))
            xs = slice(max(dx, 0), res + min(dx, 0))
            xd = slice(max(-dx, 0), res + min(-dx, 0))
            take = src_v[ys, xs] & ~valid[yd, xd]
            if not take.any():
                continue
            dst = xyzr[yd, xd]
            dst[take] = src_x[ys, xs][take]
            valid[yd, xd] |= take
            filled[yd, xd] |= take
    if filled.any():
        centres = (np.arange(res, dtype=np.float32) + 0.5) / res
        u_c = (0.5 - centres) * 2 * t  # per column
        v_c = (centres - 0.5) * 2 * t  # per row
        ry, rx = np.nonzero(filled)
        depth = xyzr[ry, rx, 0]
        xyzr[ry, rx, 1] = u_c[rx] * depth
        xyzr[ry, rx, 2] = v_c[ry] * depth


class GridAccumulator:
    """Persistent grid updated only with *new* points; each pixel remembers
    when it was last hit and expires after ``seconds``. Cost per update does
    not depend on the integration time (10 s / 512 px stays cheap)."""

    def __init__(self):
        self.key = None

    def update(self, lidar, res: int, fov_deg: float, seconds: float,
               ring_seconds: float) -> None:
        key = (id(lidar), res, fov_deg)
        if key != self.key:  # (re)start: seed from what the ring buffer still holds
            self.key = key
            self.t = float(np.tan(np.radians(fov_deg) * 0.5))
            self.res = res
            self.xyzr = np.zeros((res * res, 4), dtype=np.float32)
            self.stamp = np.full(res * res, -np.inf)
            self.seq = max(0, lidar.written - int(min(seconds, ring_seconds) * POINTS_PER_SEC))
        p, self.seq = lidar.get_points_since(self.seq)
        p = p[p[:, 0] > 0.05]  # drops zero (no return) points too
        if len(p):
            idx, ok = project(p, self.res, self.t)
            self.xyzr[idx] = p[ok]
            self.stamp[idx] = time.time()

    def snapshot(self, seconds: float, fill: int, coord: str) -> tuple[np.ndarray, np.ndarray]:
        """(res, res, 4) grid in ``coord`` (r 0-1) and (res, res) valid mask."""
        res = self.res
        valid = (self.stamp > time.time() - seconds).reshape(res, res)
        xyzr = self.xyzr.reshape(res, res, 4).copy()
        xyzr[~valid] = 0.0
        fill_holes(xyzr, valid, self.t, fill)
        return convert(xyzr, coord), valid


# ---------------------------------------------------------------- colours
_TURBO_R = (0.13572138, 4.61539260, -42.66032258, 132.13108234, -152.94239396, 59.28637943)
_TURBO_G = (0.09140261, 2.19418839, 4.84296658, -14.18503333, 4.27729857, 2.82956604)
_TURBO_B = (0.10667330, 12.64194608, -60.58204836, 110.36276771, -89.90310912, 27.34824973)


def turbo(t: np.ndarray) -> np.ndarray:
    """Google Turbo colormap (polynomial fit), t in 0..1 -> (N, 3) float32."""
    t = np.clip(t, 0.0, 1.0).astype(np.float32)
    out = np.empty(t.shape + (3,), dtype=np.float32)
    for i, c in enumerate((_TURBO_R, _TURBO_G, _TURBO_B)):
        v = np.full_like(t, c[5])
        for k in range(4, -1, -1):
            v = v * t + c[k]
        out[..., i] = v
    return np.clip(out, 0.0, 1.0)


def chop_columns(pts: np.ndarray, out: np.ndarray, color: str = "depth",
                 near: float = 1.0, far: float = 10.0) -> int:
    """Write (N, 4) points as rows of CHOP_CHANNELS into ``out`` (8, >=N).
    Returns N."""
    n = len(pts)
    out[:4, :n] = pts[:, :4].T
    out[4, :n] = np.linalg.norm(pts[:, :3], axis=1)
    if color == "white":
        out[5:8, :n] = 1.0
    else:
        t = (out[4, :n] - near) / max(1e-6, far - near) if color == "depth" else out[3, :n]
        out[5:8, :n] = turbo(t).T
    return n
