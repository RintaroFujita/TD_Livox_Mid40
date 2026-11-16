"""Logic behind td/livox_callbacks.py (Script CHOP / Script TOP).

The LiDAR itself is handled by a separate server process (livox_mid40.server)
which TouchDesigner launches automatically; these operators only send
parameters and copy finished arrays out of shared memory.

Mode "grid" (default, Kinect-like):
    Points are accumulated into a Resolution x Resolution pinhole "depth
    camera" grid (38.4 deg FOV). Each pixel holds the newest point in that
    direction and expires after <Integration> seconds; gaps are filled from
    neighbours. Gives a dense, fixed-resolution organised point cloud.

Mode "raw":
    The raw points of the last <Integration> seconds (max 2 s, rosette pattern).

Script CHOP: channels tx ty tz (m), r (reflectivity 0-1), d (distance m),
    cr cg cb (colour 0-1), one sample per point -> Geometry COMP instancing.
Script TOP:  RGBA 32-bit float texture, RGB = XYZ (m), A = valid /
    reflectivity / distance. Like Kinect TOP's point cloud output.
"""

from __future__ import annotations

import math

import numpy as np

from . import engine as E
from . import shm as S
from . import td_bridge as bridge


def setup_pars(scriptOp, repo_default: str = "") -> None:
    scriptOp.destroyCustomPars()  # drop parameters from older versions
    page = scriptOp.appendCustomPage("Livox")

    def _p(group, default, nmin=None, nmax=None, vmin=None):
        p = group[0]
        p.default = default
        p.val = default
        if nmin is not None:
            p.normMin, p.normMax = nmin, nmax
        if vmin is not None:
            p.min, p.clampMin = vmin, True
        return p

    def _menu(pg, name, label, names, labels, default):
        p = pg.appendMenu(name, label=label)[0]
        p.menuNames, p.menuLabels = names, labels
        p.default = p.val = default

    _p(page.appendToggle("Active", label="Active"), True)
    _menu(page, "Mode", "Mode", ["grid", "raw"],
          ["Grid (Kinect-like depth camera)", "Raw points (rosette)"], "grid")
    # grid: up to 10 s (pixels expire individually); raw: limited to the 2 s ring buffer
    _p(page.appendFloat("Integration", label="Integration (sec)"), 0.5, 0.05, 10.0, 0.01)
    p = _p(page.appendInt("Resolution", label="Grid Resolution"), 128, 32, S.MAX_RES, 8)
    p.max, p.clampMax = S.MAX_RES, True
    _p(page.appendInt("Fill", label="Grid Hole Fill"), 1, 0, 4, 0)
    _p(page.appendFloat("Fov", label="Grid FOV (deg)"), E.MID40_FOV_DEG, 10, 60, 1)
    _menu(page, "Coord", "Coordinates", ["td", "livox"],
          ["TouchDesigner (Y up, -Z forward)", "Livox raw (X fwd, Y left, Z up)"], "td")
    _menu(page, "Color", "Color (cr cg cb)", list(S.COLORS),
          ["Distance (turbo colormap)", "Reflectivity (turbo colormap)", "White"], "depth")
    _p(page.appendFloat("Near", label="Color Near (m)"), 1.0, 0, 20, 0)
    _p(page.appendFloat("Far", label="Color Far (m)"), 10.0, 0, 50, 0)
    if scriptOp.family == "TOP":
        try:  # XYZ needs float texels, 8-bit would clamp to 0-1
            scriptOp.par.format = "rgba32float"
        except Exception:
            pass
        _menu(page, "Alpha", "Alpha", ["valid", "reflectivity", "distance"],
              ["1 = point, 0 = empty", "Reflectivity (0-1)", "Distance (m)"], "valid")
        _p(page.appendInt("Width", label="Raw Texture Width"), 256, 16, 1024, 1)

    conn = scriptOp.appendCustomPage("Connection")
    _p(conn.appendFolder("Repo", label="Repo Folder"), repo_default)
    _p(conn.appendStr("Lidarip", label="LiDAR IP (empty=auto)"), "")
    _p(conn.appendToggle("Dhcp", label="Serve DHCP"), True)
    _p(conn.appendStr("Hostip", label="PC IP (wired)"), "192.168.1.50")
    _p(conn.appendFile("Python", label="Python (empty=auto)"), "")
    conn.appendPulse("Reconnect", label="Restart Server")


REQUIRED_PARS = ("Active", "Mode", "Integration", "Resolution", "Fill", "Fov", "Coord",
                 "Color", "Near", "Far", "Repo", "Lidarip", "Dhcp", "Hostip", "Python")


def _warn(scriptOp, msg: str) -> None:
    fn = getattr(scriptOp, "addWarning", None) or getattr(scriptOp, "addScriptError", None)
    if fn:
        fn(msg)


def _empty_cols() -> np.ndarray:
    return np.zeros((S.N_CHANS, 0), np.float32)


def cook(scriptOp) -> None:
    missing = [n for n in REQUIRED_PARS if not hasattr(scriptOp.par, n)]
    if scriptOp.family == "TOP":
        missing += [n for n in ("Alpha", "Width") if not hasattr(scriptOp.par, n)]
    if missing:
        _warn(scriptOp, "Livox parameters missing (" + ", ".join(missing)
              + "): press 'Setup Parameters' and check the callbacks DAT for errors")
        if scriptOp.family == "CHOP":
            _cook_chop(scriptOp, _empty_cols())
        return

    f = None
    par = scriptOp.par
    if par.Active.eval():
        note = bridge.ensure(par.Lidarip.eval(), par.Dhcp.eval(), par.Hostip.eval(),
                             par.Python.eval(), owner=scriptOp.path)
        if bridge.is_owner(scriptOp.path):  # one operator drives the processing settings
            bridge.send(mode=par.Mode.eval(), res=int(par.Resolution.eval()),
                        fov=float(par.Fov.eval()), fill=int(par.Fill.eval()),
                        coord=par.Coord.eval(), seconds=float(par.Integration.eval()),
                        color=par.Color.eval(), near=float(par.Near.eval()),
                        far=float(par.Far.eval()))
        st = bridge.status()
        scriptOp.store("livox_status", st)
        if st.get("state") != "streaming":
            _warn(scriptOp, f"Livox {st.get('state')}: {st.get('error', '')} (log: {st.get('log')})")
        elif note:
            _warn(scriptOp, note)
        f = bridge.frame()
    elif bridge.is_owner(scriptOp.path):
        bridge.shutdown()  # only the owner may stop the server; others just go quiet

    if scriptOp.family == "CHOP":
        _cook_chop(scriptOp, f.cols if f is not None else _empty_cols())
    else:
        _cook_top(scriptOp, f)


def _cook_chop(scriptOp, cols: np.ndarray) -> None:
    """cols: (8, N) float32 rows for CHOP_CHANNELS (rows contiguous)."""
    scriptOp.clear()
    # A CHOP cannot be 0 samples long; TD clamps the length, so always match
    # whatever length TD actually applied (empty -> one dummy point at origin).
    scriptOp.numSamples = max(1, cols.shape[1])
    n = scriptOp.numSamples
    if cols.shape[1] != n:
        buf = np.zeros((S.N_CHANS, n), dtype=np.float32)
        k = min(n, cols.shape[1])
        buf[:, :k] = cols[:, :k]
        cols = buf
    for i, name in enumerate(E.CHOP_CHANNELS):
        scriptOp.appendChan(name).copyNumpyArray(cols[i])


def _cook_top(scriptOp, f) -> None:
    par = scriptOp.par
    alpha = par.Alpha.eval()
    if f is not None and f.mode == "grid" and f.res > 0:
        xyzr, valid = f.grid, f.valid.astype(bool)
    elif f is not None and f.mode == "raw":
        w = max(1, int(par.Width.eval()))
        n = f.n
        h = max(1, math.ceil(max(n, 1) / w))
        xyzr = np.zeros((h * w, 4), np.float32)
        xyzr[:n] = f.cols[:4].T
        valid = np.zeros(h * w, bool)
        valid[:n] = True
        xyzr, valid = xyzr.reshape(h, w, 4), valid.reshape(h, w)
    else:
        r = int(par.Resolution.eval())
        xyzr, valid = np.zeros((r, r, 4), np.float32), np.zeros((r, r), bool)
    img = np.empty(xyzr.shape, dtype=np.float32)
    img[..., :3] = xyzr[..., :3]
    if alpha == "reflectivity":
        a = xyzr[..., 3]
    elif alpha == "distance":
        a = np.linalg.norm(xyzr[..., :3], axis=-1)
    else:
        a = 1.0
    img[..., 3] = np.where(valid, a, 0.0)
    scriptOp.copyNumpyArray(img)


def pulse(par) -> None:
    if par.name == "Reconnect":
        bridge.shutdown()
