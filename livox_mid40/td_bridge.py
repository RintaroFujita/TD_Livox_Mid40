"""TouchDesigner side: launches the LiDAR server process and reads its output
from shared memory.

All LiDAR I/O and point processing happen in ``livox_mid40.server`` (a normal
Python process), so TouchDesigner's Python only copies finished arrays and
never competes for the GIL with 1000 packets/s of network traffic.

State lives in this regular module (not in a DAT), so editing the callbacks
DAT does not restart the server.
"""

from __future__ import annotations

import glob
import os
import shutil
import subprocess
import sys
import tempfile
import time
from multiprocessing import shared_memory

from . import shm as S

LOG_PATH = os.path.join(tempfile.gettempdir(), "livox_mid40_server.log")
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SHM_PREFIX = S.DEFAULT_PREFIX  # tests override this so they never touch TD's server
EXTRA_SERVER_ARGS: list[str] = []

_cfg: tuple | None = None
_owner = ""
_owner_seen = 0.0
OWNER_TIMEOUT = 2.0  # seconds without a call from the owner -> another operator takes over
_proc: subprocess.Popen | None = None
_python = ""
_launch_time = 0.0
_launch_error = ""
_ctl: shared_memory.SharedMemory | None = None
_data: shared_memory.SharedMemory | None = None
_reader: S.DataReader | None = None


# ---------------------------------------------------------------- process
def find_python(hint: str = "") -> str:
    """A regular CPython with numpy (not TouchDesigner's embedded one)."""
    cands = [hint] if hint else []
    cands += [shutil.which("python"), shutil.which("python3")]
    cands += sorted(glob.glob(os.path.expandvars(
        r"%LOCALAPPDATA%\Programs\Python\Python3*\python.exe")), reverse=True)
    cands += sorted(glob.glob(r"C:\Python3*\python.exe"), reverse=True)
    for c in cands:
        if c and os.path.isfile(c) and "WindowsApps" not in c and "Derivative" not in c:
            return c
    return ""


def _launch(lidar_ip: str, use_dhcp: bool, host_ip: str, python_hint: str) -> None:
    global _proc, _python, _launch_time, _launch_error
    _launch_time = time.time()
    _python = find_python(python_hint)
    if not _python:
        _launch_error = "python.exe not found: set 'Python' on the Connection page"
        return
    cmd = [_python, "-m", "livox_mid40.server", "--host-ip", host_ip,
           "--shm-prefix", SHM_PREFIX, *EXTRA_SERVER_ARGS]
    if lidar_ip:
        cmd += ["--lidar-ip", lidar_ip]
    if use_dhcp:
        cmd.append("--dhcp")
    flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    try:
        log = open(LOG_PATH, "a", encoding="utf-8")
        log.write(f"\n==== {time.ctime()} launch: {' '.join(cmd)}\n")
        log.flush()
        _proc = subprocess.Popen(cmd, cwd=REPO, stdout=log, stderr=subprocess.STDOUT,
                                 stdin=subprocess.DEVNULL, creationflags=flags)
        _launch_error = ""
    except Exception as e:
        _proc, _launch_error = None, f"launch failed: {e}"


def _attach() -> bool:
    global _ctl, _data, _reader
    if _reader is not None:
        return True
    try:
        ctl = shared_memory.SharedMemory(name=S.ctl_name(SHM_PREFIX))
        data = shared_memory.SharedMemory(name=S.data_name(SHM_PREFIX))
    except FileNotFoundError:
        return False
    _ctl, _data, _reader = ctl, data, S.DataReader(data)
    return True


def _detach() -> None:
    global _ctl, _data, _reader
    _reader = None
    for m in (_ctl, _data):
        if m is not None:
            try:
                m.close()
            except BufferError:  # a numpy view is still alive; GC will release it
                pass
    _ctl = _data = None


def _server_alive(h: dict | None) -> bool:
    return h is not None and h["state"] != "stopped" and time.time() - h["server_beat"] < 3.0


# ---------------------------------------------------------------- public API
def ensure(lidar_ip: str = "", use_dhcp: bool = True, host_ip: str = "192.168.1.50",
           python: str = "", owner: str = "") -> str:
    """Make sure a server is running and attached. Cheap; call every frame.

    The first operator to call this (``owner``) decides the connection
    settings; returns a warning for other operators asking for different ones.
    """
    global _cfg, _owner, _owner_seen
    cfg = (lidar_ip.strip(), bool(use_dhcp), host_ip.strip(), python.strip())
    now = time.time()
    if owner == _owner:
        _owner_seen = now
    elif _owner and now - _owner_seen > OWNER_TIMEOUT:
        # the owner was deleted / bypassed / deactivated: hand ownership over
        _owner, _owner_seen = owner, now
        if _cfg is not None and cfg != _cfg:
            shutdown()
    if _cfg is not None and cfg != _cfg:
        if owner != _owner:
            return f"connection settings ignored: owned by {_owner}"
        shutdown()  # owner changed the connection settings -> restart server
    if _cfg is None:
        _cfg, _owner, _owner_seen = cfg, owner, now

    h = _reader.header() if _attach() else None
    if not _server_alive(h):
        proc_running = _proc is not None and _proc.poll() is None
        if not proc_running and time.time() - _launch_time > 5.0:
            _launch(*cfg)
    return ""


def is_owner(owner: str) -> bool:
    return owner == _owner


def send(mode: str = "grid", res: int = 128, fov: float = 38.4, fill: int = 1,
         coord: str = "td", seconds: float = 0.5, color: str = "depth",
         near: float = 1.0, far: float = 10.0) -> None:
    """Write processing parameters + heartbeat to the server."""
    if _ctl is None:
        return
    _ctl.buf[: S._CTL.size] = S.ctl_pack(heartbeat=time.time(), mode=mode, res=res, fov=fov,
                                         fill=fill, coord=coord, seconds=seconds, color=color,
                                         near=near, far=far)


def frame() -> S.Frame | None:
    """Newest frame (zero-copy views into shared memory) or None."""
    if _reader is None:
        return None
    h = _reader.header()
    if not _server_alive(h):
        return None
    return _reader.frame()


def frame_still_valid(f: S.Frame) -> bool:
    """True if the slot was not overwritten while it was being copied."""
    return _reader is not None and _reader.slot_seq(f.slot) == f.seq


def shutdown() -> None:
    """Ask the server to quit and forget the configuration."""
    global _cfg, _owner, _proc
    if _ctl is not None:
        try:
            _ctl.buf[: S._CTL.size] = S.ctl_pack(heartbeat=time.time(), mode="grid", res=128,
                                                 fov=38.4, fill=1, coord="td", seconds=0.5,
                                                 color="depth", near=1.0, far=10.0, quit=True)
        except Exception:
            pass
    if _proc is not None:
        try:
            _proc.wait(timeout=3.0)
        except Exception:
            _proc.kill()
    _proc = None
    _detach()
    _cfg, _owner = None, ""


def status() -> dict:
    h = _reader.header() if _reader is not None else None
    out = dict(h) if h else {"state": "no server"}
    if h and not _server_alive(h):
        out["state"] = "server not responding"
    out.update(owner=_owner, python=_python, log=LOG_PATH)
    if _launch_error:
        out["error"] = _launch_error
    if _proc is not None and _proc.poll() is not None:
        out["server_exit_code"] = _proc.returncode
    return out
