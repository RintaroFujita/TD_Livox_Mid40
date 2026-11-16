# Livox Mid-40 -> TouchDesigner
# Paste this into the Callbacks DAT of a Script CHOP *or* a Script TOP
# (or point the DAT's "File" parameter at this file).
#
#   Script CHOP -> channels tx ty tz r  (one sample per point, for instancing)
#   Script TOP  -> RGBA32F point-cloud texture (RGB = XYZ metres)
#
# Script TOP: set Common > Pixel Format to "32-bit float (RGBA)".

import os
import sys

# Fallback location of this repository (used when the .toe is saved elsewhere
# and the 'Repo Folder' parameter is empty).
REPO_FALLBACK = r""


def _repo(scriptOp=None):
    cands = []
    if scriptOp is not None and hasattr(scriptOp.par, "Repo"):
        cands.append(scriptOp.par.Repo.eval())
    pf = project.folder
    cands += [pf, os.path.dirname(pf), REPO_FALLBACK]
    for c in cands:
        if c and os.path.isdir(os.path.join(c, "livox_mid40")):
            return os.path.abspath(c)
    return None


def _ops(scriptOp=None):
    repo = _repo(scriptOp)
    if repo is None:
        raise RuntimeError("livox_mid40 package not found - set 'Repo Folder' on the Livox page")
    if repo not in sys.path:
        sys.path.insert(0, repo)
    import livox_mid40.td_ops as ops
    return ops, repo


def _reload_package():
    """Close the running LiDAR session, then reload every livox_mid40 module
    (dependencies first) so edits take effect without restarting TD."""
    import importlib
    old = sys.modules.get("livox_mid40.td_bridge")
    if old is not None and hasattr(old, "shutdown"):
        try:
            old.shutdown()  # free UDP ports before the new module opens them
        except Exception as e:
            print("livox: shutdown before reload failed:", e)
    for name in ("livox_mid40.protocol", "livox_mid40.lidar", "livox_mid40.dhcp",
                 "livox_mid40.engine", "livox_mid40.shm", "livox_mid40",
                 "livox_mid40.td_bridge", "livox_mid40.td_ops"):
        mod = sys.modules.get(name)
        if mod is not None:
            importlib.reload(mod)


def onSetupParameters(scriptOp):
    _, repo = _ops(scriptOp)
    _reload_package()
    ops, _ = _ops(scriptOp)
    # leave 'Repo Folder' empty (auto-detected) so saved .toe files carry no local path
    ops.setup_pars(scriptOp, "")


def onPulse(par):
    ops, _ = _ops(par.owner)
    ops.pulse(par)


def onGetCookLevel(scriptOp):
    # Cook every frame even when nothing downstream / no viewer is pulling.
    return CookLevel.ALWAYS


def onCook(scriptOp):
    absTime.frame  # fallback time dependency for TD builds without onGetCookLevel
    ops, _ = _ops(scriptOp)
    ops.cook(scriptOp)
