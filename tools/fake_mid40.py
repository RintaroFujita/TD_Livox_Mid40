"""Minimal Mid-40 emulator for testing without hardware (localhost only).

    python tools/fake_mid40.py
    python tools/livox_monitor.py --ip 127.0.0.1      # in another terminal

Answers handshake / info / heartbeat / sampling commands and streams a
rosette-like pattern of 100-point packets at ~100k pts/s.
"""

from __future__ import annotations

import math
import os
import socket
import struct
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from livox_mid40 import protocol as P  # noqa: E402

CODE = b"0TFDFAKE0000001\x00"


def main() -> None:
    cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd.bind(("127.0.0.1", P.LIDAR_CMD_PORT))
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    state = {"host": None, "sampling": False, "stop": False}

    def broadcaster():
        b = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        pkt = P.build_frame(P.SET_GENERAL, P.GEN_BROADCAST, CODE + b"\x01\x00\x00",
                            cmd_type=P.CMD_TYPE_MSG)
        while not state["stop"]:
            if state["host"] is None:
                b.sendto(pkt, ("127.0.0.1", P.BROADCAST_PORT))
            time.sleep(1.0)

    def streamer():
        t = 0.0
        while not state["stop"]:
            if not (state["sampling"] and state["host"]):
                time.sleep(0.05)
                continue
            ip, data_port, _ = state["host"]
            k = np.arange(100) + t
            a = k * 0.0123
            rr = 0.33 * np.sin(k * 0.000731)          # rosette within ~38 deg FOV
            ang_y, ang_z = rr * np.cos(a), rr * np.sin(a)
            dist = 3.0 + 0.5 * np.sin(ang_y * 6)        # wavy wall at ~3 m
            pts = np.zeros(100, dtype=P.POINT_DTYPES[0])
            pts["x"] = (dist * 1000).astype(np.int32)
            pts["y"] = (dist * np.tan(ang_y) * 1000).astype(np.int32)
            pts["z"] = (dist * np.tan(ang_z) * 1000).astype(np.int32)
            pts["r"] = (np.abs(ang_z) * 400).clip(0, 255).astype(np.uint8)
            hdr = struct.pack("<BBBBIBBQ", 5, 1, 1, 0, 0, 0, 0, time.time_ns())
            out.sendto(hdr + pts.tobytes(), (ip, data_port))
            t += 100
            time.sleep(0.001)

    threading.Thread(target=broadcaster, daemon=True).start()
    threading.Thread(target=streamer, daemon=True).start()
    print("fake Mid-40 listening on 127.0.0.1:65000 (Ctrl+C to quit)")

    try:
        while True:
            buf, addr = cmd.recvfrom(1024)
            fr = P.parse_frame(buf)
            ack = b"\x00"
            if fr.key == (P.SET_GENERAL, P.GEN_HANDSHAKE):
                ip = ".".join(str(b) for b in fr.payload[:4])
                data_port, cmd_port, imu_port = struct.unpack_from("<HHH", fr.payload, 4)
                state["host"] = (ip, data_port, cmd_port)
                print(f"handshake from {ip} data={data_port} cmd={cmd_port}")
            elif fr.key == (P.SET_GENERAL, P.GEN_QUERY_INFO):
                ack = b"\x00\x03\x08\x00\x00"
            elif fr.key == (P.SET_GENERAL, P.GEN_HEARTBEAT):
                ack = b"\x00\x01\x00" + struct.pack("<I", 0)
            elif fr.key == (P.SET_GENERAL, P.GEN_SAMPLING):
                state["sampling"] = fr.payload[:1] == b"\x01"
                print("sampling", state["sampling"])
            elif fr.key == (P.SET_GENERAL, P.GEN_DISCONNECT):
                state["host"], state["sampling"] = None, False
                print("disconnect")
            elif fr.key == (P.SET_LIDAR, P.LIDAR_READ_EXTRINSIC):
                ack = b"\x00" + struct.pack("<fffiii", 0, 0, 0, 0, 0, 0)
            reply_to = (state["host"][0], state["host"][2]) if state["host"] else addr
            cmd.sendto(P.build_frame(fr.cmd_set, fr.cmd_id, ack, fr.seq, P.CMD_TYPE_ACK), reply_to)
    except KeyboardInterrupt:
        state["stop"] = True


if __name__ == "__main__":
    main()
