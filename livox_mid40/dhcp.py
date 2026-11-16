"""Tiny single-lease DHCP server for a LiDAR directly cabled to this PC.

Mid-40 ships in *dynamic IP* mode and keeps sending DHCPDISCOVER when there is
no router. This hands it one fixed address so it starts broadcasting.

    python -m livox_mid40.dhcp --server-ip 192.168.1.50 --lease-ip 192.168.1.100
"""

from __future__ import annotations

import argparse
import logging
import socket
import struct
import threading

log = logging.getLogger("livox_mid40.dhcp")

MAGIC = b"\x63\x82\x53\x63"
DISCOVER, OFFER, REQUEST, DECLINE, ACK, NAK, RELEASE, INFORM = range(1, 9)
_NAMES = {1: "DISCOVER", 2: "OFFER", 3: "REQUEST", 4: "DECLINE", 5: "ACK", 6: "NAK",
          7: "RELEASE", 8: "INFORM"}


def _ip(s: str) -> bytes:
    return socket.inet_aton(s)


def _parse_options(buf: bytes) -> dict[int, bytes]:
    opts, i = {}, 240
    while i < len(buf):
        code = buf[i]
        if code == 0:
            i += 1
            continue
        if code == 255 or i + 1 >= len(buf):
            break
        n = buf[i + 1]
        opts[code] = buf[i + 2 : i + 2 + n]
        i += 2 + n
    return opts


class MiniDHCPServer:
    def __init__(self, server_ip: str = "192.168.1.50", lease_ip: str = "192.168.1.100",
                 netmask: str = "255.255.255.0", lease_time: int = 86400,
                 mac_prefix: str | None = None):
        self.server_ip = server_ip
        self.lease_ip = lease_ip
        self.netmask = netmask
        self.lease_time = lease_time
        self.mac_prefix = mac_prefix.lower() if mac_prefix else None  # e.g. "34:d2:62" (DJI/Livox)
        self.leased = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sock: socket.socket | None = None

    # ---------------------------------------------------------------- control
    def start(self) -> "MiniDHCPServer":
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        # Bind to the wired adapter's address so replies leave through it.
        s.bind((self.server_ip, 67))
        s.settimeout(0.5)
        self._sock = s
        self._thread = threading.Thread(target=self._loop, name="livox-dhcp", daemon=True)
        self._thread.start()
        log.info("DHCP server on %s:67, lease %s", self.server_ip, self.lease_ip)
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._sock:
            self._sock.close()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    # ---------------------------------------------------------------- internals
    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                buf, _addr = self._sock.recvfrom(1500)
            except (socket.timeout, OSError):
                continue
            if len(buf) < 240 or buf[0] != 1 or buf[236:240] != MAGIC:
                continue
            mac = buf[28:34].hex(":")
            if self.mac_prefix and not mac.startswith(self.mac_prefix):
                continue
            opts = _parse_options(buf)
            mtype = opts.get(53, b"\x00")[0]
            log.debug("%s from %s", _NAMES.get(mtype, mtype), mac)
            if mtype == DISCOVER:
                self._reply(buf, OFFER)
                log.info("OFFER %s -> %s", self.lease_ip, mac)
            elif mtype == REQUEST:
                sid = opts.get(54)
                if sid is not None and sid != _ip(self.server_ip):
                    continue  # client chose another server
                want = opts.get(50) or buf[12:16]
                if want in (_ip(self.lease_ip), b"\x00\x00\x00\x00"):
                    self._reply(buf, ACK)
                    log.info("ACK  %s -> %s", self.lease_ip, mac)
                    self.leased.set()
                else:
                    self._reply(buf, NAK)
                    log.info("NAK  %s requested %s", mac, socket.inet_ntoa(want))

    def _reply(self, req: bytes, mtype: int) -> None:
        xid, flags = req[4:8], req[10:12]
        yi = b"\x00" * 4 if mtype == NAK else _ip(self.lease_ip)
        pkt = struct.pack("!BBBB", 2, 1, 6, 0) + xid + b"\x00\x00" + flags
        pkt += b"\x00" * 4 + yi + _ip(self.server_ip) + b"\x00" * 4  # ciaddr yiaddr siaddr giaddr
        pkt += req[28:44] + b"\x00" * 192 + MAGIC                    # chaddr, sname, file
        opt = bytes([53, 1, mtype]) + bytes([54, 4]) + _ip(self.server_ip)
        if mtype != NAK:
            opt += bytes([51, 4]) + struct.pack("!I", self.lease_time)
            opt += bytes([58, 4]) + struct.pack("!I", self.lease_time // 2)
            opt += bytes([59, 4]) + struct.pack("!I", self.lease_time * 7 // 8)
            opt += bytes([1, 4]) + _ip(self.netmask)
        pkt += opt + b"\xff"
        pkt += b"\x00" * max(0, 300 - len(pkt))
        self._sock.sendto(pkt, ("255.255.255.255", 68))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--server-ip", default="192.168.1.50")
    ap.add_argument("--lease-ip", default="192.168.1.100")
    ap.add_argument("--mac-prefix", default="34:d2:62", help="only answer these MACs ('' = any)")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
    srv = MiniDHCPServer(a.server_ip, a.lease_ip, mac_prefix=a.mac_prefix or None).start()
    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        srv.stop()


if __name__ == "__main__":
    main()
