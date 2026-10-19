#!/usr/bin/env python3
"""Bench test of the RTK base station's Blue Pill, without the rover. NOTHING ON THE ROVER.

    python3 tools/base_bench.py /dev/ttyUSB0                 # watch: mode, survey, position, RTCM
    python3 tools/base_bench.py /dev/ttyUSB0 survey 300 2.0  # survey-in: at least 300 s, until 2.0 m
    python3 tools/base_bench.py /dev/ttyUSB0 fixed 26.7478123 75.8554012 418.237   # lat lon height(ellipsoid)

Connect a USB-serial adapter (3.3 V) in place of the RFD868 (Blue Pill PA2 -> adapter RX,
PA3 <- adapter TX, GND), 57600 baud -- or the base-side radio's twin on the PC. It sends
the commands as the rover's base_station node does (dagger_bringup/base_link.py) and
prints what the base says, once a second, plus how many RTCM bytes arrive.
Needs pyserial (apt: python3-serial; pip: pyserial).
"""
import os
import struct
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "dagger_bringup"))
from dagger_bringup.base_link import (BASE_SYSID, MODE_FIXED, MODE_SURVEYING, BaseLink, decode,  # noqa: E402
                                      x25)

CRC_EXTRA = {0: 50, 75: 158, 77: 143, 233: 35, 251: 170, 252: 44}
MODES = {1: "no fix", MODE_SURVEYING: "surveying", MODE_FIXED: "fixed"}


def frames(port):
    """MAVLink 2 frames from the serial stream: (sysid, msgid, payload)."""
    buf = bytearray()
    while True:
        buf += port.read(port.in_waiting or 1)
        while True:
            i = buf.find(b"\xfd")
            if i < 0:
                buf.clear()
                break
            del buf[:i]
            if len(buf) < 10:
                break
            n, signed = buf[1], buf[2] & 1
            total = 10 + n + 2 + (13 if signed else 0)
            if len(buf) < total:
                break
            msgid = buf[7] | buf[8] << 8 | buf[9] << 16
            payload = bytes(buf[10:10 + n])
            crc = struct.unpack_from("<H", buf, 10 + n)[0]
            ok = msgid in CRC_EXTRA and x25(bytes([CRC_EXTRA[msgid]]), x25(payload, x25(bytes(buf[1:10])))) == crc
            if ok:
                yield buf[5], msgid, payload
                del buf[:total]
            else:
                del buf[:1]                   # not a frame after all: look further on


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    port = serial.Serial(sys.argv[1], 57600, timeout=0.1)
    link = BaseLink()
    args = sys.argv[2:]
    if args[:1] == ["survey"]:
        link.survey(float(args[1]) if len(args) > 1 else 300.0, float(args[2]) if len(args) > 2 else 2.0)
    elif args[:1] == ["fixed"]:
        link.fixed(float(args[1]), float(args[2]), float(args[3]))
    rtcm_bytes, last, acked = 0, time.monotonic(), False
    for sysid, msgid, payload in frames(port):
        now = time.monotonic()
        if sysid == BASE_SYSID:
            if msgid == 233:
                rtcm_bytes += payload[1] if len(payload) > 1 else 0
            elif msgid == 77 and link.pending is not None:
                acked = decode(77, payload)["result"] == 0
                print("command", "ACCEPTED" if acked else "REFUSED")
            link.on_message(sysid, msgid, payload, now)
        out = link.due(now)
        if out is not None:
            port.write(out[0])
        if now - last >= 1.0:
            last = now
            v, pos = link.values, link.position()
            line = f"{'heard' if link.heard(now) else 'NOT heard'}  mode {MODES.get(link.mode, link.mode)}"
            sats = v.get("BS_SATS")
            line += "  F9P not heard" if sats == -1 else (f"  satellites {sats}" if sats is not None else "")
            if link.mode == MODE_SURVEYING:
                line += f"  survey {v.get('BS_ACC', 0):.2f} m after {v.get('BS_DUR', 0)} s"
            if pos:
                line += f"  at {pos[0]:.7f}, {pos[1]:.7f}, {pos[2]:.3f} m" + ("  (survey done)" if v.get("BS_DONE") else "")
            print(line + f"  RTCM {rtcm_bytes} B/s")
            rtcm_bytes = 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
