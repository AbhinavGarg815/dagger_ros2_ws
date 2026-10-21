#!/usr/bin/env python3
"""The time from GPS, read from the Cube at boot, BEFORE the rover stack opens its port.

    sudo python3 tools/gps_time.py --set          # set the Jetson's clock from GPS (dagger-wait-clock.sh)
    python3 tools/gps_time.py                     # only print it (the stack must be stopped: one reader per port)

Why: this Jetson's clock has no backup battery, so at power-on it starts at the last
shutdown time; in the field there is no time server. GPS time is right anywhere under
the sky. The Cube passes it on in MAVLink SYSTEM_TIME (time_unix_usec: 0 until the GPS
has time). We ask for it with MAV_CMD_REQUEST_MESSAGE -- a request to send a message,
which changes nothing on the Cube -- once a second, for up to --wait seconds.

Stdlib only (no pymavlink or pyserial on the rover): the two messages are packed by
hand; tools/test_gps_time.py checks them against pymavlink where it is installed.
"""
import argparse
import os
import struct
import subprocess
import sys
import time

PORT = "/dev/serial/by-id/usb-CubePilot_CubeOrange+_44002F000E51333233343437-if00"
OUR_SYSID, OUR_COMPID = 255, 190          # a ground station's ids; MAVROS is not running yet
SYSTEM_TIME, COMMAND_LONG = 2, 76
CRC_EXTRA = {SYSTEM_TIME: 137, COMMAND_LONG: 152}
MAV_CMD_REQUEST_MESSAGE = 512
SANE_AFTER = 1735689600                    # 2025-01-01: anything earlier is not GPS time


def x25(data, crc=0xFFFF):
    """MAVLink's checksum (CRC-16/MCRF4XX)."""
    for b in data:
        t = b ^ (crc & 0xFF)
        t = (t ^ (t << 4)) & 0xFF
        crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return crc


def request_frame(seq, msgid=SYSTEM_TIME):
    """MAVLink 2 COMMAND_LONG: MAV_CMD_REQUEST_MESSAGE(msgid) to system 1, component 1."""
    payload = struct.pack("<7fHBBB", float(msgid), 0, 0, 0, 0, 0, 0, MAV_CMD_REQUEST_MESSAGE, 1, 1, 0)
    while len(payload) > 1 and payload[-1] == 0:
        payload = payload[:-1]               # MAVLink 2 trims trailing zero bytes
    header = bytes([len(payload), 0, 0, seq & 0xFF, OUR_SYSID, OUR_COMPID,
                    COMMAND_LONG & 0xFF, (COMMAND_LONG >> 8) & 0xFF, COMMAND_LONG >> 16])
    crc = x25(bytes([CRC_EXTRA[COMMAND_LONG]]), x25(payload, x25(header)))
    return b"\xfd" + header + payload + struct.pack("<H", crc)


class Reader:
    """MAVLink 1 and 2 frames out of a byte stream; yields (sysid, msgid, payload) with a good checksum."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            starts = [i for i in (self.buf.find(b"\xfd"), self.buf.find(b"\xfe")) if i >= 0]
            if not starts:
                self.buf.clear()
                return out
            del self.buf[:min(starts)]
            v2 = self.buf[0] == 0xFD
            hl = 10 if v2 else 6
            if len(self.buf) < hl:
                return out
            n = self.buf[1]
            total = hl + n + 2 + (13 if v2 and self.buf[2] & 1 else 0)
            if len(self.buf) < total:
                return out
            msgid = (self.buf[7] | self.buf[8] << 8 | self.buf[9] << 16) if v2 else self.buf[5]
            sysid = self.buf[5] if v2 else self.buf[3]
            payload = bytes(self.buf[hl:hl + n])
            crc = struct.unpack_from("<H", self.buf, hl + n)[0]
            if msgid in CRC_EXTRA and x25(bytes([CRC_EXTRA[msgid]]), x25(payload, x25(bytes(self.buf[1:hl])))) == crc:
                out.append((sysid, msgid, payload))
                del self.buf[:total]
            else:
                del self.buf[:1]             # not a frame we know, or damaged: look further on


def unix_seconds(payload):
    """SYSTEM_TIME's time_unix_usec in seconds, or None when the Cube has no GPS time yet."""
    usec, _boot_ms = struct.unpack("<QI", payload.ljust(12, b"\0")[:12])
    s = usec / 1e6
    return s if s > SANE_AFTER else None


def gps_time(port, wait_s):
    """Seconds since 1970 from GPS, through the Cube, or None after wait_s."""
    import termios                           # Linux only: here, so the codec tests run anywhere
    import tty
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        tty.setraw(fd)
        attrs = termios.tcgetattr(fd)
        attrs[4] = attrs[5] = termios.B115200   # USB: the speed does not matter, raw does
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        reader, seq, end, asked = Reader(), 0, time.monotonic() + wait_s, 0.0
        while time.monotonic() < end:
            if time.monotonic() - asked >= 1.0:
                os.write(fd, request_frame(seq))
                seq, asked = seq + 1, time.monotonic()
            try:
                data = os.read(fd, 4096)
            except BlockingIOError:
                time.sleep(0.05)
                continue
            for sysid, msgid, payload in reader.feed(data):
                if sysid == 1 and msgid == SYSTEM_TIME:
                    s = unix_seconds(payload)
                    if s is not None:
                        return s
        return None
    finally:
        os.close(fd)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--wait", type=float, default=90.0, help="seconds to wait for GPS time")
    ap.add_argument("--set", action="store_true", help="set the system clock (needs root)")
    a = ap.parse_args()
    if not os.path.exists(a.port):
        print(f"no Cube at {a.port}")
        return 1
    s = gps_time(a.port, a.wait)
    if s is None:
        print(f"no GPS time from the Cube in {a.wait:.0f} s (no GPS fix yet?)")
        return 1
    off = s - time.time()
    print(f"GPS time {time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(s))} UTC; the clock is {off:+.1f} s off")
    if a.set and abs(off) > 2.0:
        subprocess.run(["date", "-u", "-s", f"@{s:.3f}"], check=True, stdout=subprocess.DEVNULL)
        print(f"clock set from GPS: {time.strftime('%Y-%m-%d %H:%M:%S %Z')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
