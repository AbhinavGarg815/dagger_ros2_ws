#!/usr/bin/env python3
"""Offline test of tools/gps_time.py (the GPS time from the Cube at boot). NO CUBE.

    python3 tools/test_gps_time.py        (or: python3 -m pytest tools/test_gps_time.py)

The frames must be what MAVLink says, byte for byte: checked against pymavlink when it
is installed (the PC), and against frames pymavlink made (GOLDEN), which needs nothing.
"""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gps_time import Reader, request_frame, unix_seconds  # noqa: E402

# pymavlink 2.4 (common), system 255 / component 190, seq 3:
# COMMAND_LONG(target 1/1, MAV_CMD_REQUEST_MESSAGE 512, param1 = 2 SYSTEM_TIME)
GOLDEN_REQUEST = "fd20000003ffbe4c000000000040000000000000000000000000000000000000000000000000000201012bae"
# SYSTEM_TIME(time_unix_usec 1792560000123456, time_boot_ms 5000) from system 1/1, seq 9: MAVLink 2 and 1
GOLDEN_TIME = "fd0a000009010102000040c20be6525e06008813f56b"
GOLDEN_TIME_V1 = "fe0c0901010240c20be6525e060088130000f6ec"


def pymavlink():
    try:
        from pymavlink.dialects.v20 import common as m
        return m
    except ImportError:
        return None


def test_the_request_matches_pymavlink():
    m = pymavlink()
    assert request_frame(3).hex() == GOLDEN_REQUEST
    if m is None:
        return
    mav = m.MAVLink(None, srcSystem=255, srcComponent=190)
    mav.seq = 3
    want = mav.command_long_encode(1, 1, 512, 0, 2, 0, 0, 0, 0, 0, 0).pack(mav)
    assert request_frame(3) == want, (request_frame(3).hex(), want.hex())


def test_system_time_is_read_out_of_a_stream_in_pieces():
    frame = bytes.fromhex(GOLDEN_TIME)
    stream = b"\x00\x01noise\xfe\x03" + frame + bytes.fromhex(GOLDEN_REQUEST)   # rubbish, a broken start, other traffic
    r, got = Reader(), []
    for i in range(0, len(stream), 7):                           # arrives in small pieces
        got += r.feed(stream[i:i + 7])
    times = [unix_seconds(p) for s, msgid, p in got if msgid == 2]
    assert times and abs(times[0] - 1792560000.123456) < 1e-6
    v1 = [unix_seconds(p) for _, msgid, p in Reader().feed(bytes.fromhex(GOLDEN_TIME_V1)) if msgid == 2]
    assert v1 and abs(v1[0] - 1792560000.123456) < 1e-6, "MAVLink 1 too"


def test_no_gps_time_yet_is_none_and_a_damaged_frame_is_skipped():
    assert unix_seconds(struct.pack("<QI", 0, 5000)) is None, "0 until the GPS has time"
    assert unix_seconds(struct.pack("<QI", 3_600_000_000, 5000)) is None, "1970: not GPS time"
    assert unix_seconds(b"") is None, "all zeros trimmed by MAVLink 2"
    frame = bytearray.fromhex(GOLDEN_TIME)
    frame[12] ^= 0xFF
    assert not [x for x in Reader().feed(bytes(frame)) if x[1] == 2]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok ", name)
