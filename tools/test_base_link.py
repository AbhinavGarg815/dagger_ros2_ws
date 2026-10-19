#!/usr/bin/env python3
"""Offline test of dagger_bringup.base_link (the RTK base over MAVLink). NO ROS, NO ROVER.

    python3 tools/test_base_link.py        (or: python3 -m pytest tools/test_base_link.py)

The frames must be what MAVLink 2 says, byte for byte: checked against pymavlink
when it is installed (on the PC), and against frames pymavlink made (GOLDEN),
which needs nothing. Then the base's status as heard, commands resent until
acknowledged, and the site store.
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "dagger_bringup"))
from dagger_bringup.base_link import (BASE_SYSID, MODE_FIXED, MODE_SURVEYING, RESEND_TRIES, BaseLink,  # noqa: E402
                                      SiteStore, decode, encode, from_payload64, to_payload64)

# Made by pymavlink 2.4 (common dialect), sysid 1 / compid 191, seq 7: fixed on
# 26.7478123 N, 75.8554012 E, 418.237 m -- the one command that carries coordinates.
GOLDEN_FIXED = ("fd2000000701bf4b0000000000400000000000000000000000006b64f10f"
                "9c9d362d561ed143227947dc7f88")
GOLDEN_FIXED_FIELDS = {"param1": 2.0, "x": 267478123, "y": 758554012, "z": 418.237, "command": 31010,
                       "target_system": 71, "target_component": 220}


def frame(msgid, fields, sysid=BASE_SYSID, compid=220):
    return encode(msgid, fields, sysid=sysid, compid=compid)


def test_matches_the_golden_frame():
    data, _, _ = encode(75, GOLDEN_FIXED_FIELDS, seq=7)
    assert data.hex() == GOLDEN_FIXED, data.hex()


def test_matches_pymavlink_when_installed():
    try:
        from pymavlink.dialects.v20 import common as m
    except ImportError:
        return                                # the rover: GOLDEN covers it
    mav = m.MAVLink(None, srcSystem=1, srcComponent=191)
    mav.seq = 7
    want = mav.command_int_encode(71, 220, 0, 31010, 0, 0, 2.0, 0, 0, 0, 267478123, 758554012, 418.237).pack(mav)
    got, _, _ = encode(75, GOLDEN_FIXED_FIELDS, seq=7)
    assert got == want, (got.hex(), want.hex())
    for msgid, msg in ((0, mav.heartbeat_encode(0, 8, 0, 3, 4, 3)),
                       (77, mav.command_ack_encode(31010, 0, 0, 0, 1, 191)),
                       (252, mav.named_value_int_encode(1234, b"BS_HGT", 418237)),
                       (251, mav.named_value_float_encode(1234, b"BS_ACC", 2.5))):
        mav.seq = 7
        want = msg.pack(mav)
        fields = {n: getattr(msg, n) for n in msg.get_fieldnames()}
        got, _, _ = encode(msgid, fields, seq=7)
        assert got == want, (msgid, got.hex(), want.hex())


def test_payload64_round_trip():
    _, payload, _ = encode(252, {"time_boot_ms": 5, "value": -7, "name": b"BS_LAT"})
    words = to_payload64(payload)
    assert from_payload64(words, len(payload)) == payload
    assert decode(252, payload)["name"] == "BS_LAT" and decode(252, payload)["value"] == -7


def hear(link, now, mode, **values):
    _, p, _ = frame(0, {"custom_mode": mode, "type": 0, "autopilot": 8, "system_status": 4, "mavlink_version": 3})
    link.on_message(BASE_SYSID, 0, p, now)
    for k, v in values.items():
        msgid = 251 if isinstance(v, float) else 252
        _, p, _ = frame(msgid, {"time_boot_ms": 0, "value": v, "name": k.encode()})
        link.on_message(BASE_SYSID, msgid, p, now)


def test_status_as_heard():
    link = BaseLink()
    assert not link.heard(0.0)
    hear(link, 10.0, MODE_SURVEYING, BS_ACC=2.4, BS_DUR=95)
    assert link.heard(12.0) and not link.heard(13.5), "heard while heartbeats keep coming"
    assert link.mode == MODE_SURVEYING and abs(link.values["BS_ACC"] - 2.4) < 1e-6 and link.position() is None
    hear(link, 20.0, MODE_FIXED, BS_DONE=1, BS_LAT=267478123, BS_LON=758554012, BS_HGT=418237)
    lat, lon, h = link.position()
    assert abs(lat - 26.7478123) < 1e-9 and abs(lon - 75.8554012) < 1e-9 and abs(h - 418.237) < 1e-9
    assert "BS_ACC" not in link.values, "survey progress gone once fixed"
    # Something from another system is not the base.
    _, p, _ = frame(0, {"custom_mode": MODE_SURVEYING}, sysid=1)
    link.on_message(1, 0, p, 21.0)
    assert link.mode == MODE_FIXED


def test_a_command_is_resent_until_acknowledged():
    link = BaseLink()
    link.fixed(26.7478123, 75.8554012, 418.237)
    sent = [link.due(t) for t in (0.0, 0.5, 1.1, 2.2)]
    assert sent[0] is not None and sent[1] is None and sent[2] is not None and sent[3] is not None
    f = decode(75, sent[0][1])
    assert f["param1"] == 2.0 and f["x"] == 267478123 and f["target_system"] == 71 and f["command"] == 31010
    _, p, _ = frame(77, {"command": 31010, "result": 0, "target_system": 1, "target_component": 191})
    link.on_message(BASE_SYSID, 77, p, 2.5)
    assert link.due(5.0) is None, "acknowledged: no more resends"
    link.survey(300, 2.0)
    n = sum(1 for t in range(20) if link.due(float(t)) is not None)
    assert n == RESEND_TRIES, "never acknowledged: gives up after a few tries"


def test_sites_are_kept_and_matched():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "sites.json")
        s = SiteStore(path)
        assert s.suggested_name() == "Site 1"
        assert s.add("Yard base", 26.7478123, 75.8554012, 418.237) and not s.add("Yard base", 0, 0, 0)
        again = SiteStore(path)
        assert again.get("Yard base")["height_m"] == 418.237 and again.suggested_name() == "Site 2"
        assert again.at((26.7478124, 75.8554012, 418.24))["name"] == "Yard base", "1 cm away: on it"
        assert again.at((26.7478223, 75.8554012, 418.237)) is None, "1 m away: not on it"
        assert again.delete("Yard base") and SiteStore(path).sites == []


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok   ", name)
    print("all passed")


def test_each_request_is_numbered_and_its_resends_keep_the_number():
    link = BaseLink()
    link.survey(300, 2.0)
    first, resend = decode(75, link.due(0.0)[1]), decode(75, link.due(1.5)[1])
    assert first["param4"] == resend["param4"] >= 1, "a resend carries the same number"
    link.survey(300, 2.0)
    again = decode(75, link.due(3.0)[1])
    assert again["param4"] != first["param4"], "Survey again: same settings, a new number"
    assert again["param1"] == 1.0 and again["param2"] == 300.0
