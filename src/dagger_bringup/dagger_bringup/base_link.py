"""The RTK base station, as the rover sees it over MAVLink (no ROS here). Step R1.

The conversation is in DaggerGCS docs/proposals/base-station.md: the Blue Pill at
the base (system 71) sends HEARTBEAT (custom_mode = its mode), NAMED_VALUE_*
(survey progress, the position it uses) and COMMAND_ACK; the rover sends it
COMMAND_INT 31010 (survey-in, or fixed on coordinates). ArduPilot forwards it all
between its radio port and the USB link MAVROS uses.

Only the six messages this needs are encoded here (pymavlink is not on the
rover); tools/test_base_link.py checks them byte for byte against pymavlink
where it is installed, and against frames it made, where it is not.
Sites (places the base stands, with their coordinates) are kept in a JSON file.
"""
import json
import math
import os
import struct

BASE_SYSID, BASE_COMPID = 71, 220            # the Blue Pill
OUR_SYSID, OUR_COMPID = 1, 191               # as MAVROS (the onboard computer)
CMD_BASE = 31010                             # MAV_CMD_USER_1
ACTION_SURVEY, ACTION_FIXED = 1, 2
MODE_NO_FIX, MODE_SURVEYING, MODE_FIXED = 1, 2, 3
HEARD_S = 3.0                                # heard while a heartbeat came within this
RESEND_S, RESEND_TRIES = 1.0, 5

# msgid -> (struct format, CRC_EXTRA, field names), as MAVLink common.xml (MAVLink 2 order)
MESSAGES = {
    0: ("<IBBBBB", 50, ("custom_mode", "type", "autopilot", "base_mode", "system_status", "mavlink_version")),
    75: ("<ffffiifHBBBBB", 158, ("param1", "param2", "param3", "param4", "x", "y", "z", "command",
                                 "target_system", "target_component", "frame", "current", "autocontinue")),
    77: ("<HBBiBB", 143, ("command", "result", "progress", "result_param2", "target_system", "target_component")),
    251: ("<If10s", 170, ("time_boot_ms", "value", "name")),
    252: ("<Ii10s", 44, ("time_boot_ms", "value", "name")),
}


def x25(data, crc=0xFFFF):
    """MAVLink's checksum (CRC-16/MCRF4XX)."""
    for b in data:
        t = b ^ (crc & 0xFF)
        t = (t ^ (t << 4)) & 0xFF
        crc = ((crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4)) & 0xFFFF
    return crc


def encode(msgid, fields, seq=0, sysid=OUR_SYSID, compid=OUR_COMPID):
    """A MAVLink 2 frame: (bytes, payload as sent -- trailing zeros trimmed --, checksum)."""
    fmt, extra, names = MESSAGES[msgid]
    values = (fields.get(n, 0) for n in names)
    payload = struct.pack(fmt, *(v.encode("ascii") if isinstance(v, str) else v for v in values))
    while len(payload) > 1 and payload[-1] == 0:
        payload = payload[:-1]               # MAVLink 2 trims trailing zero bytes
    header = bytes([len(payload), 0, 0, seq & 0xFF, sysid, compid,
                    msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF])
    crc = x25(bytes([extra]), x25(payload, x25(header)))
    return b"\xfd" + header + payload + struct.pack("<H", crc), payload, crc


def decode(msgid, payload):
    """Fields of one of MESSAGES from its (possibly trimmed) payload, or None."""
    if msgid not in MESSAGES:
        return None
    fmt, _, names = MESSAGES[msgid]
    full = struct.calcsize(fmt)
    out = dict(zip(names, struct.unpack(fmt, bytes(payload[:full]).ljust(full, b"\0"))))
    if "name" in out:
        out["name"] = out["name"].split(b"\0", 1)[0].decode("ascii", "replace")
    return out


def to_payload64(payload):
    """mavros_msgs/Mavlink.payload64: the payload in little-endian 64-bit words."""
    padded = payload + b"\0" * (-len(payload) % 8)
    return list(struct.unpack(f"<{len(padded) // 8}Q", padded)) if padded else []


def from_payload64(words, length):
    return struct.pack(f"<{len(words)}Q", *words)[:length]


class BaseLink:
    """What the base last said, and the command being sent to it until it is acknowledged."""

    def __init__(self):
        self.heard_at = None
        self.mode = MODE_NO_FIX
        self.values = {}                     # BS_ACC, BS_DUR, BS_DONE, BS_LAT, BS_LON, BS_HGT
        self.pending = None                  # (fields, tries left, last sent)
        self.seq = 0

    def heard(self, now):
        return self.heard_at is not None and now - self.heard_at <= HEARD_S

    def on_message(self, sysid, msgid, payload, now):
        """Anything from the base; True when it changed what we know."""
        if sysid != BASE_SYSID:
            return False
        m = decode(msgid, payload)
        if m is None:
            return False
        if msgid == 0:
            self.heard_at = now
            changed = self.mode != m["custom_mode"]
            self.mode = m["custom_mode"]
            if self.mode != MODE_FIXED:
                for k in ("BS_LAT", "BS_LON", "BS_HGT"):
                    self.values.pop(k, None)
            if self.mode != MODE_SURVEYING:
                self.values.pop("BS_ACC", None)
                self.values.pop("BS_DUR", None)
            return changed
        if msgid in (251, 252):
            changed = self.values.get(m["name"]) != m["value"]
            self.values[m["name"]] = m["value"]
            return changed
        if msgid == 77 and m["command"] == CMD_BASE and self.pending is not None:
            self.pending = None              # done or refused: either way, stop resending
            return True
        return False

    def survey(self, min_s, accuracy_m):
        self._command({"param1": ACTION_SURVEY, "param2": float(min_s), "param3": float(accuracy_m)})

    def fixed(self, lat, lon, height_m):
        self._command({"param1": ACTION_FIXED, "x": int(round(lat * 1e7)), "y": int(round(lon * 1e7)),
                       "z": float(height_m)})

    def _command(self, fields):
        fields = dict(fields, command=CMD_BASE, target_system=BASE_SYSID, target_component=BASE_COMPID)
        self.pending = (fields, RESEND_TRIES, -1e9)

    def due(self, now):
        """The command frame to send now (first time, or a resend), or None."""
        if self.pending is None:
            return None
        fields, tries, last = self.pending
        if now - last < RESEND_S:
            return None
        if tries <= 0:
            self.pending = None              # never acknowledged: the status says what it does
            return None
        self.pending = (fields, tries - 1, now)
        self.seq = (self.seq + 1) & 0xFF
        return encode(75, fields, seq=self.seq)

    def position(self):
        """(lat, lon, height m) the base sends corrections from, or None."""
        v = self.values
        if self.mode != MODE_FIXED or not {"BS_LAT", "BS_LON", "BS_HGT"} <= v.keys():
            return None
        return v["BS_LAT"] / 1e7, v["BS_LON"] / 1e7, v["BS_HGT"] / 1000.0


def distance_m(a, b):
    """Metres between two (lat, lon) close together."""
    k = 111_320.0
    return math.hypot((a[0] - b[0]) * k, (a[1] - b[1]) * k * math.cos(math.radians(a[0])))


class SiteStore:
    """Places the base stands: name, latitude, longitude, height above the ellipsoid."""

    SAME_M = 0.05                            # the base is "on" a site within this

    def __init__(self, path):
        self.path = os.path.expanduser(path)
        self.sites = []
        try:
            with open(self.path, encoding="utf-8") as f:
                self.sites = [dict(s) for s in json.load(f)]
        except (OSError, ValueError):
            pass                             # no file yet, or damaged: start empty

    def _write(self):
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.sites, f, indent=1)
        os.replace(tmp, self.path)

    def get(self, name):
        return next((s for s in self.sites if s["name"] == name), None)

    def add(self, name, lat, lon, height_m):
        """False when the name is taken."""
        if self.get(name) is not None:
            return False
        self.sites.append({"name": name, "lat": lat, "lon": lon, "height_m": height_m})
        self._write()
        return True

    def delete(self, name):
        s = self.get(name)
        if s is None:
            return False
        self.sites.remove(s)
        self._write()
        return True

    def suggested_name(self):
        n = len(self.sites) + 1
        while self.get(f"Site {n}") is not None:
            n += 1
        return f"Site {n}"

    def at(self, position):
        """The site the base is on (its position within SAME_M of the site's), or None."""
        if position is None:
            return None
        return next((s for s in self.sites
                     if distance_m(position[:2], (s["lat"], s["lon"])) <= self.SAME_M
                     and abs(position[2] - s["height_m"]) <= 0.1), None)
