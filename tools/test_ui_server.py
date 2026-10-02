#!/usr/bin/env python3
"""End-to-end test of the click-to-follow UI path, NO camera, NO rover.

    python3 tools/test_ui_server.py

Runs in ROS_DOMAIN_ID 77 (set before rclpy import, inherited by the nodes it
starts) with the real person_tracker and ui_server (port 18080, allow_follow on),
a fake camera (real JPEGs + detections of two people, same stamps, 10 Hz), and a
WebSocket client standing in for the browser. Checks: page served; frames arrive
paired with their tracks; server-side frame age; select reaches the tracker;
FOLLOW enables; the DEADMAN disables on missing heartbeats and on disconnect; a
slow client gets fresh frames, not a backlog.
"""
import array
import asyncio
import json
import os
import signal
import struct
import subprocess
import sys
import threading
import time
import urllib.request

os.environ["ROS_DOMAIN_ID"] = "77"
import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import tornado.websocket  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402
from std_msgs.msg import Bool, Int32  # noqa: E402
from dagger_msgs.msg import PersonDetection, PersonDetections  # noqa: E402

PORT = 18080
LIB = "/home/jetson/ros2_ws/install/dagger_bringup/lib/dagger_bringup"
results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""), flush=True)


env = dict(os.environ)
procs = [subprocess.Popen([f"{LIB}/{exe}", "--ros-args", *args], env=env, start_new_session=True,
                          stdout=open(f"/tmp/test_ui_{exe}.log", "w"), stderr=subprocess.STDOUT)
         for exe, args in (("person_tracker", ["-p", "image_rotation_deg:=180",
                                               "-p", "select_requires_range:=false"]),
                           ("ui_server", ["-p", f"port:={PORT}", "-p", "allow_follow:=true",
                                          "-p", "deadman_s:=1.5"]))]

rclpy.init()
n = Node("fake_camera")
be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
p_img = n.create_publisher(CompressedImage, "/follow/ui/image/compressed", be)
p_det = n.create_publisher(PersonDetections, "/follow/person_detections", 10)
enable_log, select_log = [], []
n.create_subscription(Bool, "/follow/enable", lambda m: enable_log.append((time.time(), m.data)), 10)
n.create_subscription(Int32, "/follow/select", lambda m: select_log.append(m.data), 10)
from dagger_msgs.msg import MarkerTrack, MarkerTracks, PersonTracks  # noqa: E402
p_mk = n.create_publisher(MarkerTracks, "/follow/marker_tracks", 10)
mk_on, mk_sel = [False], [-1]          # a stand-in marker_tracker: one marker (id 7) per frame
hop = {"tracks": [], "image": []}
n.create_subscription(PersonTracks, "/follow/tracks", lambda m: hop["tracks"].append(
    time.time() - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)), 10)
n.create_subscription(CompressedImage, "/follow/ui/image/compressed", lambda m: hop["image"].append(
    time.time() - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)), be)

img = np.zeros((480, 640, 3), np.uint8)
cv2.putText(img, "fake camera", (180, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
img[::7] = np.random.randint(0, 255, (len(img[::7]), 640, 3), np.uint8)   # texture ~ real JPEG size
JPG = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
k = [0]


def tick():
    k[0] += 1
    st = n.get_clock().now().to_msg()
    d = PersonDetections()
    d.header.stamp, d.header.frame_id = st, "d435_color_optical_frame"
    d.image_width, d.image_height = 640, 480
    for cx in (200 + k[0], 450):                    # one walks right slowly, one stands
        det = PersonDetection()
        det.x1, det.y1, det.x2, det.y2, det.score = cx - 30.0, 160.0, cx + 30.0, 320.0, 0.9
        d.detections.append(det)
    im = CompressedImage()
    im.header, im.format = d.header, "jpeg"
    im.data = array.array("B", JPG)    # bytes would cost ~30 ms per message in rclpy
    p_img.publish(im)
    p_det.publish(d)
    if mk_on[0]:
        mk = MarkerTracks()
        mk.header, mk.image_width, mk.image_height, mk.image_rotation_deg = d.header, 640, 480, 180
        mk.id_base, mk.selected_id = 1000000, mk_sel[0]
        mk.target_state = "TRACKING" if mk_sel[0] >= 0 else "NONE"
        t = MarkerTrack()
        t.id, t.marker_id, t.visible, t.confirmed = 1000007, 7, True, True
        t.corners = [300.0, 200.0, 340.0, 202.0, 338.0, 242.0, 298.0, 240.0]
        t.range_m, t.bearing_deg, t.heading_deg, t.range_source = 3.2, 1.5, float("nan"), "lidar"
        mk.tracks.append(t)
        p_mk.publish(mk)


n.create_timer(0.1, tick)
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()


class Page:
    """Browser stand-in: a background reader collects every message as it arrives
    (event-driven, like a browser's onmessage); the test inspects the inbox."""

    def __init__(self, conn):
        self.c = conn
        self.inbox = []                            # (arrival time, message)
        self.closed = None
        self.reader = asyncio.ensure_future(self._read())

    async def _read(self):
        while True:
            m = await self.c.read_message()
            if m is None:
                self.closed = (self.c.close_code, self.c.close_reason)
                return
            self.inbox.append((time.time(), m))

    async def send(self, text):
        await self.c.write_message(text)

    def close(self):
        self.c.close()


async def page(query=""):
    return Page(await tornado.websocket.websocket_connect(f"ws://127.0.0.1:{PORT}/ws{query}",
                                                          max_message_size=2 ** 22))


def raw_slow_client(pause_s):
    """A page on a SLOW link: tiny receive buffer, reads nothing for pause_s, then
    returns (age of the first frame, frames until a fresh one, seconds from the first
    frame to the fresh one)."""
    import base64
    import socket
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32768)
    s.connect(("127.0.0.1", PORT))
    key = base64.b64encode(os.urandom(16)).decode()
    s.sendall((f"GET /ws HTTP/1.1\r\nHost: 127.0.0.1:{PORT}\r\nUpgrade: websocket\r\n"
               f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
               f"Sec-WebSocket-Version: 13\r\n\r\n").encode())
    buf = b""
    while b"\r\n\r\n" not in buf:
        buf += s.recv(4096)
    buf = buf.split(b"\r\n\r\n", 1)[1]
    time.sleep(pause_s)                           # the link stalls: nothing is read

    def read_exact(n):
        nonlocal buf
        while len(buf) < n:
            buf += s.recv(65536)
        out, buf = buf[:n], buf[n:]
        return out

    ages, t_first = [], None
    while len(ages) < 40:
        b0, b1 = read_exact(2)
        ln = b1 & 0x7F
        if ln == 126:
            ln = struct.unpack(">H", read_exact(2))[0]
        elif ln == 127:
            ln = struct.unpack(">Q", read_exact(8))[0]
        payload = read_exact(ln)
        if b0 & 0x0F == 2:                        # binary = a frame
            ages.append(time.time() - parse(payload)[0]["stamp"])
            t_first = t_first or time.time()
            if ages[-1] < 0.3:
                break
    s.close()
    return ages[0], len(ages), time.time() - t_first


def parse(buf):
    ln = struct.unpack(">I", buf[:4])[0]
    return json.loads(buf[4:4 + ln]), buf[4 + ln:]


async def frames(ws, seconds, hb=True):
    """Collect the frames that arrive during `seconds`, heartbeating like the page."""
    start, end, last_hb = len(ws.inbox), time.time() + seconds, 0
    while time.time() < end:
        if ws.closed is not None:
            raise ConnectionError(f"page connection closed: {ws.closed}")
        if hb and time.time() - last_hb > 0.25:
            await ws.send(json.dumps({"type": "hb", "t": time.time()}))
            last_hb = time.time()
        await asyncio.sleep(0.02)
    return [(t, *parse(m)) for t, m in ws.inbox[start:] if isinstance(m, bytes)]


async def window(ws, seconds, hb=True):
    """Frames AND 'boxes' messages that arrive during `seconds` (video first:
    each frame's boxes come as a separate text message, after the frame)."""
    i = len(ws.inbox)
    fr = await frames(ws, seconds, hb)
    bx = [(t, json.loads(m)) for t, m in ws.inbox[i:] if isinstance(m, str)]
    return fr, [(t, b) for t, b in bx if b.get("type") == "boxes"]


async def main():
    await asyncio.sleep(4.0)                                   # nodes up + discovery
    body = urllib.request.urlopen(f"http://127.0.0.1:{PORT}/", timeout=3).read().decode()
    check("page served at /", "Dagger follow" in body and "<canvas" in body and "/ws" in body)
    check("page runs on older Android WebViews (no DOMMatrix / DOMPoint), has the handset layout",
          "new DOMMatrix" not in body and "new DOMPoint" not in body and "body.handset" in body
          and 'id="lite"' in body and 'id="full"' in body)

    # LITE video for a slow link (SIYI MK32 at range): per page, the same frames
    small, big = await page("?video=lite"), await page()
    (got_s, bx_s), (got_b, bx_b) = await asyncio.gather(window(small, 1.5), window(big, 1.5))
    dims = lambda g: {cv2.imdecode(np.frombuffer(j, np.uint8), cv2.IMREAD_COLOR).shape[:2] for _, _, j in g}  # noqa: E731
    size = lambda g: sorted(len(j) for _, _, j in g)[len(g) // 2] if g else 0  # noqa: E731
    check("LITE page (/ws?video=lite): 320x240 frames, header video=lite; the other page still full 640x480",
          len(got_s) >= 6 and len(got_b) >= 6 and dims(got_s) == {(240, 320)} and dims(got_b) == {(480, 640)}
          and all(h.get("video") == "lite" for _, h, _ in got_s) and all(h.get("video") == "full" for _, h, _ in got_b),
          f"{len(got_s)} / {len(got_b)} frames, {dims(got_s)} / {dims(got_b)}, {size(got_s)} / {size(got_b)} bytes")
    check("... and both pages get the same frames and the same boxes",
          bool({h["seq"] for _, h, _ in got_s} & {h["seq"] for _, h, _ in got_b})
          and len(bx_s) >= 6 and bool({b["stamp"] for _, b in bx_s} & {b["stamp"] for _, b in bx_b}))
    await big.send(json.dumps({"type": "video", "mode": "lite"}))
    await frames(big, 0.3)
    got = await frames(big, 1.0)
    check("switching a page to LITE by message takes effect",
          len(got) >= 6 and dims(got) == {(240, 320)}, f"{dims(got)}")
    small.close(); big.close()
    await asyncio.sleep(0.5)

    ws = await page()
    if True:
        got, bx = await window(ws, 3.0)
        boxed = {b["stamp"]: b for _, b in bx}
        mine = [h for _, h, _ in got if h["stamp"] in boxed]
        check("frames arrive at ~10 Hz", 25 <= len(got) <= 33, f"{len(got)} in 3 s")
        check("every frame gets its own boxes (same stamp, 2 people) as a separate message",
              len(mine) >= len(got) - 1 and all(len(boxed[h["stamp"]]["tracks"]) == 2 for h in mine),
              f"{len(mine)}/{len(got)} frames with boxes")
        check("JPEG intact", all(j[:2] == b"\xff\xd8" and j[-2:] == b"\xff\xd9" for _, _, j in got))
        med = lambda v: sorted(v)[len(v) // 2]  # noqa: E731
        ages = [(h["sent"] - h["stamp"]) * 1000 for _, h, _ in got]
        img, trk = 1000 * med(hop["image"]), 1000 * med(hop["tracks"])
        # the harness's own Python delivery (fake camera -> its own subscriber) is
        # tens of ms on this CPU; judge only what ui_server ADDS on top of it
        check("VIDEO FIRST: a frame leaves < 15 ms after its image arrives (it does not wait for its boxes)",
              med(ages) - img < 15, f"stamp->send median {med(ages):.1f} ms vs stamp->image {img:.1f} ms")
        bage = [(t - b["stamp"]) * 1000 for t, b in bx]
        check("... and its boxes leave < 15 ms after the tracks arrive",
              med(bage) - trk < 15, f"stamp->boxes here median {med(bage):.1f} ms vs stamp->tracks {trk:.1f} ms")
        rx = sorted((t - h["stamp"]) * 1000 for t, h, _ in got)
        print(f"         loopback stamp -> client receive: median {rx[len(rx) // 2]:.1f} ms, max {rx[-1]:.1f} ms")

        tid = bx[-1][1]["tracks"][0]["id"]
        await ws.send(json.dumps({"type": "select", "id": tid}))
        got, bx = await window(ws, 1.0)
        check(f"select #{tid} reaches the tracker", tid in select_log and bx[-1][1]["selected"] == tid
              and bx[-1][1]["state"] == "TRACKING", f"state {bx[-1][1].get('state')}")

        # ---- ArUco markers (marker_tracker's tracks, aligned to the same frame) ----
        mk_on[0] = True
        await frames(ws, 0.5)
        got, bx = await window(ws, 2.0)
        withm = [b for _, b in bx if b.get("markers") and b.get("tracks") is not None]
        m0 = withm[-1]["markers"][0] if withm else {}
        check("marker tracker up: every frame's boxes carry its markers AND its people",
              16 <= len(got) <= 23 and len(withm) >= len(got) - 1, f"{len(withm)} boxes with both / {len(got)} frames")
        check("marker outline, id, range reach the page; NaN heading -> null",
              m0.get("id") == 1000007 and m0.get("mid") == 7 and len(m0.get("corners", [])) == 8
              and m0.get("ok") is True and m0.get("range") == 3.2 and m0.get("heading") is None, str(m0))
        check("a person stays the selection while no marker is selected",
              bx[-1][1]["selected"] == tid, str(bx[-1][1]["selected"]))
        await ws.send(json.dumps({"type": "select", "id": 1000007}))
        await frames(ws, 0.3)
        mk_sel[0] = 1000007                       # what marker_tracker does on that select
        got, bx = await window(ws, 1.0)
        check("clicking the marker sends its id; the page then shows the marker as the target",
              1000007 in select_log and bx[-1][1]["selected"] == 1000007
              and bx[-1][1]["state"] == "TRACKING", f"{bx[-1][1].get('selected')} {bx[-1][1].get('state')}")
        mk_on[0], mk_sel[0] = False, -1
        got, bx = await window(ws, 2.5)
        tail = [b for t_, b in bx if t_ > got[0][0] + 1.5]
        stall = max(b_[0] - a_[0] for a_, b_ in zip(got, got[1:])) if len(got) > 1 else 9.9
        # no stall is the point; the exact count depends on the harness's own 10 Hz
        # timer under load (7-10 in the last second at load average 6, 2026-09-30)
        check("marker tracker stops: video keeps flowing (no gap > 0.5 s), boxes no longer wait for markers",
              len(got) >= 18 and stall < 0.5 and len(tail) >= 5 and all("markers" not in h for h in tail),
              f"{len(got)} frames in 2.5 s, longest gap {stall:.2f} s, {len(tail)} in the last second")

        await ws.send(json.dumps({"type": "follow"}))
        await frames(ws, 0.5)
        check("FOLLOW publishes enable=true", any(v for _, v in enable_log))
        t0 = time.time()
        enable_log.clear()
        await frames(ws, 2.5, hb=False)                        # page stops sending heartbeats
        off = [t for t, v in enable_log if v is False]
        check("DEADMAN: no heartbeat -> enable=false within ~1.7 s", bool(off) and off[0] - t0 < 2.0,
              f"after {off[0] - t0:.2f} s" if off else "never")

        # Heartbeats first: FOLLOW from a page whose heartbeat is still stale (it was
        # just silent for 2.5 s) is switched off again at once by the deadman --
        # fail-safe, and it made this check race (follow was already off at the
        # disconnect, 2026-09-30). A browser sends heartbeats all the time.
        await frames(ws, 0.6)
        enable_log.clear()
        await ws.send(json.dumps({"type": "follow"}))
        await frames(ws, 0.5)
        armed = bool(enable_log) and enable_log[-1][1] is True
        ws.close()
    enable_log.clear()
    t0 = time.time()
    await asyncio.sleep(1.0)
    off = [t for t, v in enable_log if v is False]
    check("DEADMAN: page disconnects while following -> enable=false", armed and bool(off),
          (f"after {off[0] - t0:.2f} s" if off else "never") + ("" if armed else " (follow was not on)"))

    # the deadman belongs to the page that pressed FOLLOW: another page staying
    # connected (a laptop left open) must not keep the rover going
    op, other = await page("?video=lite"), await page()
    await asyncio.gather(frames(op, 0.6), frames(other, 0.6))
    enable_log.clear()
    await op.send(json.dumps({"type": "follow"}))
    await asyncio.gather(frames(op, 0.5), frames(other, 0.5))
    armed = bool(enable_log) and enable_log[-1][1] is True
    enable_log.clear()
    t0 = time.time()
    await asyncio.gather(frames(op, 2.5, hb=False), frames(other, 2.5))   # the operator's link stalls
    off = [t for t, v in enable_log if v is False]
    check("DEADMAN: the FOLLOW page goes silent, another page still heartbeating -> enable=false",
          armed and bool(off) and off[0] - t0 < 2.0,
          (f"after {off[0] - t0:.2f} s" if off else "never") + ("" if armed else " (follow was not on)"))
    await asyncio.gather(frames(op, 0.6), frames(other, 0.6))
    enable_log.clear()
    await op.send(json.dumps({"type": "follow"}))
    await asyncio.gather(frames(op, 0.5), frames(other, 0.5))
    armed = bool(enable_log) and enable_log[-1][1] is True
    enable_log.clear()
    t0 = time.time()
    op.close()
    await frames(other, 1.0)
    off = [t for t, v in enable_log if v is False]
    check("DEADMAN: the FOLLOW page disconnects, another page still connected -> enable=false",
          armed and bool(off), (f"after {off[0] - t0:.2f} s" if off else "never") + ("" if armed else " (follow was not on)"))
    other.close()
    await asyncio.sleep(0.5)

    loop = asyncio.get_event_loop()
    age0, drained, burst = await loop.run_in_executor(None, raw_slow_client, 2.0)
    # What was already in flight when the link stalled is delivered first (TCP);
    # the cap keeps that to a couple of frames, delivered as one burst, so the
    # page (which decodes only the newest waiting frame) is fresh again at once.
    check("slow link (2 s stall): backlog <= 3 frames, fresh again within 0.3 s",
          drained <= 4 and burst < 0.3,
          f"{drained - 1} stale frame(s) (oldest {age0:.2f} s), fresh after {1000 * burst:.0f} ms")


try:
    asyncio.get_event_loop().run_until_complete(main())
except Exception:                                  # a crash must fail the test, loudly
    import traceback
    traceback.print_exc()
    results.append(False)
finally:
    for p in procs:
        os.killpg(p.pid, signal.SIGINT)
    time.sleep(1.5)
    for p in procs:
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGKILL)
    n.destroy_node()
    rclpy.try_shutdown()
    print(f"\n{sum(results)}/{len(results)} passed", flush=True)
    sys.stdout.flush()
    os._exit(0 if results and all(results) else 1)   # skip interpreter teardown of the spin thread
