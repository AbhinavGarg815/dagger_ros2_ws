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
from dagger_msgs.msg import PersonTracks  # noqa: E402
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


async def page():
    return Page(await tornado.websocket.websocket_connect(f"ws://127.0.0.1:{PORT}/ws",
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


async def main():
    await asyncio.sleep(4.0)                                   # nodes up + discovery
    body = urllib.request.urlopen(f"http://127.0.0.1:{PORT}/", timeout=3).read().decode()
    check("page served at /", "Dagger follow" in body and "<canvas" in body and "/ws" in body)

    ws = await page()
    if True:
        got = await frames(ws, 3.0)
        paired = [h for _, h, j in got if h.get("tracks") is not None]
        check("frames arrive at ~10 Hz", 25 <= len(got) <= 33, f"{len(got)} in 3 s")
        check("every frame paired with its tracks (2 people)",
              len(paired) == len(got) and all(len(h["tracks"]) == 2 for h in paired),
              f"{len(paired)}/{len(got)} paired")
        check("JPEG intact", all(j[:2] == b"\xff\xd8" and j[-2:] == b"\xff\xd9" for _, _, j in got))
        med = lambda v: sorted(v)[len(v) // 2]  # noqa: E731
        ages = [(h["sent"] - h["stamp"]) * 1000 for _, h, _ in got]
        trk = 1000 * med(hop["tracks"])
        # the harness's own Python delivery (fake camera -> its own subscriber) is
        # tens of ms on this CPU; judge only what ui_server ADDS on top of the
        # tracks' ROS delivery
        check("ui_server adds < 15 ms between tracks arriving and the frame going out",
              med(ages) - trk < 15,
              f"stamp->send median {med(ages):.1f} ms vs stamp->tracks {trk:.1f} ms "
              f"(image hop {1000 * med(hop['image']):.1f} ms)")
        rx = sorted((t - h["stamp"]) * 1000 for t, h, _ in got)
        print(f"         loopback stamp -> client receive: median {rx[len(rx) // 2]:.1f} ms, max {rx[-1]:.1f} ms")

        tid = got[-1][1]["tracks"][0]["id"]
        await ws.send(json.dumps({"type": "select", "id": tid}))
        got = await frames(ws, 1.0)
        check(f"select #{tid} reaches the tracker", tid in select_log and got[-1][1]["selected"] == tid
              and got[-1][1]["state"] == "TRACKING", f"state {got[-1][1].get('state')}")

        await ws.send(json.dumps({"type": "follow"}))
        await frames(ws, 0.5)
        check("FOLLOW publishes enable=true", any(v for _, v in enable_log))
        t0 = time.time()
        enable_log.clear()
        await frames(ws, 2.5, hb=False)                        # page stops sending heartbeats
        off = [t for t, v in enable_log if v is False]
        check("DEADMAN: no heartbeat -> enable=false within ~1.7 s", bool(off) and off[0] - t0 < 2.0,
              f"after {off[0] - t0:.2f} s" if off else "never")

        await ws.send(json.dumps({"type": "follow"}))
        await frames(ws, 0.5)
        ws.close()
    enable_log.clear()
    t0 = time.time()
    await asyncio.sleep(1.0)
    off = [t for t, v in enable_log if v is False]
    check("DEADMAN: page disconnects -> enable=false", bool(off), f"after {off[0] - t0:.2f} s" if off else "never")

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
