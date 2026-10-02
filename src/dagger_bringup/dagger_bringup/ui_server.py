#!/usr/bin/env python3
"""Browser UI for click-to-select following: video + boxes out, clicks in.

    http://<rover-ip>:8080        the page (web/index.html)
    ws://<rover-ip>:8080/ws       one WebSocket per open page

Built on tornado (apt python3-tornado 6.1). Not python3-websockets: Ubuntu
22.04's 9.1 passes asyncio.Lock(loop=None), which Python 3.10 rejects -- a
connection never completes (reproduced 2026-09-29) -- and it also cuts off
HTTP bodies served from its handshake hook.

OUT, per camera frame the detector processed: ONE binary message =
    4-byte big-endian length | JSON header | JPEG
where the header carries that frame's tracks (person_tracker): the image and its
boxes travel together, so what the operator clicks is exactly what they see.
LATEST FRAME ONLY: each page has one slot; a frame that arrives while the
previous one is still being sent replaces the waiting one. A slow link drops
frames, it never queues them, so the picture cannot fall behind reality.
No WebSocket compression (JPEG does not compress). Cross-origin WebSockets are
refused (tornado's default), so another website open in the operator's browser
cannot command the rover.

IN (JSON text): hb (heartbeat, answered with the server clock for the page's
latency meter), select {id}, clear, follow, stop.
DEADMAN: if following was enabled FROM THE UI and THE PAGE THAT PRESSED FOLLOW
has sent no heartbeat for deadman_s, or has disconnected, follow is disabled --
the rover stops when the operator's link is gone, whatever other pages are still
open (a laptop left connected must not keep the rover going for an operator on
the SIYI handset whose radio dropped). Following enabled elsewhere (CLI,
follow_solo.sh) is not touched by the deadman.
VIDEO: per page, full (640x480) or LITE (320x240, /ws?video=lite or a
{"type": "video", "mode": "lite"} message) for a slow link.

Threads: rclpy spins in a background executor thread; tornado owns the asyncio
loop in the main thread; frames cross with call_soon_threadsafe.
"""
import asyncio
import collections
import json
import os
import socket
import struct
import sys
import threading
import time

import rclpy
import tornado.web
import tornado.websocket
from ament_index_python.packages import get_package_share_directory
from dagger_msgs.msg import MarkerTracks, PersonTracks
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Int32, String


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def num(v, nd=3):
    """JSON-safe float (NaN/inf -> null)."""
    return round(float(v), nd) if v == v and abs(v) != float("inf") else None


def shrink(jpeg, quality=70):
    """Half-size (320x240) re-encode of a frame for a page on a slow link (LITE):
    decoded straight at half size, 2.4 ms, ~10 KB instead of ~29 KB (aruco2 frames,
    2026-10-02). Only worth it when the radio is slow: at short range the SIYI link
    carries a full frame in about the time this takes."""
    import cv2
    import numpy as np
    im = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_REDUCED_COLOR_2)
    ok, enc = cv2.imencode(".jpg", im, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return enc.tobytes() if ok else jpeg


class Hub:
    """asyncio side: the latest frame and the connected pages."""

    def __init__(self):
        self.frame = None                # (header dict, jpeg bytes)
        self.seq = 0
        self.clients = {}                # PageSocket -> last heartbeat (monotonic)
        self.lite_frame = (0, None)      # (seq, future of the shrunk JPEG): made once, shared

    def lite(self, seq, jpeg):
        """Awaitable LITE JPEG of frame `seq`, encoded in a worker thread (cv2
        releases the GIL) so the loop keeps serving other pages meanwhile."""
        if self.lite_frame[0] != seq:
            self.lite_frame = (seq, asyncio.get_event_loop().run_in_executor(None, shrink, jpeg))
        return self.lite_frame[1]

    def publish(self, header, jpeg):
        self.seq += 1
        header["seq"] = self.seq
        self.frame = (header, jpeg)
        for c in self.clients:
            c.wake.set()


class UiServer(Node):

    def __init__(self, loop, hub):
        super().__init__("ui_server")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("image_topic", "/follow/ui/image/compressed")
        self.declare_parameter("tracks_topic", "/follow/tracks")
        self.declare_parameter("select_topic", "/follow/select")
        self.declare_parameter("enable_topic", "/follow/enable")
        self.declare_parameter("deadman_s", 1.5)
        # A frame whose tracks have not arrived after this is sent without boxes
        # (tracker down) rather than held back.
        self.declare_parameter("pair_timeout_s", 0.2)
        # FOLLOW from the page. Off until the tracker drives /follow/target from
        # the selection (phase 2); STOP always works.
        self.declare_parameter("allow_follow", False)
        # per-page socket send buffer request (bytes); Linux doubles it -> ~128 KiB,
        # ~3 frames of 640x480 JPEG q70
        self.declare_parameter("sndbuf_bytes", 65536)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.host, self.port = g("host"), int(g("port"))
        self.deadman_s = float(g("deadman_s"))
        self.pair_timeout = float(g("pair_timeout_s"))
        self.allow_follow = bool(g("allow_follow"))
        self.sndbuf = int(g("sndbuf_bytes"))
        self.loop, self.hub = loop, hub

        self.lock = threading.Lock()
        self.images = collections.OrderedDict()   # stamp -> (msg, arrival)
        self.tracks = collections.OrderedDict()   # stamp -> msg
        self.markers = collections.OrderedDict()  # stamp -> MarkerTracks (marker_tracker, optional)
        self.marker_seen = -1e9                   # last MarkerTracks arrival: is marker_tracker up?
        self.follow_enabled = None
        self.follow_state = ""                     # follow_controller's /follow/state
        self.target_source = ""
        self.target_range = None
        self.ui_armed = False                     # following was enabled from the page
        self.armed_page = None                    # ... from THIS page: its heartbeat is the deadman

        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(CompressedImage, g("image_topic"), self.on_image, be)
        self.create_subscription(PersonTracks, g("tracks_topic"), self.on_tracks, 10)
        self.create_subscription(MarkerTracks, "/follow/marker_tracks", self.on_markers, 10)
        self.create_subscription(Bool, g("enable_topic"),
                                 lambda m: setattr(self, "follow_enabled", m.data), 10)
        self.create_subscription(String, "/follow/state", self.on_state, 10)
        self.create_subscription(String, "/follow/target_source",
                                 lambda m: setattr(self, "target_source", m.data), 10)
        self.create_subscription(PoseStamped, "/follow/target", self.on_target, be)
        self.select_pub = self.create_publisher(Int32, g("select_topic"), 10)
        self.enable_pub = self.create_publisher(Bool, g("enable_topic"), 10)
        self.create_timer(0.05, self.flush_unpaired)

    # ---------------- ROS thread ----------------
    def on_state(self, m):
        self.follow_state = m.data
        if m.data.startswith("disabled"):
            self.ui_armed, self.armed_page = False, None   # the controller stood down by itself

    def on_target(self, m):
        p = m.pose.position
        self.target_range = (p.x ** 2 + p.y ** 2) ** 0.5

    def on_image(self, m):
        with self.lock:
            self.images[stamp_s(m.header)] = (m, time.monotonic())
            while len(self.images) > 5:
                self.images.popitem(last=False)
            self.pair(stamp_s(m.header))

    def on_tracks(self, m):
        with self.lock:
            self.tracks[stamp_s(m.header)] = m
            while len(self.tracks) > 5:
                self.tracks.popitem(last=False)
            self.pair(stamp_s(m.header))

    def on_markers(self, m):
        with self.lock:
            self.marker_seen = time.monotonic()
            self.markers[stamp_s(m.header)] = m
            while len(self.markers) > 5:
                self.markers.popitem(last=False)
            self.pair(stamp_s(m.header))

    def pair(self, st):
        """Caller holds the lock. Emit the frame at stamp st once all parts exist:
        the image, its person tracks and -- while marker_tracker is publishing --
        its marker tracks."""
        want_markers = time.monotonic() - self.marker_seen < 1.0
        if st in self.images and st in self.tracks and (st in self.markers or not want_markers):
            img, _ = self.images.pop(st)
            trk = self.tracks.pop(st)
            mk = self.markers.pop(st, None)
            for d in (self.images, self.tracks, self.markers):   # anything older can no longer pair
                for k in [k for k in d if k < st]:
                    del d[k]
            self.emit(img, trk, mk)

    def flush_unpaired(self):
        now = time.monotonic()
        with self.lock:
            for st in [k for k, (_, t0) in self.images.items() if now - t0 > self.pair_timeout]:
                img, _ = self.images.pop(st)
                # a part that never came is shown as missing, the rest as they are
                self.emit(img, self.tracks.pop(st, None), self.markers.pop(st, None))

    def emit(self, img, trk, mk=None):
        h = {"type": "frame", "stamp": stamp_s(img.header),
             "follow": self.follow_enabled, "follow_state": self.follow_state,
             "allow_follow": self.allow_follow,
             "source": self.target_source,
             "range": num(self.target_range) if self.target_range is not None else None}
        if trk is not None:
            h.update(w=trk.image_width, h=trk.image_height, rot=trk.image_rotation_deg,
                     selected=trk.selected_id, state=trk.target_state,
                     tracks=[{"id": t.id, "box": [num(t.x1, 1), num(t.y1, 1), num(t.x2, 1),
                                                  num(t.y2, 1)],
                              "score": num(t.score, 2), "visible": t.visible,
                              "range": num(t.range_m, 2), "bearing": num(t.bearing_deg, 1),
                              "ranged": t.ranged}
                             for t in trk.tracks])
        else:
            h.update(tracks=None, selected=-1, state="NO_TRACKER")
        if mk is not None:
            # corners: the marker's outline in native image pixels, like the boxes
            h["markers"] = [{"id": t.id, "mid": t.marker_id,
                             "corners": [num(v, 1) for v in t.corners],
                             "visible": t.visible, "ok": t.confirmed,
                             "range": num(t.range_m, 2), "bearing": num(t.bearing_deg, 1),
                             "heading": num(t.heading_deg, 0), "src": t.range_source}
                            for t in mk.tracks]
            if trk is None:
                h.update(w=mk.image_width, h=mk.image_height, rot=mk.image_rotation_deg)
            if mk.selected_id >= 0:                # a marker is the selection: its state counts
                h.update(selected=mk.selected_id, state=mk.target_state)
        self.loop.call_soon_threadsafe(self.hub.publish, h, bytes(img.data))

    # ---------------- commands (loop thread; rclpy publish is thread-safe) ----------------
    def set_follow(self, on, why, page=None):
        for _ in range(3):                         # a single message can be dropped
            self.enable_pub.publish(Bool(data=on))
        self.ui_armed, self.armed_page = on, (page if on else None)
        self.get_logger().info(f"follow {'ENABLED' if on else 'DISABLED'} from the UI ({why})")

    def command(self, msg, page=None):
        t = msg.get("type")
        if t == "select":
            tid = int(msg["id"])
            self.select_pub.publish(Int32(data=tid))
            self.get_logger().info(f"UI: select #{tid}")
            return None
        if t == "clear":
            self.select_pub.publish(Int32(data=-1))
            return None
        if t == "stop":
            self.set_follow(False, "STOP pressed")
            return None
        if t == "follow":
            if not self.allow_follow:
                return {"type": "error", "msg": "FOLLOW from the page is not enabled yet "
                                                "(allow_follow:=false)"}
            self.set_follow(True, "FOLLOW pressed", page)
            return None
        return {"type": "error", "msg": f"unknown command {t!r}"}


class PageHandler(tornado.web.RequestHandler):

    def initialize(self, body):
        self.body = body

    def get(self):
        self.set_header("Content-Type", "text/html; charset=utf-8")
        self.set_header("Cache-Control", "no-store")
        self.write(self.body)


class PageSocket(tornado.websocket.WebSocketHandler):

    def initialize(self, node, hub):
        self.node, self.hub = node, hub
        self.wake = asyncio.Event()
        self.sender = None
        self.lite = False                # half-size video for this page (/ws?video=lite, or a message)

    def open(self):
        self.hub.clients[self] = time.monotonic()
        self.lite = self.get_query_argument("video", "full") == "lite"
        self.set_nodelay(True)                     # small replies (pong) must not wait for Nagle
        # Latest-only must hold on a slow link too: the kernel send buffer grows to
        # megabytes = seconds of OLD frames queued below the app. Cap it at ~3 JPEGs
        # so backpressure reaches send_frames(), which then skips to the newest.
        # (tornado has moved the stream onto ws_connection by the time open() runs)
        sock = self.ws_connection.stream.socket
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.node.sndbuf)
        self.sndbuf = sock.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)  # kernel doubles it
        self.node.get_logger().info(
            f"UI connected: {self.request.remote_ip} ({len(self.hub.clients)} page(s), "
            f"send buffer {self.sndbuf // 1024} KiB, video {'LITE' if self.lite else 'full'})")
        self.sender = asyncio.ensure_future(self.send_frames())
        if self.hub.frame is not None:
            self.wake.set()

    async def send_frames(self):
        last = 0
        try:
            while True:
                await self.wake.wait()
                self.wake.clear()
                if self.hub.frame is None or self.hub.frame[0]["seq"] == last:
                    continue
                header, jpeg = self.hub.frame
                last = header["seq"]
                if self.lite:
                    jpeg = await self.hub.lite(last, jpeg)
                hdr = json.dumps(dict(header, sent=time.time(), video="lite" if self.lite else "full"),
                                 separators=(",", ":")).encode()
                # awaits until tornado has handed the bytes to the socket: while it
                # waits, newer frames only overwrite hub.frame -> latest-only
                await self.write_message(struct.pack(">I", len(hdr)) + hdr + jpeg, binary=True)
        except (tornado.websocket.WebSocketClosedError, asyncio.CancelledError):
            pass

    def on_message(self, raw):
        if not isinstance(raw, str):
            return
        try:
            msg = json.loads(raw)
        except ValueError:
            return
        if msg.get("type") == "hb":
            self.hub.clients[self] = time.monotonic()
            self.reply({"type": "pong", "t": msg.get("t"), "server": time.time()})
            return
        if msg.get("type") == "video":                 # this page only
            self.lite = msg.get("mode") == "lite"
            self.node.get_logger().info(f"UI {self.request.remote_ip}: video {'LITE' if self.lite else 'full'}")
            return
        try:
            reply = self.node.command(msg, self)
        except (KeyError, TypeError, ValueError) as e:
            reply = {"type": "error", "msg": f"bad command: {e}"}
        if reply:
            self.reply(reply)

    def reply(self, obj):
        try:
            self.write_message(json.dumps(obj))
        except tornado.websocket.WebSocketClosedError:
            pass

    def on_close(self):
        if self.sender is not None:
            self.sender.cancel()
        self.hub.clients.pop(self, None)
        self.node.get_logger().info(f"UI disconnected ({len(self.hub.clients)} page(s))")


async def deadman(node, hub):
    while True:
        await asyncio.sleep(0.2)
        if not node.ui_armed:
            continue
        beat = hub.clients.get(node.armed_page)    # the page that pressed FOLLOW, not any page
        if beat is None:
            node.set_follow(False, "DEADMAN: the page that pressed FOLLOW disconnected")
        elif time.monotonic() - beat > node.deadman_s:
            node.set_follow(False, f"DEADMAN: no heartbeat from the page that pressed FOLLOW "
                                   f"for {node.deadman_s:.1f} s")


def spin_quietly(executor):
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main():
    # Our own Ctrl-C handling: rclpy's handler would shut the ROS context down
    # FIRST, and the "follow off" below could then not be published at all
    # ("publisher's context is invalid", seen 2026-09-29).
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    hub = Hub()
    node = UiServer(loop, hub)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=spin_quietly, args=(executor,), daemon=True)
    spin.start()
    with open(os.path.join(get_package_share_directory("dagger_bringup"), "web",
                           "index.html"), "rb") as f:
        page = f.read()
    code = 0
    try:
        app = tornado.web.Application(
            [(r"/", PageHandler, {"body": page}), (r"/index.html", PageHandler, {"body": page}),
             (r"/ws", PageSocket, {"node": node, "hub": hub})],
            # keepalive: ping every 2 s, drop a silent peer after 6 s. The timeout
            # MUST exceed the interval -- 5/5 closed every page exactly 5 s after it
            # connected (first ping: "no pong since open"). Safety does not rely on
            # this; the 1.5 s heartbeat deadman does.
            websocket_ping_interval=2, websocket_ping_timeout=6,
            websocket_max_message_size=16384)
        app.listen(node.port, node.host)
        loop.create_task(deadman(node, hub))
        node.get_logger().info(f"click-to-follow UI on http://{node.host}:{node.port}/ "
                               f"(follow from page: {'on' if node.allow_follow else 'off'})")
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    except OSError as e:                           # port taken: say so and exit cleanly
        node.get_logger().fatal(f"cannot listen on {node.host}:{node.port}: {e}")
        code = 1
    finally:
        if node.ui_armed:
            node.set_follow(False, "ui_server shutting down")
        # stop spinning BEFORE destroying the node (destroying it under a running
        # executor thread aborted the process: "terminate called ...")
        executor.shutdown(timeout_sec=1.0)
        spin.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
