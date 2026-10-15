"""The video channel: ws://<rover>:8090/video, beside /api so a frame never delays a STOP.

OUT only (nothing comes in): Envelope.video_frame -- the follow camera's JPEG,
as the person detector publishes it on /follow/ui/image/compressed -- and
Envelope.targets -- who is in it (follow.targets_msg), when the trackers speak.
LATEST FRAME ONLY: a frame that would queue behind an unsent one is dropped, so
a slow link loses frames and never falls behind. ?lite=1: every second frame
only (the frames are not re-encoded on the Jetson). The pairing and safety
checks are /api's; this socket can command nothing.
"""
import tornado.websocket

from . import api_pb2 as pb
from .rover_state import wall_ms


class VideoSocket(tornado.websocket.WebSocketHandler):

    def initialize(self, gw):
        self.gw = gw
        self.sending = None
        self.lite = False
        self.count = 0

    def check_origin(self, origin):
        return True                        # the handset app is not a web page

    def open(self):
        self.set_nodelay(True)
        self.lite = self.get_argument("lite", "0") == "1"
        self.gw.video_sockets.add(self)
        self.gw.log.info(f"video to {self.request.remote_ip} ({'lite' if self.lite else 'full'})")

    def on_close(self):
        self.gw.video_sockets.discard(self)
        self.gw.log.info(f"video to {self.request.remote_ip} closed")

    def on_message(self, raw):
        pass

    def send_frame(self, frame):
        self.count += 1
        if self.lite and self.count % 2:
            return
        if self.sending is not None and not self.sending.done():
            return                         # the last one is still going: drop this one
        self.sending = self._send(video_frame=frame)

    def send_targets(self, targets):
        self._send(targets=targets)

    def _send(self, **body):
        env = pb.Envelope(rover_id=self.gw.rover_id, sent_at_ms=wall_ms(), **body)
        try:
            return self.write_message(env.SerializeToString(), binary=True)
        except tornado.websocket.WebSocketClosedError:
            return None
