"""One app connection on ws://<rover>:8090/api: handshake, heartbeats, commands.

Binary frames, one dagger.api Envelope each (proto/dagger/api.proto, whose header
has the seven rules). Text frames and undecodable frames are ignored.

HANDSHAKE: the first message is Hello. HelloReply.accepted = protocol version 1.
Refused -> nothing more is sent on this connection, ever. Accepted -> RoverSettings,
then this connection gets State and Heartbeats on the gateway's timers and every
Event. Only an accepted connection counts for the link-loss policy.
COMMANDS: before the handshake -> REJECTED / NO_HANDSHAKE; addressed to another
rover -> REJECTED / WRONG_ROVER (an empty rover_id is accepted: STOP must not fail
on a formality). Results are cached by command_id across connections (rule 3), so
an app that resends after reconnecting gets the same answer and no second effect.
"""
import time

import tornado.websocket
from google.protobuf.message import DecodeError

from . import api_pb2 as pb
from .base import sites_pb
from .rover_state import mono_ms, wall_ms


def version_name(v):
    return pb.ProtocolVersion.Name(v) if v in pb.ProtocolVersion.values() else str(v)


class ApiSocket(tornado.websocket.WebSocketHandler):

    def initialize(self, gw):
        self.gw = gw
        self.accepted = False              # completed the handshake
        self.refused = False               # said Hello with another version: silence from now on
        self.last_heard = time.monotonic()
        self.peer_hb = None                # the app's last heartbeat: (seq, sent_time_ms, arrived mono ms)
        self.rtt_ms = None                 # from the app's echoes of OUR heartbeats
        self.hb_seq = 0
        self.state_seq = 0
        self.event_seq = 0

    def open(self):
        self.set_nodelay(True)             # a STOP answer must not wait for Nagle
        self.gw.sockets.add(self)
        self.gw.log.info(f"app connected from {self.request.remote_ip} (not yet handshaken)")

    def on_close(self):
        self.gw.sockets.discard(self)
        self.gw.log.info(f"app {self.request.remote_ip} disconnected"
                         + (" (was handshaken)" if self.accepted else ""))
        if self.accepted:
            self.accepted = False
            self.gw.app_gone()

    # ---------------- in ----------------
    def on_message(self, raw):
        if isinstance(raw, str):
            return
        env = pb.Envelope()
        try:
            env.ParseFromString(raw)
        except DecodeError:
            self.gw.log.warn(f"app {self.request.remote_ip}: undecodable frame ({len(raw)} bytes)")
            return
        if self.refused:
            return
        body = env.WhichOneof("body")
        if self.accepted:
            self.last_heard = time.monotonic()
        if body == "command":              # first: STOP jumps the queue
            self.on_command(env)
        elif body == "heartbeat":
            self.on_heartbeat(env.heartbeat)
        elif body == "hello":
            self.on_hello(env.hello)

    def on_hello(self, hello):
        ok = hello.protocol_version == pb.PROTOCOL_VERSION_1
        reply = pb.HelloReply(accepted=ok, protocol_version=pb.PROTOCOL_VERSION_1,
                              rover_id=self.gw.rover_id, rover_name=self.gw.rover_name,
                              rover_software_version=self.gw.software_version)
        if ok:
            reply.capabilities.extend(self.gw.capabilities)
        self.gw.log.info(f"app {self.request.remote_ip} Hello {version_name(hello.protocol_version)} "
                         f"'{hello.app_version}' client '{hello.client_id}': "
                         f"{'accepted' if ok else 'REFUSED'}")
        self.send(hello_reply=reply)
        if not ok:
            self.refused = True
            return
        first = not self.accepted
        self.accepted = True
        self.last_heard = time.monotonic()
        self.send(settings=self.gw.settings.msg())
        if self.gw.routes_on:
            self.send(routes=self.gw.route_list())   # Phase 3: the saved routes
        if self.gw.maps_on:
            self.send(maps=self.gw.map_list())       # Step C1: the saved maps
        if self.gw.base_on:
            self.send(base_sites=sites_pb(self.gw.node.base_sites))   # Step R1: the base's sites
        if first:
            self.send_state()              # the picture at once, not 200 ms later

    def on_heartbeat(self, hb):
        if not self.accepted:
            return
        now = mono_ms()
        self.peer_hb = (hb.seq, hb.sent_time_ms, now)
        if hb.echo_sent_time_ms:
            self.rtt_ms = max(0, now - hb.echo_sent_time_ms - hb.echo_hold_ms)

    def on_command(self, env):
        cmd = env.command
        if not self.accepted:
            self.result(cmd.command_id, pb.REJECT_REASON_NO_HANDSHAKE)
            return
        if env.rover_id and env.rover_id != self.gw.rover_id:
            self.result(cmd.command_id, pb.REJECT_REASON_WRONG_ROVER)
            return
        cached = self.gw.results.get(cmd.command_id) if cmd.command_id else None
        if cached is not None:
            self.send(command_result=cached)
            return
        res = self.gw.command(cmd, self)
        if res is None:                    # answered later (ImportMission reads the autopilot first)
            return
        if cmd.command_id:
            self.gw.remember(res)
        self.send(command_result=res)
        if cmd.WhichOneof("kind") == "stop":
            self.send_state()              # STOPPED now, not at the next tick

    def result(self, command_id, reason):
        """A rejection that is not remembered: the same id may succeed once the cause is gone."""
        self.send(command_result=pb.CommandResult(command_id=command_id,
                                                  outcome=pb.COMMAND_OUTCOME_REJECTED,
                                                  reject_reason=reason))

    # ---------------- out ----------------
    def send(self, **body):
        env = pb.Envelope(rover_id=self.gw.rover_id, sent_at_ms=wall_ms(), **body)
        try:
            self.write_message(env.SerializeToString(), binary=True)
        except tornado.websocket.WebSocketClosedError:
            pass

    def silence_ms(self):
        return int((time.monotonic() - self.last_heard) * 1000)

    def send_state(self):
        self.state_seq += 1
        st = self.gw.state_msg()
        st.seq = self.state_seq
        st.link.since_last_heard_ms = self.silence_ms()
        if self.rtt_ms is not None:
            st.link.round_trip_ms = self.rtt_ms
        self.send(state=st)

    def send_heartbeat(self):
        self.hb_seq += 1
        hb = pb.Heartbeat(seq=self.hb_seq, sent_time_ms=mono_ms())
        if self.peer_hb is not None:
            seq, sent, arrived = self.peer_hb
            hb.echo_seq, hb.echo_sent_time_ms = seq, sent
            hb.echo_hold_ms = mono_ms() - arrived
        self.send(heartbeat=hb)

    def send_event(self, ev):
        self.event_seq += 1
        e = pb.Event()
        e.CopyFrom(ev)
        e.seq = self.event_seq
        self.send(event=e)
