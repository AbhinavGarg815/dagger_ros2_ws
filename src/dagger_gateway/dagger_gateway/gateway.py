#!/usr/bin/env python3
"""The operator app's gateway: the only thing the app talks to.

    ros2 run dagger_gateway gateway                  # ws://<rover>:8090/api
    ros2 run dagger_gateway gateway --stop-dry-run   # STOP changes the state only

One WebSocket per app, binary frames, one protobuf Envelope each; the contract
is proto/ (git submodule dagger_proto) -- read proto/README.md and the header of
proto/dagger/api.proto. The gateway owns the operating state, readiness, alerts,
the link-loss policy and what commands are accepted; the app shows and asks.
Video (Phase 4) gets its own WebSocket, never /api, so a frame never delays a STOP.

Built like dagger_bringup/ui_server.py: rclpy spins in a background thread,
tornado (apt python3-tornado 6.1) owns the asyncio loop in the main thread,
crossings go through call_soon_threadsafe. (Not python3-websockets: Ubuntu
22.04's 9.1 never completes a connection on Python 3.10 -- see ui_server.py.)

Ports in use elsewhere: 8080 follow page, 8081 round-trip test, 8082 fake rover
(PC), 8765 Foxglove.
"""
import asyncio
import collections
import signal
import sys
import threading
import time

import rclpy
import tornado.web
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.utilities import remove_ros_args

from . import api_pb2 as pb
from .ros_io import RosIO
from .rover_state import RoverCore, Settings
from .session import ApiSocket

try:
    from .build_info import VERSION
except ImportError:                        # running from a tree setup.py has not built
    VERSION = "gateway (unbuilt)"

CAPABILITIES = (pb.CAPABILITY_STOP, pb.CAPABILITY_READINESS, pb.CAPABILITY_ALERTS)


class Gateway:
    """asyncio side: the sessions, the core, the timers."""

    def __init__(self, node, loop):
        self.node, self.loop = node, loop
        self.log = node.get_logger()
        g = node.param
        self.rover_id, self.rover_name = g("rover_id"), g("rover_name")
        self.software_version = VERSION
        self.capabilities = CAPABILITIES
        self.settings = Settings(heartbeat_ms=int(g("heartbeat_period_ms")),
                                 state_ms=int(g("state_period_ms")),
                                 follow_stop_ms=int(g("follow_stop_after_silence_ms")),
                                 route_pause_ms=int(g("route_pause_after_silence_ms")),
                                 max_speed_mps=float(g("max_speed_mps")))
        self.sockets = set()
        self.results = collections.OrderedDict()   # command_id -> CommandResult (rule 3)
        self.core = RoverCore(self.settings, self.broadcast, node)
        node.on_estop_edge = lambda on: loop.call_soon_threadsafe(self.on_estop_edge, on)

    def apps(self):
        return [s for s in self.sockets if s.accepted]

    def broadcast(self, ev):
        for s in self.apps():
            s.send_event(ev)

    # ---------------- commands ----------------
    def command(self, cmd, sock):
        kind = cmd.WhichOneof("kind")
        ok = pb.CommandResult(command_id=cmd.command_id, outcome=pb.COMMAND_OUTCOME_ACCEPTED)
        if kind == "stop":
            self.log.warn(f"STOP from app {sock.request.remote_ip} ({cmd.command_id})")
            self.node.stop()
            self.core.stop()
            return ok
        return pb.CommandResult(command_id=cmd.command_id, outcome=pb.COMMAND_OUTCOME_REJECTED,
                                reject_reason=pb.REJECT_REASON_NOT_SUPPORTED)

    def remember(self, res):
        self.results[res.command_id] = res
        while len(self.results) > 512:
            self.results.popitem(last=False)

    def on_estop_edge(self, on):
        if on:
            self.core.stop()
        else:
            self.core.release()

    def app_gone(self):
        """An app that completed the handshake disconnected: silent from now on."""
        self.core.link_tick([float("inf")])

    # ---------------- state ----------------
    def state_msg(self):
        st = pb.RoverState(operating_state=self.core.state, armed=False, speed_mps=0.0)
        st.readiness.extend(self.core.readiness_msgs())
        st.active_alerts.extend(self.core.alerts.sorted())
        return st

    def tick_state(self):
        self.core.set_following(self.node.following())
        self.core.link_tick([s.silence_ms() for s in self.apps()])
        for s in self.apps():
            s.send_state()

    def tick_heartbeat(self):
        for s in self.apps():
            s.send_heartbeat()


async def every(period_s, fn, log):
    """fn() at a fixed rate (no drift); an exception is logged, the timer goes on."""
    nxt = time.monotonic()
    while True:
        nxt += period_s
        try:
            fn()
        except Exception as e:  # noqa: BLE001
            log.error(f"{fn.__name__}: {e!r}")
        await asyncio.sleep(max(0.0, nxt - time.monotonic()))


def spin_quietly(executor):
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main():
    args = remove_ros_args(sys.argv)[1:]
    dry = "--stop-dry-run" in args
    # Our own Ctrl-C handling, as ui_server: rclpy's handler would shut ROS down first.
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    node = RosIO()
    node.dry_run = node.dry_run or dry
    gw = Gateway(node, loop)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=spin_quietly, args=(executor,), daemon=True)
    spin.start()
    host, port = node.param("host"), int(node.param("port"))
    code = 0
    try:
        app = tornado.web.Application(
            [(r"/api", ApiSocket, {"gw": gw})],
            # keepalive only; safety rests on the heartbeat silence, not on this.
            # The timeout must exceed the interval (ui_server.py, 2026-09-29).
            websocket_ping_interval=2, websocket_ping_timeout=6,
            websocket_max_message_size=16384)
        app.listen(port, host)
        # systemd stops with SIGTERM; a background job ignores SIGINT
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, loop.stop)
        loop.create_task(every(gw.settings.state_ms / 1000.0, gw.tick_state, gw.log))
        loop.create_task(every(gw.settings.heartbeat_ms / 1000.0, gw.tick_heartbeat, gw.log))
        gw.log.info(f"{VERSION} on ws://{host}:{port}/api as '{gw.rover_id}'"
                    + (" -- STOP DRY RUN" if node.dry_run else ""))
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    except OSError as e:                   # port taken: say so and exit cleanly
        gw.log.fatal(f"cannot listen on {host}:{port}: {e}")
        code = 1
    finally:
        executor.shutdown(timeout_sec=1.0)
        spin.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
