#!/usr/bin/env python3
"""How old is the follow page's video when it leaves the rover? READ-ONLY.

    python3 tools/ui_latency.py               # 10 s, full video, on the Jetson itself
    python3 tools/ui_latency.py --lite -s 20  # the half-size (LITE) stream

Connects to ui_server like a page (needs T1 with use_camera:=true and T4) and,
per frame, splits the delay on the Jetson's own clock:
    camera -> sent   capture stamp to ui_server sending the frame (video first:
                     camera driver + JPEG, not detection)
    sent -> here     the network (about 0 on the Jetson itself)
    boxes            capture stamp to that frame's boxes arriving (detection +
                     tracking); the page shows the previous boxes until then
What the handset shows ('age' on the page) adds the radio and the browser's
decode + draw. Never sends heartbeats or commands: it cannot keep follow alive
or start anything.
"""
import argparse
import asyncio
import json
import struct
import time

import numpy as np
import tornado.websocket


async def run(a):
    url = f"ws://{a.host}:{a.port}/ws" + ("?video=lite" if a.lite else "")
    ws = await tornado.websocket.websocket_connect(url, max_message_size=2 ** 22)
    rows, boxes, end = [], [], time.time() + a.secs
    while time.time() < end:
        try:
            m = await asyncio.wait_for(ws.read_message(), timeout=max(0.1, end - time.time()))
        except asyncio.TimeoutError:
            break
        if m is None:
            raise SystemExit("connection closed by ui_server")
        now = time.time()
        if not isinstance(m, bytes):
            b = json.loads(m)
            if b.get("type") == "boxes":
                boxes.append(now - b["stamp"])
            continue
        n = struct.unpack(">I", m[:4])[0]
        h = json.loads(m[4:4 + n])
        rows.append((now, h["stamp"], h["sent"], len(m) - 4 - n, h.get("tracks") is not None))
    ws.close()
    return rows, boxes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("-s", "--secs", type=float, default=10.0)
    ap.add_argument("--lite", action="store_true", help="the half-size LITE stream")
    a = ap.parse_args()
    rows, boxes = asyncio.get_event_loop().run_until_complete(run(a))
    if len(rows) < 2:
        raise SystemExit(f"{len(rows)} frames in {a.secs:.0f} s -- is T4 (follow.launch.py) up, "
                         f"and T1 with use_camera:=true?")
    r = np.array([(n - s, e - s, n - e, b) for n, s, e, b, _ in rows])
    fps = (len(rows) - 1) / (rows[-1][0] - rows[0][0])
    gaps = np.diff([x[0] for x in rows])
    q = lambda v: f"median {1e3 * np.median(v):5.1f} ms, p95 {1e3 * np.percentile(v, 95):5.1f}"  # noqa: E731
    print(f"{len(rows)} frames, {fps:.1f} fps, longest gap {1e3 * gaps.max():.0f} ms, "
          f"{'LITE' if a.lite else 'full'} video {np.median(r[:, 3]) / 1024:.0f} KB/frame "
          f"({np.median(r[:, 3]) * 8 * fps / 1e6:.1f} Mb/s), {len(boxes)} boxes messages")
    print(f"  camera -> sent : {q(r[:, 1])}")
    print(f"  sent   -> here : {q(r[:, 2])}")
    print(f"  camera -> here : {q(r[:, 0])}")
    if boxes:
        print(f"  camera -> boxes: {q(np.array(boxes))}")


if __name__ == "__main__":
    main()
