#!/usr/bin/env python3
"""Byte half of the contract, from the gateway's own generated code. NO ROS.

    python3 src/dagger_gateway/test/test_vectors.py     (or pytest)

Every proto/vectors/NAME.bin must decode to exactly what NAME.txtpb says, and
encode back (deterministic: map keys sorted) to exactly the same bytes. The app
checks the same files from Kotlin; proto/make_vectors.py --check is the reference.
Run tools/gen_proto.sh (or colcon build) first.
"""
import glob
import os
import sys

from google.protobuf import text_format

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from dagger_gateway import api_pb2 as pb  # noqa: E402

VECTORS = os.path.join(HERE, "..", "..", "..", "proto", "vectors")


def vectors():
    found = sorted(glob.glob(os.path.join(VECTORS, "*.bin")))
    assert found, f"no vectors in {VECTORS}: git submodule update --init proto"
    return found


def test_every_vector_round_trips():
    bad = []
    for path in vectors():
        raw = open(path, "rb").read()
        got = pb.Envelope()
        got.ParseFromString(raw)
        with open(path[:-4] + ".txtpb", encoding="utf-8") as f:
            want = text_format.Parse(f.read(), pb.Envelope())
        if got != want:
            bad.append(f"{os.path.basename(path)}: decodes to something else")
        elif got.SerializeToString(deterministic=True) != raw:
            bad.append(f"{os.path.basename(path)}: re-encodes to other bytes")
    assert not bad, "\n".join(bad)


def test_optional_presence():
    """proto3 `optional` (experimental in 3.12): 0 is a reading, absent is not."""
    env = pb.Envelope()
    env.ParseFromString(open(os.path.join(VECTORS, "state_rtt_zero.bin"), "rb").read())
    assert env.state.link.HasField("round_trip_ms") and env.state.link.round_trip_ms == 0
    assert not pb.Link().HasField("round_trip_ms")
    env.ParseFromString(open(os.path.join(VECTORS, "state_no_fix.bin"), "rb").read())
    assert not env.state.HasField("position")


if __name__ == "__main__":
    test_every_vector_round_trips()
    test_optional_presence()
    print(f"ok: {len(vectors())} vectors decode, match their .txtpb and re-encode to the same bytes; "
          "optional presence holds")
