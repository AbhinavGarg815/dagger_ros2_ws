#!/usr/bin/env bash
# Generate the gateway's Python classes from the app <-> rover contract (proto/, a
# git submodule of dagger_proto). Run by dagger_gateway's setup.py on every colcon
# build; the output is git-ignored. Never edit it by hand.
#
# protoc is apt's protobuf-compiler 3.12.4, matching the runtime (python3-protobuf
# 3.12.4). 3.12 needs --experimental_allow_proto3_optional for the schema's
# `optional` fields; checked byte for byte against proto/vectors (test_vectors.py).
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$WS/proto"
OUT="${1:-$WS/src/dagger_gateway/dagger_gateway/api_pb2.py}"
[ -f "$SRC/dagger/api.proto" ] || { echo "no $SRC/dagger/api.proto: git submodule update --init proto" >&2; exit 1; }
command -v protoc >/dev/null || { echo "no protoc: sudo apt install protobuf-compiler" >&2; exit 1; }
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
protoc --experimental_allow_proto3_optional -I"$SRC" --python_out="$TMP" "$SRC/dagger/api.proto"
mv "$TMP/dagger/api_pb2.py" "$OUT"
echo "generated $OUT from $(git -C "$SRC" rev-parse --short HEAD 2>/dev/null || echo proto/)"
