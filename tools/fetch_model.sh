#!/usr/bin/env bash
# Fetch the follow-me detector's ONNX and build a TensorRT engine for THIS machine.
#
#     bash tools/fetch_model.sh            # yolox_nano (default)
#     bash tools/fetch_model.sh yolox_s    # bigger, better at range
#
# models/ is gitignored: the ONNX is multi-MB and a TensorRT engine is tied to
# this exact GPU + TensorRT version, so it is rebuilt rather than committed.
#
# Why a pre-exported ONNX instead of exporting one: torch/ultralytics are not
# installed on this Jetson (deliberately -- torch on JetPack is painful), so
# there is no way to export locally. These are the official Megvii YOLOX release
# assets. COCO classes; class 0 is "person".
set -euo pipefail

MODEL="${1:-yolox_nano}"
DIR="$(cd "$(dirname "$0")/.." && pwd)/models"
URL="https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/${MODEL}.onnx"
TRTEXEC=/usr/src/tensorrt/bin/trtexec

mkdir -p "$DIR"
if [ ! -s "$DIR/${MODEL}.onnx" ]; then
    echo "fetching ${MODEL}.onnx ..."
    curl -fL -o "$DIR/${MODEL}.onnx" "$URL"
fi
sha256sum "$DIR/${MODEL}.onnx"

echo "building FP16 engine (several minutes on the Orin) ..."
"$TRTEXEC" --onnx="$DIR/${MODEL}.onnx" \
           --saveEngine="$DIR/${MODEL}_fp16.engine" \
           --fp16 2>&1 | tee "$DIR/trtexec_${MODEL}.log" | tail -20

echo
echo "engine: $DIR/${MODEL}_fp16.engine"
echo "NOTE: the throughput printed above is for THIS nvpmodel power mode"
echo "      (currently: $(nvpmodel -q 2>/dev/null | grep -i 'power mode' || echo unknown))."
