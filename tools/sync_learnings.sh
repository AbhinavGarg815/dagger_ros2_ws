#!/usr/bin/env bash
# Copy Claude Code's project memory -- the running notes of decisions, lessons and
# hand-offs -- into docs/learnings/claude-memory/ so it is versioned with the code.
#   bash tools/sync_learnings.sh
# Run before committing; Claude does it at each session wrap-up. The memory itself
# stays where Claude Code keeps it; this is a copy (changes made here are overwritten).
set -euo pipefail
src=${CLAUDE_MEMORY_DIR:-/home/nvidia/.claude/projects/-home-jetson-ros2-ws/memory}
dst="$(cd "$(dirname "$0")/.." && pwd)/docs/learnings/claude-memory"
[ -d "$src" ] || { echo "no memory folder at $src (set CLAUDE_MEMORY_DIR)"; exit 1; }
mkdir -p "$dst"
rsync -a --delete --include='*.md' --exclude='*' "$src/" "$dst/"
echo "synced $(ls "$dst" | wc -l) files -> $dst"
