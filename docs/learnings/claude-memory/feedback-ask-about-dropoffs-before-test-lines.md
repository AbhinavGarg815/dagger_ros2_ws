---
name: feedback-ask-about-dropoffs-before-test-lines
description: Never pick a rover test line/heading from the costmap alone -- drop-offs (steps, kerbs down) are invisible to it; ask the operator about the site first
metadata:
  type: feedback
---
2026-10-02: choosing a detour-test line, Claude searched the live costmap for free headings and proposed "turn 60° left" -- toward a step drop-off along the whole left side of the test lane (the "obstacles" there were plants). The user corrected it: "one side is wall and other side is steps … there is drop-off on whole left side of lane".

**Why:** the LiDAR costmap only marks things that stick UP; a drop-off returns nothing and reads as free space (negative-obstacle gap, report P18/Q32). A safety-critical recommendation built on it can point a 350 kg rover at a ledge.

**How to apply:** before proposing any test line, heading, goal or detour room, ask the operator what bounds the site (walls, steps, kerbs, slopes) and treat their description as ground truth; use the costmap only to locate things that stand up. In tests near edges, have tall markers placed along the edge so the costmap sees it. Related: [[project-obstacle-avoidance-next-20261002]], [[feedback-user-drives-physical-motion]].
