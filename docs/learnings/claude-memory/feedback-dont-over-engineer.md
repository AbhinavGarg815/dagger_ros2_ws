---
name: feedback-dont-over-engineer
description: User said "don't over engineer the system" — fix real, measured problems with the simplest change; don't add layers for minor effects
metadata:
  type: feedback
---

User (2026-10-05, mid-turn while I was stacking variants on a minor LiDAR twist-spike issue): "dont over engineer the system".

**Why:** the spikes barely mattered live (EKF already smoothed them), my first fix (acceleration gate) made accuracy worse, and I was about to add a second mechanism. Each extra layer is more to maintain and test on a robot that already has many moving parts.

**How to apply:** before building, check the live impact with data; if it's small, record it and move on (or offer it as optional). Prefer the simplest change that fixes a measured problem; when a fix doesn't clearly win, revert rather than add another layer. Present "do nothing" as a real option. Related: [[user-abhinav-robotics-engineer]] (plan first, one recommendation).
