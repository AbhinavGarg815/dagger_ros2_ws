---
name: feedback-no-blocking-waits-during-tests
description: Don't run long blocking wait/poll loops while the user is running a rover test -- wait for them to report "done"
metadata:
  type: feedback
---
When the user starts a timed rover test (e.g. `tools/follow_solo.sh ... HH:MM:SS`), do NOT launch a long blocking tool call that polls for the bag/recorder to finish. Acknowledge briefly and wait for the user to say it's done, then analyse the bag.

**Why:** the user rejected such wait loops twice on 2026-09-26 (13:34 and 14:40 stop tests) -- it ties up the session while they are at the rover and can't interact.

**How to apply:** after giving test instructions, end the turn. When they report "done"/paste output, then read the bag. Related: [[feedback-user-drives-physical-motion]].
