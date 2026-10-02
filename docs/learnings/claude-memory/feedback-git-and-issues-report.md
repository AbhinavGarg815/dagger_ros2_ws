---
name: feedback-git-and-issues-report
description: Commit as Abhinav (no trailer); keep the running issues report updated same session; reports + learnings live IN the repo (docs/) and GitHub is kept current at each wrap-up
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 4d48c96a-557d-4105-b251-751ed27dee94
  modified: 2026-07-24T08:18:15.564Z
---

Two standing instructions from Abhinav (2026-07-13), both UPDATED 2026-07-24 now that the workspace is solely his:

1. **Maintain git whenever required** — commit working checkpoints proactively as author **Abhinav** (his name/email, **no co-author trailer**), local only (no remote). **UPDATE (2026-07-24): the workspace is now entirely ours — commit ANY package freely.** The old "only my dirs" split and the "colleague commits his own dirs" rule are OBSOLETE (colleague's `dagger_slam` + `dagger_controller` were moved out to `/home/jetson/colleague_pkgs_backup_2026-07-24/`; `dagger_description` kept). Our stack is `dagger_bringup` (+ `msgs_comm` + vendored deps).

2. **Keep a running issues/errors report** — symptom, root cause, fix, status. Kept at **`/home/jetson/dagger_reports/`** (migrated there 2026-07-24 from /home/nvidia when Claude Code moved to the jetson user). The old "outside the shared workspace" rationale is moot now that the workspace is not shared. Update as issues arise; finalize when the project ends.

**UPDATE 2026-09-17:** the report went un-updated 07-13 → 09-17 because Claude ran as `nvidia` and the folder was jetson-owned — never flagged; user called it out. Fixed: folder now group `dagger`, g+rwX, setgid → nvidia can write. Back-filled Part 2 (sections J–N, H12–H18, current config, open items P1–P9) + historical banner and pending-FCU-changes table in `ardupilot_config_checklist.md`. **How to apply:** add an entry (symptom/cause/fix/status/commit) in the SAME session as each fix or finding, and update the 'Current configuration' + 'Open items' blocks; if the folder is ever unwritable, tell the user immediately.

**Why:** Abhinav wants a private running log and clean git history. As of 2026-07-24 the collaboration/cross-user constraints are lifted — see [[dagger-outdoor-ugv-project]] and [[reference-dagger-monitoring-tools]].

**UPDATE 2026-10-02 (user: "include our all reports and learnings in github, and keep it updated"):** the reports now live IN the repo at `/home/jetson/ros2_ws/docs/reports/` (issues_and_fixes.md, ardupilot_config_checklist.md, next_session_prompt.md, tools/, archive/). `/home/jetson/dagger_reports/<file>` and `/tools` are SYMLINKS into it — always EDIT THE REPO PATH (a tool that rewrites a file by rename would replace the symlink and silently fork the copies). Learnings = a copy of this memory folder at `docs/learnings/claude-memory/`, refreshed by `bash tools/sync_learnings.sh`. **How to apply:** at every session wrap-up: sync learnings → commit reports + learnings + code as Abhinav → push to GitHub (user asked for it to be kept current). Pushing at other times: still ask.
