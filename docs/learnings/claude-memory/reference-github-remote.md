---
name: reference-github-remote
description: "ros2_ws is on GitHub (private) — origin git@github.com:AbhinavGarg815/dagger_ros2_ws.git, SSH key ~/.ssh/id_ed25519_github"
metadata:
  node_type: memory
  type: reference
  originSessionId: 5c15c34f-dbac-4cc2-816f-8b5cfc96d79b
  modified: 2026-09-29T07:35:19.211Z
---

Since 2026-09-29 `/home/jetson/ros2_ws` has `origin` = `git@github.com:AbhinavGarg815/dagger_ros2_ws.git` (PRIVATE, user's choice), `main` tracks `origin/main`.

- Auth: SSH key `~/.ssh/id_ed25519_github` (user nvidia, no passphrase), `~/.ssh/config` Host github.com → that key; GitHub ED25519 host key verified SHA256:+DiY3wvvV6TuJJhbpZisF/zLDA0zPMSvHdkr4UvCOqU. `gh` CLI is NOT installed.
- GitHub had auto-created a README commit; it was MERGED (not force-pushed) → merge commit 95542bb.
- In the repo since 2026-10-02: reports (`docs/reports/`, old dagger_reports paths symlink there) and learnings (`docs/learnings/claude-memory/`, synced copy of Claude's memory). Not in the repo: bags, models, vendored third-party src (git-ignored).
- Repo contains site lat/lon (maps/*_geo.yaml, commit messages) and the author Gmail — reasons it is private; don't make it public without rewriting that.
- Pushing policy: user has not said to auto-push — ask (see [[feedback-git-and-issues-report]]).
- 2026-09-30 wrap-up: user said "update our github" → pushed 15 commits, origin/main = 652cd4e (includes COMMANDS.md, ArUco following, tools/fixtures/aruco_narrow_margin.npz = 12 grayscale crops of the marker held by the user).
- 2026-10-02 obstacle-avoidance wrap-up: pushed the near-field work (safety_scan / dagger_perception, Q65), reports, learnings and tomorrow's hand-off prompt — origin/main = the "wrap-up 2026-10-02" commit (see `git log`).
