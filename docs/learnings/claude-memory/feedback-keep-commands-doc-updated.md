---
name: feedback-keep-commands-doc-updated
description: Keep /home/jetson/ros2_ws/COMMANDS.md (the project's command reference) updated in the same session as any new or changed command
metadata:
  type: feedback
---

User 2026-09-30: "make a document containing all the commands used in this project from very basic to last command, and their use … proper standard way … easy to run tasks, debugging … Don't clutter document with unnecessary details, duplicate commands, be precise. and regularly update that document as we continue our project."

The document is `/home/jetson/ros2_ws/COMMANDS.md` (in the repo, linked from README.md; sections: setup, build, bring-up, stop, navigation, SLAM, following, recording, analysis, debugging, offline tests, FCU params, git, quick reference).

**Why:** the user wants one place to run tasks and debug from, without reading the 1000-line issues report.

**How to apply:** whenever a session adds or changes a command — a new tool, launch argument, test, expected test count, topic, port, or the marker/launch line the user runs — edit COMMANDS.md in the SAME session and commit it with the change. Keep it terse: one line per command, tables, no history or rationale (that belongs in the issues report, see [[feedback-git-and-issues-report]]), no duplicates. Only commands that exist and were verified. Update its "Last updated" date.
