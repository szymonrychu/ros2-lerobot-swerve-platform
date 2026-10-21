---
title: mcp_server raw feed subscriptions on a second node
status: done
repo: ros2-lerobot-sverve-platform
value: Cuts mcp_server idle CPU by about 0.3 core
files: [nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/mcp_server/config.py, nodes/mcp_server/mcp_server/__main__.py, nodes/mcp_server/tests/test_feeds_executor.py, nodes/mcp_server/README.md, tests/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [mcp-raw-imu-and-head-states]
owns: []
uses: []
model: ""
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/client-cpu-budget-recovery-a--mcp-server-raw-feed-subscrip
branch: client-cpu-budget-recovery-a--mcp-server-raw-feed-subscrip
verified_head: f78c5b90b338c75a36bd0d4a62170331c5416454
run: client-cpu-budget-recovery-a
provides: [feeds-spinner]
acs_for_audit: []
process_gaps: []
---

# mcp_server raw feed subscriptions on a second node

## Requirement

mcp_server feeds on an EventsExecutor node (CPU second pass, measured 2026-10-21). Evidence (scratchpad/cpu2/threads.txt, rates.txt): mcp_server uses 0.52-0.68 core idle (no tool calls for 5 h); one Python thread (the spin_once / MultiThreadedExecutor dispatch thread) uses 55.7%, the 4 pool workers 1.3% each: the cost is executor overhead per wake (~200 wakes/s: /head/joint_states 47 Hz, /odom 50, /odometry/filtered 26, /imu/data 25, follower throttled 9, rf2o 7, plus 5 Hz orphan check and 4 Hz monitor tick), not callback work.
Fix: move the raw high-rate feed subscriptions (the cache_raw / raw_store subscriptions, nodes/mcp_server/mcp_server/ros_iface.py ~643-720, whose callbacks only lock and store; monitor_tick already flushes them) onto a second rclpy node spun by rclpy.experimental.EventsExecutor on its own thread; the main node keeps its MultiThreadedExecutor (AutoPoiMarker blocks its callback waiting on poi_store, so the main executor must stay multi-threaded). Config switch feeds_executor: events|main (default events) in the mcp_server config (code default, client.yml untouched: it is owned by another in-progress epic), so it can be rolled back by config. Thread-safety: same locks as today; startup/shutdown destroy the feeds node on its thread (ROS-thread affinity rule from web_ui). Expected -0.3..0.4 core.
Tests first in nodes/mcp_server/tests: the feeds node owns exactly the raw subscriptions, the main node no longer has them, monitor_tick still sees fresh samples, the config switch main restores the old wiring, clean shutdown. Update nodes/mcp_server/README.md and tests/README.md if it lists mcp tests.

## Acceptance conditions

- a1: with feeds_executor events the raw feed subscriptions live on a separate node spun by an EventsExecutor thread [evidence: test:nodes/mcp_server/tests/test_feeds_executor.py::test_raw_feeds_on_separate_events_node]
- a2: monitor_tick and the tools still read fresh IMU, head joint states, odometry and follower feedback [evidence: test:nodes/mcp_server/tests/test_feeds_executor.py::test_monitor_tick_sees_fresh_feed_samples]
- a3: feeds_executor main restores the single-node wiring [evidence: test:nodes/mcp_server/tests/test_feeds_executor.py::test_main_mode_restores_single_node]
- a4: mcp_server tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/__main__.py`
- `nodes/mcp_server/tests/test_feeds_executor.py`
- `nodes/mcp_server/README.md`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

mcp_server idle CPU drops by about 0.3 core with the same tool behaviour; feeds_executor: main rolls it back by config.

What is true and observable when the task is done.

## Steps

1. Write the failing tests in nodes/mcp_server/tests/test_feeds_executor.py.
2. Add the feeds node and its EventsExecutor thread in ros_iface.py / __main__.py, the feeds_executor config field in config.py.
3. Move the raw feed subscriptions; keep locks and monitor_tick flushing.
4. Run the verify command, update the READMEs, commit.

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
