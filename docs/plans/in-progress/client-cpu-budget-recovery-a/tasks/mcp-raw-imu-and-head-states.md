---
title: MCP raw IMU and head states
status: done
repo: ros2-lerobot-sverve-platform
value: "mcp_server's dispatch thread stops decoding ~70 messages/s it reads at 4 Hz (about 0.08 core) with bump detection unchanged."
files: [nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/mcp_server/staleness.py, nodes/mcp_server/tests/test_raw_monitor_feeds.py, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_raw_monitor_feeds.py tests/test_monitor.py tests/test_head_tools.py tests/test_head_controller.py tests/test_ros_iface_state_cache.py -q && uv run poe lint"
depends_on: []
owns: []
uses: []
model: ""
estimate: 75
actual: ""
review: full
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/client-cpu-budget-recovery-a--mcp-raw-imu-and-head-states
branch: worktree branch of client-cpu-budget-recovery-a--mcp-raw-imu-and-head-states
verified_head: 950f300941505e9e6ae149a3bdaf7468582194c8
run: client-cpu-budget-recovery-a
provides: []
acs_for_audit: []
process_gaps: []
---

# MCP raw IMU and head states

## Requirement

Item 6 of the brief (design D6; approval A6): mcp_server takes `/imu/data` and `/head/joint_states` as raw (serialized) subscriptions and deserializes them only on the 0.25 s monitor tick or on read, reusing the existing raw machinery (`RosInterface.cache_raw`, `raw_store`, `LazyStamped`, `MonitorFeed` in `mcp_server/staleness.py`).

- `/head/joint_states` (config.head.state_topic, ~47 Hz): `self.cache(...)` at ros_iface.py:663 becomes `self.cache_raw(JointState, config.head.state_topic, "head_joint_states", SENSOR_QOS)`; readers (`get("head_joint_states")`, ros_iface.py:1564 and every other reader) get the decoded message lazily, at most one decode per message.
- `/imu/data` (now 25 Hz after imu-publish-rate-25-hz): the `self.subscribe(Imu, t.imu, ...)` at ros_iface.py:708-719 becomes a raw subscription whose monitor feed keeps every serialized sample received since the previous tick in arrival order (bounded queue, `IMU_FEED_MAX_SAMPLES` constant, oldest dropped and counted in a log line when full) and, on the monitor tick, decodes each queued sample and calls `monitor.on_imu` for it in order. Bump detection (`bump_min_samples` consecutive samples) and the tilt fallback therefore see exactly the samples they see today (design D6, open question Q1). The queueing feed is a new public class next to `MonitorFeed` in `mcp_server/staleness.py`, or a `keep_all` option of `MonitorFeed`; `cache_raw` gains the matching parameter.
- The raw callbacks only store bytes under `self.lock` or the feed's own lock; no decode happens on the executor's dispatch path for these two topics.
- Tests first (red seen) in `tests/test_raw_monitor_feeds.py`: `test_head_joint_states_are_decoded_only_when_read`, `test_imu_samples_are_decoded_on_the_monitor_tick`, `test_imu_tick_replays_every_queued_sample_in_order` (two consecutive spike samples between ticks still emit one bump), `test_imu_queue_is_bounded`. Existing head tool and monitor tests (test_head_tools.py, test_head_controller.py, test_monitor.py, test_ros_iface_state_cache.py) keep passing.
- `nodes/mcp_server/README.md` lists both topics as raw and explains the tick replay.
- Not changed: web_ui, the MultiThreadedExecutor, any other subscription.

## Acceptance conditions

- a1: head joint states are stored raw and decoded only when read [evidence: test:nodes/mcp_server/tests/test_raw_monitor_feeds.py::test_head_joint_states_are_decoded_only_when_read]
- a2: IMU samples are decoded on the monitor tick, not in the subscription callback [evidence: test:nodes/mcp_server/tests/test_raw_monitor_feeds.py::test_imu_samples_are_decoded_on_the_monitor_tick]
- a3: every IMU sample since the previous tick reaches monitor.on_imu in order, so bump detection is unchanged [evidence: test:nodes/mcp_server/tests/test_raw_monitor_feeds.py::test_imu_tick_replays_every_queued_sample_in_order]
- a4: the IMU queue is bounded [evidence: test:nodes/mcp_server/tests/test_raw_monitor_feeds.py::test_imu_queue_is_bounded]
- a5: the raw-feed, monitor, head and state-cache tests and lint pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_raw_monitor_feeds.py tests/test_monitor.py tests/test_head_tools.py tests/test_head_controller.py tests/test_ros_iface_state_cache.py -q && uv run poe lint]

## Files

- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/staleness.py`
- `nodes/mcp_server/tests/test_raw_monitor_feeds.py`
- `nodes/mcp_server/README.md`

## Interfaces

Reuse `RosInterface.cache_raw` / `raw_store` (nodes/mcp_server/mcp_server/ros_iface.py:1130-1180), `LazyStamped` and `MonitorFeed` (nodes/mcp_server/mcp_server/staleness.py:118); `monitor_tick` (ros_iface.py:1265) already flushes every feed before `monitor.tick()`. New: a keep-all feed (class or `MonitorFeed` option) and the matching `cache_raw` parameter. Keep `Monitor.on_imu(ax, ay, az, qx, qy, qz, qw)`, the `get("head_joint_states")` return shape, `MONITOR_TICK_S` (0.25 s) and the 4-thread MultiThreadedExecutor.

## Expected outcome

No `/imu/data` or `/head/joint_states` message is deserialized on the executor's dispatch path; head reads and bump/tilt events behave as before; the listed tests pass.

## Steps

- Write tests/test_raw_monitor_feeds.py against the existing fakes (tests/fakes.py, conftest.py); `cd nodes/mcp_server && uv run --frozen pytest tests/test_raw_monitor_feeds.py -q` FAILS.
- Add the keep-all feed, switch the two subscriptions; rerun, PASS; run test_monitor.py, test_head_tools.py, test_head_controller.py, test_ros_iface_state_cache.py.
- `uv run poe lint`; update the README.
- Commit `perf(mcp_server): raw IMU and head joint state subscriptions`.

## Notes

Evidence (brief item 6): the MultiThreadedExecutor's dispatch thread holds 95% of mcp_server's CPU, IMU ~40 Hz (25 Hz after imu-publish-rate-25-hz) and /head/joint_states 47 Hz; expected -0.08 core. Design D6 and open question Q1: bump detection needs `bump_min_samples` (2) consecutive samples, so the IMU feed replays every queued sample at the tick instead of only the newest; if the user answers Q1 no, the orchestrator amends this task to the latest-only feed. web_ui and mcp_server stay Python and keep their executors (user decision). Deploying mcp_server kills a running claude_agent session: the runbook's agent idle guard covers it.
