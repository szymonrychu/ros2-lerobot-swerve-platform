---
title: Marker events executor
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "marker_detector's callbacks run on the cheaper EventsExecutor, switchable back by config."
files: [nodes/marker_detector/marker_detector/config.py, nodes/marker_detector/marker_detector/node.py, nodes/marker_detector/tests/test_executor_select.py, nodes/marker_detector/tests/ros_stubs.py, nodes/marker_detector/README.md]
verify: "cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run poe lint"
depends_on: []
owns: []
uses: []
model: ""
estimate: 40
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/client-cpu-budget-recovery-a--marker-events-executor
branch: client-cpu-budget-recovery-a--marker-events-executor
verified_head: ""
run: ""
provides: []
---

# Marker events executor

## Requirement

Item 2 of the brief for marker_detector: config key in `marker_detector/config.py` (MarkerDetectorConfig, strict pydantic), `make_executor` and the spin in `marker_detector/node.py` run_node() (today `rclpy.spin(node)` at node.py:178). The tf2 TransformListener keeps its own spin thread (spin_thread=True) and is not affected.

Shared rules for the five events-executor tasks (design D2; approval A2):

- New config key `executor`, values `events` | `single_threaded`, default `single_threaded` in code (so the server, tests and any unset config keep today's behaviour); an unknown value is a config error at load time, like every other invalid key of that node.
- New public function `make_executor(kind: str) -> rclpy.executors.Executor` in the node's module (no leading underscore, typed, docstring with Args/Returns): `events` returns `rclpy.experimental.EventsExecutor()`, `single_threaded` returns `rclpy.executors.SingleThreadedExecutor()`. The import of `rclpy.experimental` sits at the top of the module (no try/except).
- The run function adds the node to `make_executor(config.executor)` and spins that executor (no `rclpy.spin(node)` on the global executor); shutdown handling (KeyboardInterrupt, ExternalShutdownException, RCLError) and `destroy_node`/`try_shutdown` stay exactly as they are.
- EventsExecutor ignores callback groups: confirm the node creates no callback group and no MultiThreadedExecutor semantics, and say so in the README.
- The test stubs (`tests/ros_stubs.py` or the test's own monkeypatch) stub `rclpy.experimental.EventsExecutor`; no test needs a real ROS install.
- Node README: the `executor` key in the config table, with the rollback note (set `single_threaded` in `group_vars/client.yml` and redeploy the node).

## Acceptance conditions

- a1: the executor key defaults to single_threaded [evidence: test:nodes/marker_detector/tests/test_executor_select.py::test_executor_defaults_to_single_threaded]
- a2: an unknown executor value is rejected at config load [evidence: test:nodes/marker_detector/tests/test_executor_select.py::test_unknown_executor_is_rejected]
- a3: make_executor returns an EventsExecutor for events and a SingleThreadedExecutor for single_threaded [evidence: test:nodes/marker_detector/tests/test_executor_select.py::test_make_executor_builds_the_configured_kind]
- a4: the run function spins the configured executor with the node added [evidence: test:nodes/marker_detector/tests/test_executor_select.py::test_run_spins_the_configured_executor]
- a5: the README documents the executor key and its rollback [evidence: judgement:]
- a6: the node's tests and lint pass [evidence: command:cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/marker_detector/marker_detector/config.py`
- `nodes/marker_detector/marker_detector/node.py`
- `nodes/marker_detector/tests/test_executor_select.py`
- `nodes/marker_detector/tests/ros_stubs.py`
- `nodes/marker_detector/README.md`

## Interfaces

New config key `executor: events | single_threaded` (default `single_threaded`) and public `make_executor(kind: str) -> rclpy.executors.Executor` in the node module named in the requirement. Consumed by task client-executor-config, which sets `executor: events` in `ansible/group_vars/client.yml`. Keep every topic, QoS, timer and shutdown path unchanged.

## Expected outcome

`cd nodes/marker_detector && uv run --frozen pytest tests -q` passes, including `tests/test_executor_select.py`; with no `executor` key the node behaves exactly as today.

## Steps

- Write nodes/marker_detector/tests/test_executor_select.py (stub `rclpy.experimental.EventsExecutor` and `SingleThreadedExecutor`); `cd nodes/marker_detector && uv run --frozen pytest tests/test_executor_select.py -q` FAILS (no key, no make_executor).
- Add the config key with validation, `make_executor`, and spin the configured executor; rerun, PASS; run the whole package suite.
- `uv run poe lint`; update the README config table and rollback note.
- Commit `perf(marker_detector): configurable EventsExecutor`.

## Notes

Design D2: rclpy.experimental.EventsExecutor (installed rclpy 7.1.12 on the client) runs the same callbacks with far less Python executor overhead; it is experimental and ignores callback groups, so it is used only in single-threaded nodes with one default callback group, and every node keeps a config rollback. The helper lives in each node (no shared package: laser_filter has no shared/ dependency, and shared/ros2_common would pull opencv and scipy into every venv). Expected saving across the five nodes about 0.3 core (estimate; the runbook measures it). Deployment is task client-executor-config, not this task.

Task marker-detector-raw-image changes the same node.py afterwards (it depends on this task).
