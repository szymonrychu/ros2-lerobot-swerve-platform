---
title: Filter node events executor
status: done
repo: ros2-lerobot-sverve-platform
value: "filter_node's 100 Hz loop and feedback callbacks run on the cheaper EventsExecutor, switchable back by config."
files: [nodes/filter_node/filter_node/config.py, nodes/filter_node/filter_node/node.py, nodes/filter_node/tests/test_executor_select.py, nodes/filter_node/tests/ros_stubs.py, nodes/filter_node/README.md]
verify: "cd nodes/filter_node && uv run --frozen pytest tests -q && uv run poe lint"
depends_on: []
owns: []
uses: []
model: ""
estimate: 45
actual: ""
review: full
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/client-cpu-budget-recovery-a--filter-node-events-executor
branch: client-cpu-budget-recovery-a--filter-node-events-executor
verified_head: 2d6520350407698e99c2ca797363760483818a5b
run: client-cpu-budget-recovery-a
provides: []
acs_for_audit: []
process_gaps: ["The implementer said the rclpy source was unavailable; the jazzy source is fetchable. Result: EventsExecutor.spin_once calls spin(timeout_sec, stop_after_user_callback=true), which runs events_queue_.Run(end) with end = now + timeout. So it honours the timeout. Like SingleThreadedExecutor.spin_once, it can return earlier after one user callback, so loop pacing is unchanged. This belongs in the task's evidence record.", "TDD red evidence: I ran the new test file against the merge-base node.py and config.py (copied into the scratchpad, worktree untouched). Result: 4 failed, 6 errors. With HEAD: 74 passed, lint clean. The implementer should paste these into tests.red and verification.evidence."]
---

# Filter node events executor

## Requirement

Item 2 of the brief for filter_node: config key in `filter_node/config.py` (FilterConfig dataclass and its loader), `make_executor` in `filter_node/node.py`; the main loop keeps calling `executor.spin_once(timeout_sec=control_period_s)` on the configured executor (node.py:226). Confirm from the rclpy jazzy source (rclpy/src/rclpy/events_executor/, the 7.1.x tag) that EventsExecutor.spin_once honours the timeout so the 100 Hz control loop keeps its period, and pin that with a test on the loop using a stub executor.

Shared rules for the five events-executor tasks (design D2; approval A2):

- New config key `executor`, values `events` | `single_threaded`, default `single_threaded` in code (so the server, tests and any unset config keep today's behaviour); an unknown value is a config error at load time, like every other invalid key of that node.
- New public function `make_executor(kind: str) -> rclpy.executors.Executor` in the node's module (no leading underscore, typed, docstring with Args/Returns): `events` returns `rclpy.experimental.EventsExecutor()`, `single_threaded` returns `rclpy.executors.SingleThreadedExecutor()`. The import of `rclpy.experimental` sits at the top of the module (no try/except).
- The run function adds the node to `make_executor(config.executor)` and spins that executor (no `rclpy.spin(node)` on the global executor); shutdown handling (KeyboardInterrupt, ExternalShutdownException, RCLError) and `destroy_node`/`try_shutdown` stay exactly as they are.
- EventsExecutor ignores callback groups: confirm the node creates no callback group and no MultiThreadedExecutor semantics, and say so in the README.
- The test stubs (`tests/ros_stubs.py` or the test's own monkeypatch) stub `rclpy.experimental.EventsExecutor`; no test needs a real ROS install.
- Node README: the `executor` key in the config table, with the rollback note (set `single_threaded` in `group_vars/client.yml` and redeploy the node).

## Acceptance conditions

- a1: the executor key defaults to single_threaded [evidence: test:nodes/filter_node/tests/test_executor_select.py::test_executor_defaults_to_single_threaded]
- a2: an unknown executor value is rejected at config load [evidence: test:nodes/filter_node/tests/test_executor_select.py::test_unknown_executor_is_rejected]
- a3: make_executor returns an EventsExecutor for events and a SingleThreadedExecutor for single_threaded [evidence: test:nodes/filter_node/tests/test_executor_select.py::test_make_executor_builds_the_configured_kind]
- a4: the run function spins the configured executor with the node added [evidence: test:nodes/filter_node/tests/test_executor_select.py::test_run_spins_the_configured_executor]
- a5: the README documents the executor key and its rollback [evidence: judgement:]
- a6: the node's tests and lint pass [evidence: command:cd nodes/filter_node && uv run --frozen pytest tests -q && uv run poe lint]
- a7: the control loop calls spin_once with the control period on the configured executor [evidence: test:nodes/filter_node/tests/test_executor_select.py::test_control_loop_spins_once_per_period]

## Files

- `nodes/filter_node/filter_node/config.py`
- `nodes/filter_node/filter_node/node.py`
- `nodes/filter_node/tests/test_executor_select.py`
- `nodes/filter_node/tests/ros_stubs.py`
- `nodes/filter_node/README.md`

## Interfaces

New config key `executor: events | single_threaded` (default `single_threaded`) and public `make_executor(kind: str) -> rclpy.executors.Executor` in the node module named in the requirement. Consumed by task client-executor-config, which sets `executor: events` in `ansible/group_vars/client.yml`. Keep every topic, QoS, timer and shutdown path unchanged.

## Expected outcome

`cd nodes/filter_node && uv run --frozen pytest tests -q` passes, including `tests/test_executor_select.py`; with no `executor` key the node behaves exactly as today.

## Steps

- Write nodes/filter_node/tests/test_executor_select.py (stub `rclpy.experimental.EventsExecutor` and `SingleThreadedExecutor`); `cd nodes/filter_node && uv run --frozen pytest tests/test_executor_select.py -q` FAILS (no key, no make_executor).
- Add the config key with validation, `make_executor`, and spin the configured executor; rerun, PASS; run the whole package suite.
- `uv run poe lint`; update the README config table and rollback note.
- Commit `perf(filter_node): configurable EventsExecutor`.

## Notes

Design D2: rclpy.experimental.EventsExecutor (installed rclpy 7.1.12 on the client) runs the same callbacks with far less Python executor overhead; it is experimental and ignores callback groups, so it is used only in single-threaded nodes with one default callback group, and every node keeps a config rollback. The helper lives in each node (no shared package: laser_filter has no shared/ dependency, and shared/ros2_common would pull opencv and scipy into every venv). Expected saving across the five nodes about 0.3 core (estimate; the runbook measures it). Deployment is task client-executor-config, not this task.

Task filter-node-idle-feedback changes the same node.py and config.py afterwards (it depends on this task). servo-torque-off-and-neutral last changed filter_node (filter-node-released-arm, done); keep its torque-state handling intact.
