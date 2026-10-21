---
title: Feetech events executor
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Both client servo bridges (arm and base) drain callbacks on the cheaper EventsExecutor, switchable back by config."
files: [nodes/bridges/feetech_servos/config.py, nodes/bridges/feetech_servos/bridge.py, nodes/bridges/feetech_servos/tests/test_executor_select.py, nodes/bridges/feetech_servos/README.md]
verify: "cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run poe lint"
depends_on: []
owns: []
uses: []
model: ""
estimate: 60
actual: ""
review: full
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/client-cpu-budget-recovery-a--feetech-events-executor
branch: client-cpu-budget-recovery-a--feetech-events-executor
verified_head: ""
run: ""
provides: []
---

# Feetech events executor

## Requirement

Item 2 of the brief for the feetech_servos bridge (lerobot_follower and base_servos on the client; the server's leader bridge keeps the default): config key in `config.py` (BridgeConfig and its loader), `make_executor` in `bridge.py`; the cycle loop keeps `executor.spin_once(timeout_sec=0.0)` inside `spin_ready`/`drain_callbacks` (bridge.py:391). Existing tests monkeypatch `module.SingleThreadedExecutor` (test_instance_identity, test_shutdown, test_serial_wait, test_startup_goal, test_torque_loop, test_joint_currents): `make_executor` reads the module globals at call time so they keep working unchanged. Servo safety: no servo motion in this task.

Shared rules for the five events-executor tasks (design D2; approval A2):

- New config key `executor`, values `events` | `single_threaded`, default `single_threaded` in code (so the server, tests and any unset config keep today's behaviour); an unknown value is a config error at load time, like every other invalid key of that node.
- New public function `make_executor(kind: str) -> rclpy.executors.Executor` in the node's module (no leading underscore, typed, docstring with Args/Returns): `events` returns `rclpy.experimental.EventsExecutor()`, `single_threaded` returns `rclpy.executors.SingleThreadedExecutor()`. The import of `rclpy.experimental` sits at the top of the module (no try/except).
- The run function adds the node to `make_executor(config.executor)` and spins that executor (no `rclpy.spin(node)` on the global executor); shutdown handling (KeyboardInterrupt, ExternalShutdownException, RCLError) and `destroy_node`/`try_shutdown` stay exactly as they are.
- EventsExecutor ignores callback groups: confirm the node creates no callback group and no MultiThreadedExecutor semantics, and say so in the README.
- The test stubs (`tests/ros_stubs.py` or the test's own monkeypatch) stub `rclpy.experimental.EventsExecutor`; no test needs a real ROS install.
- Node README: the `executor` key in the config table, with the rollback note (set `single_threaded` in `group_vars/client.yml` and redeploy the node).

## Acceptance conditions

- a1: the executor key defaults to single_threaded [evidence: test:nodes/bridges/feetech_servos/tests/test_executor_select.py::test_executor_defaults_to_single_threaded]
- a2: an unknown executor value is rejected at config load [evidence: test:nodes/bridges/feetech_servos/tests/test_executor_select.py::test_unknown_executor_is_rejected]
- a3: make_executor returns an EventsExecutor for events and a SingleThreadedExecutor for single_threaded [evidence: test:nodes/bridges/feetech_servos/tests/test_executor_select.py::test_make_executor_builds_the_configured_kind]
- a4: the run function spins the configured executor with the node added [evidence: test:nodes/bridges/feetech_servos/tests/test_executor_select.py::test_run_spins_the_configured_executor]
- a5: the README documents the executor key and its rollback [evidence: judgement:]
- a6: the node's tests and lint pass [evidence: command:cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/bridges/feetech_servos/config.py`
- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/tests/test_executor_select.py`
- `nodes/bridges/feetech_servos/README.md`

## Interfaces

New config key `executor: events | single_threaded` (default `single_threaded`) and public `make_executor(kind: str) -> rclpy.executors.Executor` in the node module named in the requirement. Consumed by task client-executor-config, which sets `executor: events` in `ansible/group_vars/client.yml`. Keep every topic, QoS, timer and shutdown path unchanged.

## Expected outcome

`cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q` passes, including `tests/test_executor_select.py`; with no `executor` key the node behaves exactly as today.

## Steps

- Write nodes/bridges/feetech_servos/tests/test_executor_select.py (stub `rclpy.experimental.EventsExecutor` and `SingleThreadedExecutor`); `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_executor_select.py -q` FAILS (no key, no make_executor).
- Add the config key with validation, `make_executor`, and spin the configured executor; rerun, PASS; run the whole package suite.
- `uv run poe lint`; update the README config table and rollback note.
- Commit `perf(feetech_servos): configurable EventsExecutor`.

## Notes

Design D2: rclpy.experimental.EventsExecutor (installed rclpy 7.1.12 on the client) runs the same callbacks with far less Python executor overhead; it is experimental and ignores callback groups, so it is used only in single-threaded nodes with one default callback group, and every node keeps a config rollback. The helper lives in each node (no shared package: laser_filter has no shared/ dependency, and shared/ros2_common would pull opencv and scipy into every venv). Expected saving across the five nodes about 0.3 core (estimate; the runbook measures it). Deployment is task client-executor-config, not this task.

servo-torque-off-and-neutral changed bridge.py recently (torque services, neutral save); its open tasks list no files, but rebase on main first. The cycle loop polls with spin_once(timeout_sec=0.0); confirm from the rclpy jazzy source (rclpy/src/rclpy/events_executor/, the 7.1.x tag) that EventsExecutor.spin_once(0.0) returns without blocking when nothing is ready, and state it in the README, because a blocking call would stall the servo read cycle. Servo safety: no servo motion in this task.
