---
title: Swerve events executor
status: todo
repo: ros2-lerobot-sverve-platform
value: "The swerve controller's ~200 callbacks/s run on the cheaper EventsExecutor, switchable back by config."
files: [nodes/swerve_drive_controller/swerve_drive_controller/config.py, nodes/swerve_drive_controller/swerve_drive_controller/node.py, nodes/swerve_drive_controller/tests/test_executor_select.py, nodes/swerve_drive_controller/tests/ros_stubs.py, nodes/swerve_drive_controller/README.md]
verify: "cd nodes/swerve_drive_controller && uv run --frozen pytest tests -q && uv run poe lint"
depends_on: []
owns: []
uses: []
model: ""
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Swerve events executor

## Requirement

Item 2 of the brief for swerve_drive_controller (~200 events/s at 0.23 core): config key in `swerve_drive_controller/config.py` (SwerveControllerConfig), `make_executor` and the spin in `swerve_drive_controller/node.py` (today `rclpy.spin(node)` at node.py:368).

Shared rules for the five events-executor tasks (design D2; approval A2):

- New config key `executor`, values `events` | `single_threaded`, default `single_threaded` in code (so the server, tests and any unset config keep today's behaviour); an unknown value is a config error at load time, like every other invalid key of that node.
- New public function `make_executor(kind: str) -> rclpy.executors.Executor` in the node's module (no leading underscore, typed, docstring with Args/Returns): `events` returns `rclpy.experimental.EventsExecutor()`, `single_threaded` returns `rclpy.executors.SingleThreadedExecutor()`. The import of `rclpy.experimental` sits at the top of the module (no try/except).
- The run function adds the node to `make_executor(config.executor)` and spins that executor (no `rclpy.spin(node)` on the global executor); shutdown handling (KeyboardInterrupt, ExternalShutdownException, RCLError) and `destroy_node`/`try_shutdown` stay exactly as they are.
- EventsExecutor ignores callback groups: confirm the node creates no callback group and no MultiThreadedExecutor semantics, and say so in the README.
- The test stubs (`tests/ros_stubs.py` or the test's own monkeypatch) stub `rclpy.experimental.EventsExecutor`; no test needs a real ROS install.
- Node README: the `executor` key in the config table, with the rollback note (set `single_threaded` in `group_vars/client.yml` and redeploy the node).

## Acceptance conditions

- a1: the executor key defaults to single_threaded [evidence: test:nodes/swerve_drive_controller/tests/test_executor_select.py::test_executor_defaults_to_single_threaded]
- a2: an unknown executor value is rejected at config load [evidence: test:nodes/swerve_drive_controller/tests/test_executor_select.py::test_unknown_executor_is_rejected]
- a3: make_executor returns an EventsExecutor for events and a SingleThreadedExecutor for single_threaded [evidence: test:nodes/swerve_drive_controller/tests/test_executor_select.py::test_make_executor_builds_the_configured_kind]
- a4: the run function spins the configured executor with the node added [evidence: test:nodes/swerve_drive_controller/tests/test_executor_select.py::test_run_spins_the_configured_executor]
- a5: the README documents the executor key and its rollback [evidence: judgement:]
- a6: the node's tests and lint pass [evidence: command:cd nodes/swerve_drive_controller && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/swerve_drive_controller/swerve_drive_controller/config.py`
- `nodes/swerve_drive_controller/swerve_drive_controller/node.py`
- `nodes/swerve_drive_controller/tests/test_executor_select.py`
- `nodes/swerve_drive_controller/tests/ros_stubs.py`
- `nodes/swerve_drive_controller/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
