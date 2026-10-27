---
title: Nav2 live check on D435
status: todo
repo: ros2-lerobot-sverve-platform
value: "Nav2's use of the D435 is proven on the real floor before anyone relies on it."
files: [docs/runbooks/d435-switchover.md, tests/test_d435_runbook.py, tests/README.md]
verify: "uv run pytest tests/test_d435_runbook.py -q && uv run poe lint"
depends_on: [nav2-ground-layer-and-keepou, ground-live-acceptance-on-d4]
model: sonnet
review: ""
verified_head: ""
points: 3
expected_minutes: 34
expected_tokens: 2250000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Nav2 live check on D435

## Requirement

LIVE task. Goal G4. Design D6. Supersedes tof-3d-obstacle-detection-fo final-suite-and-runbook (D15). Motion rule (CLAUDE.md least-dramatic): short, slow goals only (<= 1 m, nav speed limits as configured), user watching; the arm stays parked.

1. `./scripts/deploy-nodes.sh client nav2_bringup 2>&1 | tee .logs/deploy-client-nav2.log`.
2. Empty floor in front of the robot: the local costmap (web_ui Map tab costmap layer) shows no D435 phantom cost within 2 m over 2 min with the head at home and at pan +-30 deg.
3. A 5 cm box 1 m ahead: it appears in the local costmap; a 1 m goal through it is planned around it.
4. A table-edge or step-down drop: CliffStop stops motion toward it (collision monitor state via Mimir/Loki, no rclpy).
5. `docs/runbooks/d435-switchover.md` section "Nav2": each step with its observation and the evidence source (screenshot path under docs/runbooks/img/ or log line); `tests/test_d435_runbook.py::test_nav2_section_is_recorded`.

## Acceptance conditions

- AC1: nav2_bringup is deployed with the D435 inputs [evidence: command:./scripts/deploy-nodes.sh client nav2_bringup]
- AC2: no phantom costs on an empty floor, the box is costed, CliffStop stops at the drop, all recorded [evidence: test:tests/test_d435_runbook.py::test_nav2_section_is_recorded]

## Files

- `docs/runbooks/d435-switchover.md`
- `tests/test_d435_runbook.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Add `test_nav2_section_is_recorded`; run `uv run pytest tests/test_d435_runbook.py -q`; expect a failure.
- Run the live steps with the user present; record observations and evidence paths.
- Rerun; commit `docs(runbooks): Nav2 on the D435, live check`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
