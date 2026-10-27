---
title: Ground live acceptance on D435
status: todo
repo: ros2-lerobot-sverve-platform
value: "Ground estimation is proven live on the D435 with measured rates and CPU, and the runbook says how to repeat it."
files: [docs/runbooks/ground-estimation.md, tests/test_ground_docs.py, scripts/ground_noise_report.yaml, MEMORY.md, tests/README.md]
verify: "uv run pytest tests/test_ground_docs.py tests/test_ground_estimator_readme.py -q && uv run poe lint"
depends_on: [tune-ground-estimator-on-d43]
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

# Ground live acceptance on D435

## Requirement

LIVE task. Goal G3 and G5 (Ground layer). Design D7, D14. Supersedes the ground acceptance of rgb-d-ground-estimation-and (ground-final-suite-runbook) and drivable-ground-tof-filterin (ground-live-acceptance) per D15. Servo rule: head pan/tilt only; no drive or arm motion.

1. Deploy `ground_estimator_cpp head_rgbd_cpp` with the tuned config (`./scripts/deploy-nodes.sh client ground_estimator_cpp head_rgbd_cpp 2>&1 | tee .logs/deploy-client-ground2.log`).
2. Re-run the three scenes live: `uv run python scripts/ground_record.py` captures `/ground/state` per scene; `uv run python scripts/ground_noise_report.py` (its yaml pointed at the new captures) reports per scene: the box reported lethal, the board and floor never lethal, no drop region on flat floor, state rate >= 1.8 Hz.
3. Rates and CPU from Mimir (no rclpy): head_rgbd_cpp set rate ~2 Hz, ground_estimator_cpp cloud rate >= 14 Hz, realsense / head_rgbd_cpp / ground_estimator_cpp / d435_obstacle_cloud CPU p95 over 10 min, via the runbook queries of `docs/runbooks/ground-estimation.md`.
4. Rewrite `docs/runbooks/ground-estimation.md` for the D435: preconditions (D435 pipeline nodes), scenes, thresholds, commands, the results of steps 2-3. `tests/test_ground_docs.py` pins the runbook sections and topics; the ToF topic and precondition assertions are no longer accurate and are updated to the D435 ones, each named in the commit. `MEMORY.md` records the decision ("ground estimation runs on the D435 cloud, restored <date>").

## Acceptance conditions

- AC1: live scenes show the box lethal and no lethal board, floor or phantom drop [evidence: command:uv run python scripts/ground_noise_report.py]
- AC2: the runbook is re-based on the D435 and pins the measured rates [evidence: test:tests/test_ground_docs.py::test_runbook_pins_the_rates_and_the_deploy_command]
- AC3: the runbook ground topics are outputs of the deployed D435 nodes [evidence: test:tests/test_ground_docs.py::test_runbook_ground_topics_are_node_outputs]
- AC4: MEMORY.md has the decision [evidence: test:tests/test_ground_docs.py::test_memory_has_the_decision]

## Files

- `docs/runbooks/ground-estimation.md`
- `tests/test_ground_docs.py`
- `scripts/ground_noise_report.yaml`
- `MEMORY.md`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Update `tests/test_ground_docs.py` for the D435 runbook (sections, topics, rates); run `uv run pytest tests/test_ground_docs.py -q`; expect failures.
- Run the live steps; outputs in `.logs/`; write the runbook and MEMORY.md entry from them.
- Rerun; run the verify command; commit `docs(runbooks): ground estimation on the D435, live acceptance`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
