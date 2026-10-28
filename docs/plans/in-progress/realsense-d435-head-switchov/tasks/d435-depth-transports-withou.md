---
title: D435 depth transports without compressed and theora
status: todo
repo: ros2-lerobot-sverve-platform
value: No cv_bridge error spam and no wasted CPU on depth transports nothing can decode
files: [nodes/bridges/realsense_d435/config/realsense_d435.yaml, nodes/bridges/realsense_d435/config/topics.yaml, tests/test_d435_topics.py, tests/test_realsense_d435_deploy.py, nodes/bridges/realsense_d435/README.md]
verify: "uv run pytest tests/test_d435_topics.py tests/test_realsense_d435_deploy.py -q && uv run poe lint"
depends_on: [d435-live-bring-up-and-topic]
model: sonnet
review: ""
verified_head: ""
points: 2
expected_minutes: 15
expected_tokens: 1355248
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# D435 depth transports without compressed and theora

## Requirement

realsense2_camera advertises compressed and theora image transports on the 16UC1 depth topics; when anything subscribes to them the journal shows cv_bridge 16UC1 conversion errors (found by d435-live-bring-up-and-topic review, 2026-10-28). Disable the image_transport plugins that cannot carry 16UC1 depth (compressed, theora; keep raw and compressedDepth where a consumer uses it) on the D435 depth and aligned-depth topics through the realsense node config (image_transport plugin allow/deny parameters for Jazzy), and drop those transport topics from nodes/bridges/realsense_d435/config/topics.yaml. Colour keeps compressed.

## Acceptance conditions

- a1: the D435 depth topics advertise no compressed or theora transport; topics.yaml matches [evidence: command:uv run pytest tests/test_d435_topics.py tests/test_realsense_d435_deploy.py -q]
- a2: live: subscribing to the D435 depth transports produces no cv_bridge 16UC1 error in the journal for 10 min [evidence: command:./scripts/d435_diag.sh --since 10min]

## Files

- `nodes/bridges/realsense_d435/config/realsense_d435.yaml`
- `nodes/bridges/realsense_d435/config/topics.yaml`
- `tests/test_d435_topics.py`
- `tests/test_realsense_d435_deploy.py`
- `nodes/bridges/realsense_d435/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

1. Pin in tests that depth topics list no compressed/theora transports; fails.
2. Set the image_transport plugin parameters in realsense_d435.yaml; update topics.yaml.
3. Deploy realsense_d435 (orchestrator pushes first), run d435_diag.sh --since 10min.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
