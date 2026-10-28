---
title: Marker detector zoom retired
status: done
repo: ros2-lerobot-sverve-platform
value: marker_detector stops waiting for zoom stills no camera produces.
files: [nodes/marker_detector/marker_detector/zoom.py, nodes/marker_detector/tests/test_zoom.py, nodes/marker_detector/marker_detector/config.py, nodes/marker_detector/marker_detector/pipeline.py, nodes/marker_detector/marker_detector/node.py, nodes/marker_detector/tests/test_config.py, ansible/group_vars/client.yml, tests/test_marker_detector_config.py]
verify: "cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run pytest tests/test_marker_detector_config.py -q && uv run poe test-ansible"
depends_on: [marker-detector-on-d435]
model: opus
review: ""
verified_head: b4b738dc
points: 2
expected_minutes: 23
expected_tokens: 1500000
points_actual: 3
actual_minutes: 17
actual_tokens: 4546634
actual_roles: "impl 14, review 3"
actual_role_tokens: "impl 4307096, review 239538"
---

# Marker detector zoom retired

## Requirement

Goal G9: head_camera_cpp "retired ... with their consumers". Design D11, Q1 (retire zoom everywhere; amended before it runs if the gate answers otherwise).

- Delete `marker_detector/zoom.py` and `tests/test_zoom.py` (the zoom stills came from head_camera_cpp's full-resolution crop).
- `marker_detector/config.py`: the `zoom_*` topics and `ZoomConfig` are removed (unknown keys rejected). `marker_detector/pipeline.py` and `marker_detector/node.py` no longer process zoom stills.
- client.yml marker_detector block: zoom lines removed. `tests/test_marker_detector_config.py` matches (zoom pins are no longer accurate).
- New `tests/test_config.py::test_zoom_keys_are_rejected` and a retirement assert `test_zoom_module_is_gone` (git ls-files of zoom.py is empty).
TDD exception for the deletion: pinned by the retirement test.

## Acceptance conditions

- AC1: zoom config keys are rejected [evidence: test:nodes/marker_detector/tests/test_config.py::test_zoom_keys_are_rejected]
- AC2: the zoom module is no longer tracked [evidence: test:nodes/marker_detector/tests/test_config.py::test_zoom_module_is_gone]
- AC3: suite, lint and the deployed config pin pass [evidence: command:cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run pytest tests/test_marker_detector_config.py -q]

## Files

- `nodes/marker_detector/marker_detector/zoom.py`
- `nodes/marker_detector/tests/test_zoom.py`
- `nodes/marker_detector/marker_detector/config.py`
- `nodes/marker_detector/marker_detector/pipeline.py`
- `nodes/marker_detector/marker_detector/node.py`
- `nodes/marker_detector/tests/test_config.py`
- `ansible/group_vars/client.yml`
- `tests/test_marker_detector_config.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write the two tests; run them; expect failures.
- Remove the zoom stage, config and client.yml lines; `git rm` zoom.py and its test; rerun; expect pass.
- Run the verify command; commit `refactor(marker_detector): retire the head camera zoom stage`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done: root suite on b4b738dc: 2151 passed, 9 skipped (implementer run; reviewer's root run did not finish)
