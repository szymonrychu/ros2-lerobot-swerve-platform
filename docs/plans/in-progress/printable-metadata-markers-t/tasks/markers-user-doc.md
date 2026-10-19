---
title: Markers user doc
status: escalated
repo: ros2-lerobot-sverve-platform
value: "One document tells the user how to make, print, place and use markers, and what the robot does with them."
files: [docs/markers.md, tests/README.md, MEMORY.md]
verify: "test -f docs/markers.md && grep -q '^## Payload format' docs/markers.md && grep -q 'test_marker_detector_config' tests/README.md && uv run poe lint"
depends_on: [poi-marker-upsert, marker-detector-bench, marker-detector-ansible, mcp-marker-pois, web-ui-marker-pois]
owns: []
uses: []
model: sonnet
estimate: 40
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/printable-metadata-markers-t--markers-user-doc
branch: printable-metadata-markers-t--markers-user-doc
verified_head: 5c0334d34a99904ccabefbf2535b9eeead588ccc
run: printable-metadata-markers-t
provides: []
acs_for_audit: [a1]
process_gaps: []
---

# Markers user doc

## Requirement

- `docs/markers.md` for a mid-level ROS 2 developer, with these sections: `## Payload format` (grammar, kinds table, version rule, examples, capacity per version/ECC), `## Size and frame` (S definition, module size, marker axes, POI position and orientation), `## Generating markers` (pointer to the marker-and-calibration-board generator: config, command, outputs, PDF printing at 100 % on a rigid plate, two-colour Bambu AMS printing from the two STLs or the 3MF, minimum module edge, recommended sizes: 150-200 mm for docking range, 60 mm for buttons near the arm), `## Calibration board` (the ChArUco spec, why 6x5 x 40 mm, used by scripts/head_rgbd_calibrate.py), `## Detection` (stream and zoom paths, pose pipeline, ambiguity and ToF, placeholder rules, `/markers/detections`), `## POIs` (kind marker, upsert and relocation rules, agent and web UI use), `## Range and fallback` (the computed range table: 100 mm v3 decodes to about 0.75 m in the stream and 3 m via zoom at 4 px/module; AprilTag tagStandard41h12 via apriltag_ros as the documented fallback if field range is insufficient).
- `tests/README.md`: every root test file this epic added or changed (test_marker_detector_config.py, test_ci_workflow.py, test_node_metrics_wiring.py; the generator's tests are listed by marker-and-calibration-board) with what each covers.
- `MEMORY.md`: a decision entry (QR Model 2 with P1 payload over AprilTag/ArUco, why; marker POIs in poi_store via upsert_marker; ChArUco spec owned by the generator).

## Acceptance conditions

- a1: the user doc covers payload, size and frame, generation, calibration board, detection, POIs and range [evidence: doc-section: docs/markers.md#Payload format]
- a2: the doc states the AprilTag fallback and the range table [evidence: doc-section: docs/markers.md#Range and fallback]
- a3: tests README lists every new root test file [evidence: command: grep -q test_marker_detector_config tests/README.md]
- a4: MEMORY records the marker decisions [evidence: command: grep -q 'upsert_marker' MEMORY.md]

## Files

- `docs/markers.md`
- `tests/README.md`
- `MEMORY.md`

## Interfaces

- `docs/markers.md` sections as listed.

## Expected outcome

The marker system is documented end to end.

## Steps

- Write docs/markers.md from design D1-D14 and the delivered code; update tests/README.md and MEMORY.md; run the verify command.
- Commit `docs: printable metadata markers`.

## Notes

- Documentation-only task (TDD exception: doc edit); keep it in sync with the actual defaults in the code.

- 2026-10-19 run printable-metadata-markers-t: dependency-failed on poi-marker-upsert, marker-detector-bench, marker-detector-ansible

- 2026-10-19 run printable-metadata-markers-t: dependency-failed on marker-detector-bench, marker-detector-ansible
