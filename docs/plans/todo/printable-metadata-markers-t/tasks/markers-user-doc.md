---
title: Markers user doc
status: todo
repo: ros2-lerobot-sverve-platform
value: "One document tells the user how to make, print, place and use markers, and what the robot does with them."
files: [docs/markers.md, tests/README.md, MEMORY.md]
verify: "test -f docs/markers.md && grep -q '^## Payload format' docs/markers.md && grep -q 'test_markers_' tests/README.md && uv run poe lint"
depends_on: [markers-generate-cli, poi-marker-upsert, marker-detector-bench, marker-detector-ansible, mcp-marker-pois, web-ui-marker-pois]
owns: []
uses: []
model: sonnet
estimate: 40
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Markers user doc

## Requirement

- `docs/markers.md` for a mid-level ROS 2 developer, with these sections: `## Payload format` (grammar, kinds table, version rule, examples, capacity per version/ECC), `## Size and frame` (S definition, module size, marker axes, POI position and orientation), `## Generating markers` (config, command, outputs, PDF printing at 100 % on a rigid plate, two-colour Bambu AMS printing from the two STLs or the 3MF, minimum module edge, recommended sizes: 150-200 mm for docking range, 60 mm for buttons near the arm), `## Calibration board` (the ChArUco spec, why 6x5 x 40 mm, used by scripts/head_rgbd_calibrate.py), `## Detection` (stream and zoom paths, pose pipeline, ambiguity and ToF, placeholder rules, `/markers/detections`), `## POIs` (kind marker, upsert and relocation rules, agent and web UI use), `## Range and fallback` (the computed range table: 100 mm v3 decodes to about 0.75 m in the stream and 3 m via zoom at 4 px/module; AprilTag tagStandard41h12 via apriltag_ros as the documented fallback if field range is insufficient).
- `tests/README.md`: every root test file this epic added or changed (test_shared_marker_payload.py, test_markers_symbol.py, test_markers_pdf.py, test_markers_mesh.py, test_markers_charuco.py, test_markers_generate_cli.py, test_marker_detector_config.py, test_ci_workflow.py, test_node_metrics_wiring.py) with what each covers.
- `MEMORY.md`: a decision entry (QR Model 2 with P1 payload over AprilTag/ArUco, why; marker POIs in poi_store via upsert_marker; ChArUco spec owned by the generator).

## Acceptance conditions

- a1: the user doc covers payload, size and frame, generation, calibration board, detection, POIs and range [evidence: doc-section: docs/markers.md#Payload format]
- a2: the doc states the AprilTag fallback and the range table [evidence: doc-section: docs/markers.md#Range and fallback]
- a3: tests README lists every new root test file [evidence: command: for f in test_shared_marker_payload test_markers_symbol test_markers_pdf test_markers_mesh test_markers_charuco test_markers_generate_cli test_marker_detector_config; do grep -q "$f" tests/README.md || exit 1; done]
- a4: MEMORY records the marker decisions [evidence: command: grep -q 'upsert_marker' MEMORY.md]

## Files

- `docs/markers.md`
- `tests/README.md`
- `MEMORY.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
