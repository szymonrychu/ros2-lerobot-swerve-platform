---
title: ToF stamp latency fix
status: todo
repo: ros2-lerobot-sverve-platform
value: "ToF frames carry the time they were captured instead of the time Python got round to publishing them, so depth pairs with the right RGB frame."
files: [nodes/bridges/arducam_tof/arducam_tof/bridge.py, nodes/bridges/arducam_tof/arducam_tof/config.py, nodes/bridges/arducam_tof/tests/test_bridge.py, nodes/bridges/arducam_tof/tests/test_config.py, nodes/bridges/arducam_tof/README.md]
verify: "cd nodes/bridges/arducam_tof && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: []
owns: []
uses: []
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# ToF stamp latency fix

## Requirement

- `TofConfig` gains `stamp_latency_ms: float` (default 0.0, 0 <= v <= 200; documented as the measured capture-to-return latency of the SDK frame, i.e. the ToF-RGB offset found by the design's pan-sweep procedure, D5).
- `TofBridge.publish_frame` reads the node clock immediately after `requestFrame` returns a frame (before `frame_arrays`, `depth_to_mm`, `amplitude_to_mono16` and `releaseFrame`) and stamps the shared header with that reading minus `stamp_latency_ms` (as an rclpy `Time` / `Duration`, never a negative stamp: a result below zero is clamped to the reading itself and logged once). Today the stamp is taken after the conversions (`header.stamp = self.node.get_clock().now()` at the end of `publish_frame`), which adds conversion time on top of the SDK latency.
- Dropped frames (no frame, wrong type, malformed) keep publishing nothing; the startup log line includes `stamp_latency_ms`.
- Tests first, seen failing: `tests/test_config.py` (default 0.0, a value round-trips, -1 and 201 rejected naming the key) and `tests/test_bridge.py` (with a fake clock that advances during the conversion step: the published depth, amplitude and camera_info share one header whose stamp equals the clock reading taken right after `requestFrame` minus the configured latency; latency 0 gives that reading exactly; a latency larger than the reading clamps).
- `nodes/bridges/arducam_tof/README.md`: a `## Timestamps` section: what the stamp means (frame return minus `stamp_latency_ms`), why (the RGB camera stamps at start of exposure), and how the value is measured (pan sweep, docs/head-rgbd-fusion-methodology.md) and set (client.yml `arducam_tof` config).

## Acceptance conditions

- a1: the header stamp is the clock reading right after requestFrame minus stamp_latency_ms, shared by all three messages [evidence: test: nodes/bridges/arducam_tof/tests/test_bridge.py]
- a2: stamp_latency_ms validates (0..200, default 0) [evidence: test: nodes/bridges/arducam_tof/tests/test_config.py]
- a3: the README documents the stamp meaning and the measurement [evidence: doc-section: nodes/bridges/arducam_tof/README.md#Timestamps]
- a4: arducam_tof tests and lint pass [evidence: command: cd nodes/bridges/arducam_tof && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/bridges/arducam_tof/arducam_tof/bridge.py`
- `nodes/bridges/arducam_tof/arducam_tof/config.py`
- `nodes/bridges/arducam_tof/tests/test_bridge.py`
- `nodes/bridges/arducam_tof/tests/test_config.py`
- `nodes/bridges/arducam_tof/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
