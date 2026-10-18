---
title: Marker zoom planner
status: todo
repo: ros2-lerobot-sverve-platform
value: Distant or small markers get a full-resolution look without flooding the head camera with zoom requests.
files: [nodes/marker_detector/marker_detector/zoom.py, nodes/marker_detector/tests/test_zoom.py]
verify: "cd nodes/marker_detector && uv run --frozen pytest tests/test_zoom.py -q && uv run --frozen poe lint"
depends_on: [marker-detector-skeleton]
owns: [marker-zoom]
uses: [marker-detector-config]
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

# Marker zoom planner

## Requirement

- `nodes/marker_detector/marker_detector/zoom.py` (design D10a), no ROS: `class ZoomPlanner` with an injected clock:
  - `propose(targets: list[ZoomTarget], now: float) -> ZoomTarget | None`: targets are decoded candidates with `module_px < min_module_px` or undecoded quads; returns at most one: none while a request is outstanding, within `min_interval_s` of the last request, or for a region (grid cell of 1/8 of the image) requested within `cooldown_s`; prefers undecoded quads, then the smallest module.
  - `sent(request_id, target, now)`, `on_result(result: dict, now) -> PendingZoom | None` (matches `request_id`; `ok`/`af_failed` -> waits for the image with `stamp_ns`; other statuses clear the outstanding request and keep the region cooldown), `on_image(stamp_ns, jpeg_bytes) -> ZoomFrame | None` (pairs with the result by `stamp_ns`, either arrival order), `expire(now)` clears a request after `timeout_s`.
  - Disabled (`enabled: false`) -> `propose` always None.
- Tests first in `nodes/marker_detector/tests/test_zoom.py`, seen failing: one proposal then none until the result; region cooldown and global interval honoured; `busy`/`timeout`/`camera_error` give no frame; result-then-image and image-then-result both pair; a stale request expires; disabled planner proposes nothing.

## Acceptance conditions

- a1: at most one zoom is outstanding and cooldowns are honoured [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_zoom.py -q]
- a2: zoom images pair with results by stamp in either order and failed statuses yield nothing [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_zoom.py -q]

## Files

- `nodes/marker_detector/marker_detector/zoom.py`
- `nodes/marker_detector/tests/test_zoom.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
