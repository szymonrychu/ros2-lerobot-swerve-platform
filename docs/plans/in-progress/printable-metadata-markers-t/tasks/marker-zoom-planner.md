---
title: Marker zoom planner
status: in-progress
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
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/printable-metadata-markers-t--marker-zoom-planner
branch: printable-metadata-markers-t--marker-zoom-planner
verified_head: ""
run: ""
provides: []
---

# Marker zoom planner

## Requirement

- `nodes/marker_detector/marker_detector/zoom.py` (design D10a), no ROS: `class ZoomPlanner` with an injected clock:
  - `propose(targets: list[ZoomTarget], now: float) -> ZoomTarget | None`: targets are decoded candidates with `module_px < min_module_px` or undecoded quads; returns at most one: none while a request is outstanding, within `min_interval_s` of the last request, or for a region (grid cell of 1/8 of the image) requested within `cooldown_s`; prefers undecoded quads, then the smallest module.
  - `sent(request_id, target, now)`, `on_result(result: dict, now) -> PendingZoom | None` (matches `request_id`; `ok`/`af_failed` -> waits for the image with `stamp_ns`; other statuses clear the outstanding request and keep the region cooldown), `on_image(stamp_ns, jpeg_bytes) -> ZoomFrame | None` (pairs with the result by `stamp_ns`, either arrival order), `expire(now)` clears a request after `timeout_s`.
  - `auto_zoom: false` (the default) -> `propose` always None; `process_stills: true` -> `on_result`/`on_image` also pair stills of requests this node did not send (any `request_id`), so externally triggered zooms are used; `process_stills: false` -> only this node's own requests pair.
- Tests first in `nodes/marker_detector/tests/test_zoom.py`, seen failing: one proposal then none until the result; region cooldown and global interval honoured; `busy`/`timeout`/`camera_error` give no frame; result-then-image and image-then-result both pair; a stale request expires; with the default `auto_zoom: false` nothing is proposed; an external request's result and image pair when `process_stills` is true and are ignored when false; with `auto_zoom: true` two proposals 5 s apart give one request (`min_interval_s` 10).

## Acceptance conditions

- a1: automatic zoom is off by default, and when enabled at most one request is outstanding and the rate limit and cooldowns are honoured [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_zoom.py -q]
- a2: zoom images pair with results by stamp in either order, external stills are used when process_stills is on, and failed statuses yield nothing [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_zoom.py -q]

## Files

- `nodes/marker_detector/marker_detector/zoom.py`
- `nodes/marker_detector/tests/test_zoom.py`

## Interfaces

- `nodes/marker_detector/marker_detector/zoom.py`: `ZoomPlanner`, `ZoomTarget`, `PendingZoom`, `ZoomFrame` (part `marker-zoom`).

## Expected outcome

The node can ask for zoom stills at a bounded rate and pair the answers.

## Steps

- Write `tests/test_zoom.py`; run `cd nodes/marker_detector && uv run --frozen pytest tests/test_zoom.py -q`; see it fail.
- Implement zoom.py; rerun until green; `uv run --frozen poe lint`.
- Commit `feat(marker_detector): zoom request planner`.

## Notes

- Each zoom gaps the 30 fps stream for 0.5-0.8 s (front camera epic): keep the defaults conservative.
