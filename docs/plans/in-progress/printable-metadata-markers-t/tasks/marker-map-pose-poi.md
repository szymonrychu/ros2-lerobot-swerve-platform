---
title: Marker map pose POI
status: todo
repo: ros2-lerobot-sverve-platform
value: "An accepted marker pose becomes a map-frame POI sighting, including for zoom stills with their cropped intrinsics."
files: [nodes/marker_detector/marker_detector/geometry.py, nodes/marker_detector/tests/test_geometry.py]
verify: "cd nodes/marker_detector && uv run --frozen pytest tests/test_geometry.py -q && uv run --frozen poe lint"
depends_on: [marker-pose-ippe, poi-marker-upsert]
owns: [marker-geometry]
uses: [marker-detector-config, marker-pose, poi-upsert-marker-op]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/printable-metadata-markers-t--marker-map-pose-poi
branch: printable-metadata-markers-t--marker-map-pose-poi
verified_head: ""
run: printable-metadata-markers-t
provides: []
acs_for_audit: []
process_gaps: []
---

# Marker map pose POI

## Requirement

- `nodes/marker_detector/marker_detector/geometry.py` (scipy Rotation for rotations):
  - `marker_in_map(rvec, tvec, map_from_camera: tuple[np.ndarray, np.ndarray]) -> tuple[np.ndarray, np.ndarray]`: compose map <- camera optical (translation, quaternion xyzw from TF) with camera <- marker; returns position (m) and quaternion xyzw.
  - `zoom_intrinsics(stream_K: np.ndarray, stream_size: tuple[int, int], full_size: tuple[int, int], crop: tuple[int, int, int, int], out_size: tuple[int, int]) -> np.ndarray`: scale K from the stream size to the full sensor size, shift the principal point by the crop origin, scale by out/crop (front camera epic D4/D5 contract).
  - `zoom_request(u: float, v: float, module_px: float, cfg: ZoomConfig, request_id: str) -> dict` and `zoom_factor(module_px, cfg) -> int` (smallest of 2/4/8 reaching `target_module_px`, else 8).
  - `sighting_command(candidate: Candidate, pose: PoseResult, position, quaternion, request_id: str) -> dict`: the `/poi/command` JSON `{"op": "upsert_marker", "request_id": ..., "poi": {"kind": "marker", "x", "y", "z", "orientation": [qx, qy, qz, qw], "marker": MarkerInfo dump (from `marker_info(candidate.payload)`), "confidence"}}` (contract owned by poi_store's `upsert_marker`).
  - `detection_report(stamp_ns: int, source: str, items: list[dict]) -> str`: the `/markers/detections` JSON (stamp, source `stream` or `zoom`, per marker key, payload, position, quaternion, rms_px, ratio, confidence, module_px, chooser `reprojection`/`tof`).
- Tests first in `nodes/marker_detector/tests/test_geometry.py`, seen failing: identity TF returns the camera pose unchanged; a known TF (translation + 90 deg yaw) maps a marker 1 m ahead in the optical frame to the expected map position and orientation; `zoom_intrinsics` for zoom 4 centred crop matches a hand-computed K (fx x4 then /1.125 for 1152->1024 out) and a projected test point lands at the same pixel as cropping the full-res projection; zoom factor choices; the sighting command validates as a poi_store-compatible dict (shape checked against the documented contract: keys, quaternion norm 1, marker block fields of `MarkerInfo`); the report JSON round-trips.

## Acceptance conditions

- a1: camera-frame marker poses are composed into map poses through a TF transform [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_geometry.py -q]
- a2: zoom crop intrinsics follow the crop and output scaling [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_geometry.py -q]
- a3: a sighting becomes an upsert_marker command with MarkerInfo [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_geometry.py -q]

## Files

- `nodes/marker_detector/marker_detector/geometry.py`
- `nodes/marker_detector/tests/test_geometry.py`

## Interfaces

- `nodes/marker_detector/marker_detector/geometry.py`: `marker_in_map`, `zoom_intrinsics`, `zoom_factor`, `zoom_request`, `sighting_command`, `detection_report` (part `marker-geometry`).

## Expected outcome

Pure functions turn a camera-frame pose into the POI command and the detection report.

## Steps

- Write `tests/test_geometry.py`; run `cd nodes/marker_detector && uv run --frozen pytest tests/test_geometry.py -q`; see it fail.
- Implement geometry.py; rerun until green; `uv run --frozen poe lint`.
- Commit `feat(marker_detector): map pose, zoom intrinsics and POI sighting`.

## Notes

- The zoom command fields follow the front camera epic (D4): request_id, u, v (normalised, top-left origin of the default view), zoom 2/4/8, max_px, af.

- `marker-payload` (`shared/ros2_common/marker_payload.py`: `decode_payload`, `MarkerInfo`, `marker_info`) is delivered by the blocker epic marker-and-calibration-board; it exists before this task runs.

- 2026-10-19 run printable-metadata-markers-t: dependency-failed on marker-pose-ippe, poi-marker-upsert
