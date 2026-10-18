---
title: Marker map pose POI
status: todo
repo: ros2-lerobot-sverve-platform
value: "An accepted marker pose becomes a map-frame POI sighting, including for zoom stills with their cropped intrinsics."
files: [nodes/marker_detector/marker_detector/geometry.py, nodes/marker_detector/tests/test_geometry.py]
verify: "cd nodes/marker_detector && uv run --frozen pytest tests/test_geometry.py -q && uv run --frozen poe lint"
depends_on: [marker-pose-ippe, poi-marker-upsert]
owns: [marker-geometry]
uses: [marker-detector-config, marker-pose, marker-payload, poi-upsert-marker-op]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
