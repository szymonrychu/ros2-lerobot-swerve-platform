---
title: Marker decode
status: escalated
repo: ros2-lerobot-sverve-platform
value: "Markers in a camera frame are found and decoded into validated payloads with ordered corners, and undecodable QR codes are located for a zoom."
files: [nodes/marker_detector/marker_detector/decode.py, nodes/marker_detector/marker_detector/synthetic.py, nodes/marker_detector/tests/test_decode.py]
verify: "cd nodes/marker_detector && uv run --frozen pytest tests/test_decode.py -q && uv run --frozen poe lint"
depends_on: [marker-detector-skeleton]
owns: [marker-synthetic-scenes, marker-decode]
uses: [marker-detector-config]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/printable-metadata-markers-t--marker-decode
branch: printable-metadata-markers-t--marker-decode
verified_head: f6ff72ef75957a62e8b56e80826b3a1f368fdaa0
run: printable-metadata-markers-t
provides: [marker-synthetic-helpers]
acs_for_audit: []
process_gaps: []
---

# Marker decode

## Requirement

- `nodes/marker_detector/marker_detector/synthetic.py` (used by tests and the bench): `render_marker_scene(payloads: list[str], sizes_mm: list[float], poses: list[tuple[np.ndarray, np.ndarray]], K: np.ndarray, image_size: tuple[int, int], noise_px: float = 0.0, blur_px: float = 0.0, seed: int = 0) -> np.ndarray` (grey uint8: each payload encoded with segno at the shipped ECC Q, rendered with a 4-module quiet zone at high resolution and warped with the homography of its pose (rvec, tvec of the marker frame of design D3 in the camera frame) through the pinhole K onto a mid-grey background, then optional Gaussian blur and noise); `STREAM_K` (1152x648, HFOV 66 deg: fx = fy = 887, cx 576, cy 324) and `pose_facing(distance_m, yaw_deg=0, pitch_deg=0, roll_deg=0, offset_m=(0, 0))` helpers.
- `nodes/marker_detector/marker_detector/decode.py`:
  - `Candidate` (dataclass): `payload: MarkerPayload`, `text: str`, `corners: np.ndarray` (4x2 float, top-left, top-right, bottom-right, bottom-left in the symbol's own orientation, i.e. the corner of the finder-less side is bottom-right), `version: int`, `module_px: float` (mean edge length / modules).
  - `decode_frame(grey: np.ndarray) -> tuple[list[Candidate], list[Rejection]]`: `zxingcpp.read_barcodes(grey, formats=zxingcpp.BarcodeFormat.QRCode)`; each result's text goes through `decode_payload`; invalid ones become `Rejection(reason="payload", detail=...)`, never candidates; corner order verified against the finder layout (re-ordered if the library reports another start).
  - `find_undecoded(grey: np.ndarray, decoded: list[Candidate]) -> list[np.ndarray]`: QR quads from `cv2.QRCodeDetectorAruco().detectMulti` whose centre is not inside a decoded candidate (zoom targets).
- Tests first in `nodes/marker_detector/tests/test_decode.py`, seen failing: a 150 mm marker at 1 m facing the camera decodes to its payload with corners within 1 px of the projected truth; rotations of 90, 180 and 270 deg roll keep the corner order tied to the symbol (top-left stays the finder corner of the true top-left); two markers in one frame give two candidates; a QR with text `HELLO` and one with `P2/...` give payload rejections and no candidates; module_px matches the projection within 5 %; a 60 mm marker at 3 m (module about 0.7 px) is not decoded but a 100 mm marker at 2.5 m with heavy blur is found by `find_undecoded` when zxing fails, or the test documents the measured decode limit in px/module.

## Acceptance conditions

- a1: synthetic markers decode to validated payloads with corners in symbol order [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_decode.py -q]
- a2: invalid payloads are rejected and never become candidates [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_decode.py -q]
- a3: undecodable QR quads are located for zoom [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_decode.py -q]
- a4: the synthetic scene renderer is part of the package [evidence: symbol: nodes/marker_detector/marker_detector/synthetic.py:render_marker_scene]

## Files

- `nodes/marker_detector/marker_detector/decode.py`
- `nodes/marker_detector/marker_detector/synthetic.py`
- `nodes/marker_detector/tests/test_decode.py`

## Interfaces

- `nodes/marker_detector/marker_detector/synthetic.py`: `render_marker_scene`, `STREAM_K`, `pose_facing` (part `marker-synthetic-scenes`).
- `nodes/marker_detector/marker_detector/decode.py`: `Candidate`, `Rejection`, `decode_frame`, `find_undecoded` (part `marker-decode`).

## Expected outcome

A grey frame becomes a list of valid marker candidates and zoom targets.

## Steps

- Write `tests/test_decode.py`; run `cd nodes/marker_detector && uv run --frozen pytest tests/test_decode.py -q`; see it fail.
- Implement synthetic.py and decode.py; rerun until green; `uv run --frozen poe lint`.
- Commit `feat(marker_detector): decode markers with zxing-cpp`.

## Notes

- zxing-cpp `position` gives top_left, top_right, bottom_right, bottom_left; check them on rotated synthetic symbols (assumption in the design).
- Research: zxing-cpp is the fastest maintained decoder; QRCodeDetectorAruco (OpenCV >= 4.8) is the second opinion for finding quads.

- `marker-payload` (`shared/ros2_common/marker_payload.py`: `decode_payload`, `MarkerInfo`, `marker_info`) is delivered by the blocker epic marker-and-calibration-board; it exists before this task runs.
