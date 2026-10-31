# Printable metadata markers that become POIs implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: Marker detector skeleton

- Status: done
- Files: `nodes/marker_detector/pyproject.toml`, `nodes/marker_detector/uv.lock`, `nodes/marker_detector/marker_detector/__init__.py`, `nodes/marker_detector/marker_detector/config.py`, `nodes/marker_detector/tests/__init__.py`, `nodes/marker_detector/tests/test_config.py`, `nodes/marker_detector/README.md`
- Verify: `cd nodes/marker_detector && uv run --frozen pytest tests/test_config.py -q && uv run --frozen poe lint && uv lock --check`
- Verified head: cb4e456a3a9655bf710161fb1a5831593bb77775

#### Requirement

- `nodes/marker_detector/pyproject.toml` (pattern: nodes/poi_store/pyproject.toml, hatchling, ruff, vulture, poe `lint`/`lint-fix`/`test`, pytest `pythonpath = ["."]`): runtime dependencies `opencv-python-headless>=4.9,<5`, `zxing-cpp>=2.2,<4`, `segno>=1.6,<2` (synthetic scenes and the bench), `numpy>=1.26`, `scipy>=1.11`, `pydantic>=2.5`, `pyyaml>=6.0`, `ros2-common` and `ros2-metrics` (path sources `../../shared`, `../../shared/ros2_metrics`); dev: ruff==0.15.20, vulture, poethepoet, pytest. `uv.lock` generated with `uv lock`.
- `nodes/marker_detector/marker_detector/config.py`: pydantic models (extra forbidden, errors name the key), `load_config(path: Path) -> MarkerDetectorConfig` and `config_path() -> Path` (env `MARKER_DETECTOR_CONFIG`, default `/etc/ros2/marker_detector/config.yaml`):
  - `topics`: `image` `/head_camera/image_raw/compressed`, `camera_info` `/head_camera/camera_info`, `zoom_command` `/head_camera/zoom/command`, `zoom_image` `/head_camera/zoom/image`, `zoom_result` `/head_camera/zoom/result`, `depth` `/head_rgbd/depth/image_raw`, `depth_info` `/head_rgbd/depth/camera_info`, `poi_command` `/poi/command`, `detections` `/markers/detections`.
  - `process_hz` 2.0 (0.1..15); `map_frame` `map`; `tf_timeout_s` 0.2.
  - `pose`: `ambiguity_ratio` 2.0 (> 1), `max_rms_px` 1.0, `min_module_px` 4.0, `min_confidence` 0.3.
  - `zoom`: `process_stills` true (process every `ok`/`af_failed` zoom still on the zoom topics, whoever requested it: web UI click-to-zoom, the agent's `get_camera_image` zoom, or this node), `auto_zoom` false (the node requests zoom stills itself only when true; off until the user decides), `min_interval_s` 10 (rate limit for automatic requests: at most one per this many seconds), `min_module_px` 6.0, `target_module_px` 12.0, `cooldown_s` 30, `timeout_s` 8, `max_px` 1024, `full_size` [4608, 2592]; validator `target_module_px > min_module_px >= pose.min_module_px`.
  - `tof`: `enabled` true, `max_dt_s` 0.15, `max_angle_deg` 20, `min_valid_px` 30, `max_residual_mm` 10.
  - `metrics_port: int | None` (None falls back to env METRICS_PORT).
- Tests first in `nodes/marker_detector/tests/test_config.py`, seen failing: defaults load from an empty yaml and from a full one; `auto_zoom` defaults to false and `process_stills` to true; an unknown key and an inconsistent zoom threshold are rejected naming the key; the env override is honoured; the venv check: `cv2.__version__ >= 4.8`, `hasattr(cv2, "QRCodeDetectorAruco")`, `hasattr(cv2.aruco, "CharucoDetector")`, and `import zxingcpp` succeeds (research fix-as-ac: Ubuntu's OpenCV 4.6 must not be the one imported).
- `nodes/marker_detector/README.md`: purpose, inputs and outputs (design D8), the config table, that it needs a calibrated `/head_camera/camera_info` and head TF; later tasks extend it.

#### Acceptance conditions

- a1: the config loads defaults and rejects unknown keys and inconsistent thresholds [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_config.py -q]
- a2: the node venv imports OpenCV >= 4.8 with QRCodeDetectorAruco and CharucoDetector, and zxing-cpp [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_config.py -q]
- a3: the node lock is consistent [evidence: command: cd nodes/marker_detector && uv lock --check]
- a4: the node README exists with inputs, outputs and config [evidence: command: grep -q '/poi/command' nodes/marker_detector/README.md]

#### Files

- `nodes/marker_detector/pyproject.toml`
- `nodes/marker_detector/uv.lock`
- `nodes/marker_detector/marker_detector/__init__.py`
- `nodes/marker_detector/marker_detector/config.py`
- `nodes/marker_detector/tests/__init__.py`
- `nodes/marker_detector/tests/test_config.py`
- `nodes/marker_detector/README.md`

#### Interfaces

- `nodes/marker_detector/marker_detector/config.py`: `MarkerDetectorConfig` (sections `topics`, `pose`, `zoom`, `tof`), `load_config`, `config_path` (part `marker-detector-config`).

#### Expected outcome

`cd nodes/marker_detector && uv run --frozen pytest tests -q` passes with the validated config and the right OpenCV.

#### Steps

- Create the package and pyproject; `cd nodes/marker_detector && uv lock && uv sync`.
- Write `tests/test_config.py`; run `uv run --frozen pytest tests/test_config.py -q`; see it fail.
- Implement config.py; rerun until green; `uv run --frozen poe lint`.
- Write the README; commit `feat(marker_detector): package skeleton and config`.

#### Notes

- On the Pi the venv is `--system-site-packages`; the wheel inside the venv shadows apt python3-opencv 4.6, which the import test proves locally and the runbook proves on the robot.
- The CI and lint wiring is the next task (marker-detector-ci-wiring); until it lands the root lint-coverage test lists the new directory as missing.

#### Ruling

Accepted by the orchestrator: verified and merged into main (marker_detector 45, poi_store 269 passed); marker-decode a4 and poi-marker-upsert a5 escalations were symbol-lookup false positives (render_marker_scene at synthetic.py:79, merge_marker_sighting at markers.py:41).

### S2: POI marker model

- Status: done
- Files: `nodes/poi_store/poi_store/models.py`, `nodes/poi_store/tests/test_marker_model.py`, `nodes/poi_store/README.md`
- Verify: `cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: 76ad06eae9c8523816fb8cbba903bd1a481202f2

#### Requirement

- `nodes/poi_store/poi_store/models.py` (design D11): `Poi.kind` adds `"marker"`; new fields `z: Coordinate = 0.0`, `orientation: tuple[float, float, float, float] | None = None` (`[qx, qy, qz, qw]`), `marker: MarkerInfo | None = None` (`from ros2_common.marker_payload import MarkerInfo`). Validator: kind marker requires `marker` and `orientation`; the quaternion must have a norm within 0.01 of 1 and is normalised; other kinds must have `marker` None and get `orientation` None; a marker has no polygon and no zone. The docstring's attribute list documents the new fields.
- Existing stored files (no z, orientation, marker) load unchanged; every existing poi_store test stays green.
- Tests first in `nodes/poi_store/tests/test_marker_model.py`, seen failing: a full marker POI validates and round-trips through `to_json_dict` and `Poi(**d)`; a marker without `marker` or without `orientation` is rejected; a non-unit quaternion within tolerance is normalised and one outside is rejected; `marker` on a point is rejected; a legacy point dict without the new fields loads with z 0 and orientation None; a marker with a zone is rejected.
- `nodes/poi_store/README.md`: the contract section documents kind `marker`, `z`, `orientation`, `marker` (MarkerInfo fields) and that sighting fields apply to markers, and that there is no `category` field yet.

#### Acceptance conditions

- a1: marker POIs validate with pose and MarkerInfo and round-trip through JSON [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_marker_model.py -q]
- a2: legacy POIs without the new fields load unchanged and existing tests stay green [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests -q]
- a3: invalid marker POIs (missing info, bad quaternion, marker on a point, zone) are rejected [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_marker_model.py -q]
- a4: the poi_store README documents the marker contract [evidence: command: grep -q 'marker' nodes/poi_store/README.md && grep -q 'orientation' nodes/poi_store/README.md]

#### Files

- `nodes/poi_store/poi_store/models.py`
- `nodes/poi_store/tests/test_marker_model.py`
- `nodes/poi_store/README.md`

#### Interfaces

- `nodes/poi_store/poi_store/models.py:Poi` with kind `marker`, `z`, `orientation`, `marker: MarkerInfo` (part `poi-marker-contract`).

#### Expected outcome

poi_store validates and lists marker POIs with full pose.

#### Steps

- Write `nodes/poi_store/tests/test_marker_model.py`; run `cd nodes/poi_store && uv run --frozen pytest tests/test_marker_model.py -q`; see it fail.
- Implement the model change; rerun; run `cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Update the README; commit `feat(poi_store): marker POI kind with pose and payload`.

#### Notes

- poi_store already depends on `ros2-common` (path source); no lock change is needed for a new module in it, but run `uv lock --check` to confirm.
- Do not add `category` (agent-skills epic).

- `marker-payload` (`shared/ros2_common/marker_payload.py`: `decode_payload`, `MarkerInfo`, `marker_info`) is delivered by the blocker epic marker-and-calibration-board; it exists before this task runs.

#### Ruling

Accepted by the orchestrator: verified and merged into main (marker_detector 45, poi_store 269 passed); marker-decode a4 and poi-marker-upsert a5 escalations were symbol-lookup false positives (render_marker_scene at synthetic.py:79, merge_marker_sighting at markers.py:41).

### S3: Marker decode

- Status: done
- Files: `nodes/marker_detector/marker_detector/decode.py`, `nodes/marker_detector/marker_detector/synthetic.py`, `nodes/marker_detector/tests/test_decode.py`
- Verify: `cd nodes/marker_detector && uv run --frozen pytest tests/test_decode.py -q && uv run --frozen poe lint`
- Verified head: f6ff72ef75957a62e8b56e80826b3a1f368fdaa0

#### Requirement

- `nodes/marker_detector/marker_detector/synthetic.py` (used by tests and the bench): `render_marker_scene(payloads: list[str], sizes_mm: list[float], poses: list[tuple[np.ndarray, np.ndarray]], K: np.ndarray, image_size: tuple[int, int], noise_px: float = 0.0, blur_px: float = 0.0, seed: int = 0) -> np.ndarray` (grey uint8: each payload encoded with segno at the shipped ECC Q, rendered with a 4-module quiet zone at high resolution and warped with the homography of its pose (rvec, tvec of the marker frame of design D3 in the camera frame) through the pinhole K onto a mid-grey background, then optional Gaussian blur and noise); `STREAM_K` (1152x648, HFOV 66 deg: fx = fy = 887, cx 576, cy 324) and `pose_facing(distance_m, yaw_deg=0, pitch_deg=0, roll_deg=0, offset_m=(0, 0))` helpers.
- `nodes/marker_detector/marker_detector/decode.py`:
  - `Candidate` (dataclass): `payload: MarkerPayload`, `text: str`, `corners: np.ndarray` (4x2 float, top-left, top-right, bottom-right, bottom-left in the symbol's own orientation, i.e. the corner of the finder-less side is bottom-right), `version: int`, `module_px: float` (mean edge length / modules).
  - `decode_frame(grey: np.ndarray) -> tuple[list[Candidate], list[Rejection]]`: `zxingcpp.read_barcodes(grey, formats=zxingcpp.BarcodeFormat.QRCode)`; each result's text goes through `decode_payload`; invalid ones become `Rejection(reason="payload", detail=...)`, never candidates; corner order verified against the finder layout (re-ordered if the library reports another start).
  - `find_undecoded(grey: np.ndarray, decoded: list[Candidate]) -> list[np.ndarray]`: QR quads from `cv2.QRCodeDetectorAruco().detectMulti` whose centre is not inside a decoded candidate (zoom targets).
- Tests first in `nodes/marker_detector/tests/test_decode.py`, seen failing: a 150 mm marker at 1 m facing the camera decodes to its payload with corners within 1 px of the projected truth; rotations of 90, 180 and 270 deg roll keep the corner order tied to the symbol (top-left stays the finder corner of the true top-left); two markers in one frame give two candidates; a QR with text `HELLO` and one with `P2/...` give payload rejections and no candidates; module_px matches the projection within 5 %; a 60 mm marker at 3 m (module about 0.7 px) is not decoded but a 100 mm marker at 2.5 m with heavy blur is found by `find_undecoded` when zxing fails, or the test documents the measured decode limit in px/module.

#### Acceptance conditions

- a1: synthetic markers decode to validated payloads with corners in symbol order [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_decode.py -q]
- a2: invalid payloads are rejected and never become candidates [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_decode.py -q]
- a3: undecodable QR quads are located for zoom [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_decode.py -q]
- a4: the synthetic scene renderer is part of the package [evidence: symbol: nodes/marker_detector/marker_detector/synthetic.py:render_marker_scene]

#### Files

- `nodes/marker_detector/marker_detector/decode.py`
- `nodes/marker_detector/marker_detector/synthetic.py`
- `nodes/marker_detector/tests/test_decode.py`

#### Interfaces

- `nodes/marker_detector/marker_detector/synthetic.py`: `render_marker_scene`, `STREAM_K`, `pose_facing` (part `marker-synthetic-scenes`).
- `nodes/marker_detector/marker_detector/decode.py`: `Candidate`, `Rejection`, `decode_frame`, `find_undecoded` (part `marker-decode`).

#### Expected outcome

A grey frame becomes a list of valid marker candidates and zoom targets.

#### Steps

- Write `tests/test_decode.py`; run `cd nodes/marker_detector && uv run --frozen pytest tests/test_decode.py -q`; see it fail.
- Implement synthetic.py and decode.py; rerun until green; `uv run --frozen poe lint`.
- Commit `feat(marker_detector): decode markers with zxing-cpp`.

#### Notes

- zxing-cpp `position` gives top_left, top_right, bottom_right, bottom_left; check them on rotated synthetic symbols (assumption in the design).
- Research: zxing-cpp is the fastest maintained decoder; QRCodeDetectorAruco (OpenCV >= 4.8) is the second opinion for finding quads.

- `marker-payload` (`shared/ros2_common/marker_payload.py`: `decode_payload`, `MarkerInfo`, `marker_info`) is delivered by the blocker epic marker-and-calibration-board; it exists before this task runs.

#### Ruling

Accepted by the orchestrator: verified and merged into main (marker_detector 45, poi_store 269 passed); marker-decode a4 and poi-marker-upsert a5 escalations were symbol-lookup false positives (render_marker_scene at synthetic.py:79, merge_marker_sighting at markers.py:41).

### S4: Marker detector CI wiring

- Status: done
- Files: `pyproject.toml`, `scripts/lint-all-nodes.sh`, `.github/workflows/ci.yml`, `tests/test_ci_workflow.py`
- Verify: `uv run pytest tests/test_ci_workflow.py tests/test_node_lint_coverage.py -q && uv run poe lint`
- Verified head: 06db94e685cbad97e059dff4d2e5f05f5f05b199

#### Requirement

- Root `pyproject.toml` `lint-nodes` gains `(cd nodes/marker_detector && uv run --frozen poe lint)`; `scripts/lint-all-nodes.sh` gains `nodes/marker_detector`; `.github/workflows/ci.yml` node matrix gains `marker_detector` (sorted position); `tests/test_ci_workflow.py` node count follows the tree (pattern: commit 41d1f14e for costmap_zones).
- Tests: `tests/test_ci_workflow.py` and `tests/test_node_lint_coverage.py` fail before the wiring (the node directory exists from the skeleton task) and pass after.

#### Acceptance conditions

- a1: the CI matrix equals the node directories including marker_detector [evidence: test: tests/test_ci_workflow.py]
- a2: root lint tasks cover marker_detector [evidence: test: tests/test_node_lint_coverage.py]

#### Files

- `pyproject.toml`
- `scripts/lint-all-nodes.sh`
- `.github/workflows/ci.yml`
- `tests/test_ci_workflow.py`

#### Interfaces

- CI node matrix entry `marker_detector`; root `poe lint-nodes` entry.

#### Expected outcome

CI and `uv run poe lint-nodes` include the detector.

#### Steps

- Run `uv run pytest tests/test_ci_workflow.py tests/test_node_lint_coverage.py -q`; see it fail on the missing entries.
- Add the entries; rerun until green; `uv run poe lint`.
- Commit `ci: lint and test marker_detector`.

#### Notes

- Other epics may add nodes first; set the count to what the tree holds.

#### Ruling

Accepted by the orchestrator: verified and merged into main (marker_detector 45, poi_store 269 passed); marker-decode a4 and poi-marker-upsert a5 escalations were symbol-lookup false positives (render_marker_scene at synthetic.py:79, merge_marker_sighting at markers.py:41).

### S5: Marker zoom planner

- Status: done
- Files: `nodes/marker_detector/marker_detector/zoom.py`, `nodes/marker_detector/tests/test_zoom.py`
- Verify: `cd nodes/marker_detector && uv run --frozen pytest tests/test_zoom.py -q && uv run --frozen poe lint`
- Verified head: e63fee73ce9685c4a62e1fc87e6d3bb276476021

#### Requirement

- `nodes/marker_detector/marker_detector/zoom.py` (design D10a), no ROS: `class ZoomPlanner` with an injected clock:
  - `propose(targets: list[ZoomTarget], now: float) -> ZoomTarget | None`: targets are decoded candidates with `module_px < min_module_px` or undecoded quads; returns at most one: none while a request is outstanding, within `min_interval_s` of the last request, or for a region (grid cell of 1/8 of the image) requested within `cooldown_s`; prefers undecoded quads, then the smallest module.
  - `sent(request_id, target, now)`, `on_result(result: dict, now) -> PendingZoom | None` (matches `request_id`; `ok`/`af_failed` -> waits for the image with `stamp_ns`; other statuses clear the outstanding request and keep the region cooldown), `on_image(stamp_ns, jpeg_bytes) -> ZoomFrame | None` (pairs with the result by `stamp_ns`, either arrival order), `expire(now)` clears a request after `timeout_s`.
  - `auto_zoom: false` (the default) -> `propose` always None; `process_stills: true` -> `on_result`/`on_image` also pair stills of requests this node did not send (any `request_id`), so externally triggered zooms are used; `process_stills: false` -> only this node's own requests pair.
- Tests first in `nodes/marker_detector/tests/test_zoom.py`, seen failing: one proposal then none until the result; region cooldown and global interval honoured; `busy`/`timeout`/`camera_error` give no frame; result-then-image and image-then-result both pair; a stale request expires; with the default `auto_zoom: false` nothing is proposed; an external request's result and image pair when `process_stills` is true and are ignored when false; with `auto_zoom: true` two proposals 5 s apart give one request (`min_interval_s` 10).

#### Acceptance conditions

- a1: automatic zoom is off by default, and when enabled at most one request is outstanding and the rate limit and cooldowns are honoured [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_zoom.py -q]
- a2: zoom images pair with results by stamp in either order, external stills are used when process_stills is on, and failed statuses yield nothing [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_zoom.py -q]

#### Files

- `nodes/marker_detector/marker_detector/zoom.py`
- `nodes/marker_detector/tests/test_zoom.py`

#### Interfaces

- `nodes/marker_detector/marker_detector/zoom.py`: `ZoomPlanner`, `ZoomTarget`, `PendingZoom`, `ZoomFrame` (part `marker-zoom`).

#### Expected outcome

The node can ask for zoom stills at a bounded rate and pair the answers.

#### Steps

- Write `tests/test_zoom.py`; run `cd nodes/marker_detector && uv run --frozen pytest tests/test_zoom.py -q`; see it fail.
- Implement zoom.py; rerun until green; `uv run --frozen poe lint`.
- Commit `feat(marker_detector): zoom request planner`.

#### Notes

- Each zoom gaps the 30 fps stream for 0.5-0.8 s (front camera epic): keep the defaults conservative.

#### Ruling

Accepted by the orchestrator: verified and merged into main (marker_detector 45, poi_store 269 passed); marker-decode a4 and poi-marker-upsert a5 escalations were symbol-lookup false positives (render_marker_scene at synthetic.py:79, merge_marker_sighting at markers.py:41).

### S6: MCP marker POIs

- Status: done
- Files: `nodes/mcp_server/mcp_server/perception_models.py`, `nodes/mcp_server/mcp_server/perception_tools.py`, `nodes/mcp_server/mcp_server/approach_tools.py`, `nodes/mcp_server/mcp_server/approach.py`, `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/tests/test_marker_pois.py`, `nodes/mcp_server/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_marker_pois.py tests/test_approach.py tests/test_approach_tools.py tests/test_perception_tools.py -q && uv run --frozen poe lint`
- Verified head: 8a502419b215cd4d549af00b20ca765378ed0a74

#### Requirement

- `perception_models.py`: `PoiView` gains `z: float | None`, `marker: MarkerView | None` (kind, id, size_mm, params, times_seen, last_seen, confidence), `facing_yaw_deg: float | None`, `approach_pose: dict | None` (`x`, `y`, `yaw_deg`), with Field descriptions; values only from a marker POI (null otherwise).
- A pure helper (in perception_tools.py or a function in perception_models.py, the builder's choice) `marker_facing(poi: dict, standoff_m: float) -> tuple[float | None, dict | None]`: the map yaw of the marker's +z axis projected on the floor; None when the +z axis is within 45 deg of vertical (marker lying flat or on a ceiling); approach pose = marker position + standoff along the projected normal, yaw facing the marker.
- `config.py`: `markers.standoff_m` 0.6 (0.2..2.0) under the existing config model; existing configs load unchanged.
- `perception_tools.py`: `list_pois` documents kind `marker` (fixed places from printed markers: base, charger, drop-off, button, switch, tool station; pose owned by the detector) and gains an optional `marker_kind` filter (only marker POIs of that kind); markers appear in the normal list otherwise; `add_poi` stays point/area only.
- `approach_tools.py` / `approach.py`: `POI_KINDS` includes `marker`; a marker `poi_id` target takes its `z` when `z_m` is omitted (z_m still overrides), other POI kinds still require `z_m`; the tool description says so.
- Tests first in `nodes/mcp_server/tests/test_marker_pois.py`, seen failing: a wall marker facing +x at (2, 1, 0.8) gives facing_yaw_deg 0 and approach pose (2.6, 1, 180 deg); a floor marker gives null facing and approach; `list_pois` with `marker_kind="charger"` returns only charger markers; PoiView of a point has null marker fields; approach with a marker poi_id and no z_m resolves to the marker's z, and a point poi_id without z_m still raises; existing approach and perception tests stay green.
- `nodes/mcp_server/README.md`: marker POIs in the POI section (fields, facing, approach pose, filter, approach targets).

#### Acceptance conditions

- a1: list_pois shows marker fields, facing yaw and approach pose and filters by marker kind [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_marker_pois.py -q]
- a2: approach targets accept a marker POI and use its height [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_marker_pois.py tests/test_approach.py tests/test_approach_tools.py -q]
- a3: existing perception tools keep working [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_perception_tools.py -q]
- a4: the mcp_server README documents marker POIs [evidence: command: grep -q 'marker_kind' nodes/mcp_server/README.md]

#### Files

- `nodes/mcp_server/mcp_server/perception_models.py`
- `nodes/mcp_server/mcp_server/perception_tools.py`
- `nodes/mcp_server/mcp_server/approach_tools.py`
- `nodes/mcp_server/mcp_server/approach.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/tests/test_marker_pois.py`
- `nodes/mcp_server/README.md`

#### Interfaces

- `nodes/mcp_server/mcp_server/perception_models.py:PoiView` marker fields; `list_pois(marker_kind=...)`; approach `poi_id` for kind marker.

#### Expected outcome

The agent can list chargers or buttons and send the base or the arm to them.

#### Steps

- Write `tests/test_marker_pois.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_marker_pois.py -q`; see it fail.
- Implement the fields, helper, filter and approach change; rerun the verify command.
- Update the README; commit `feat(mcp_server): marker POIs in list_pois and approach targets`.

#### Notes

- Deploying mcp_server kills a running agent session (MEMORY): the runbook checks first.
- No `category`: the agent-skills epic adds it later and can derive it from `marker.kind`.

#### Ruling

Accepted by the orchestrator: verified and merged into main (marker_detector 45, poi_store 269 passed); marker-decode a4 and poi-marker-upsert a5 escalations were symbol-lookup false positives (render_marker_scene at synthetic.py:79, merge_marker_sighting at markers.py:41).

### S7: POI marker upsert

- Status: done
- Files: `nodes/poi_store/poi_store/models.py`, `nodes/poi_store/poi_store/markers.py`, `nodes/poi_store/poi_store/store.py`, `nodes/poi_store/poi_store/config.py`, `nodes/poi_store/poi_store/node.py`, `nodes/poi_store/tests/test_marker_upsert.py`, `nodes/poi_store/README.md`
- Verify: `cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: b8699a0102508b4cf8ee901b282f1c0c55d0dc43

#### Requirement

- `models.py`: `Command.op` adds `"upsert_marker"` (poi = the sighting: kind marker, x, y, z, orientation, marker, confidence).
- `config.py`: `PoiStoreConfig.markers: MarkerMergeConfig` (`relocate_m` 0.25, `relocate_deg` 20.0, `max_pose_samples` 20), defaults so existing configs load; `node.py` passes it to the store.
- `markers.py` (pure, numpy): `merge_marker_sighting(existing: Poi, sighting: Poi, cfg: MarkerMergeConfig, now: float) -> Poi` per design D12: within relocate_m and relocate_deg -> weighted mean of position and of the sign-aligned quaternion (old weight `min(pose_samples, max_pose_samples) * existing.confidence`, new weight the sighting confidence), confidence the same weighted mean, pose_samples + 1; beyond either threshold -> pose and confidence replaced, pose_samples 1; always times_seen + 1, last_seen now, marker size/params/payload/version from the sighting; name, note, status, created_by, created_at, first_seen unchanged. `angle_between_deg(q1, q2) -> float`.
- `store.py`: `op_upsert_marker(fields)`: validates the sighting as a marker Poi (error result on invalid input, nothing changed); finds the `marker` POI with the same `marker.key`; none -> add with created_by agent, name `<kind> <id>` (<= 60), times_seen 1, first_seen = last_seen = now, pose_samples 1; found -> merge and replace; commit (revision + 1, persisted) either way; the result names `added` or `updated`.
- Tests first in `nodes/poi_store/tests/test_marker_upsert.py`, seen failing: the first sighting adds one POI with the default name; ten sightings of the same key with noisy poses leave one POI whose position is the weighted mean within 1 mm and times_seen 10; a sighting 0.5 m away replaces the pose and resets pose_samples to 1; a 30 deg rotated sighting relocates, a 5 deg one averages (quaternion with flipped sign averages correctly); user-renamed name, note and status survive later sightings; two different keys give two POIs; an invalid sighting (no marker block) returns an error and changes nothing (revision unchanged); a store reloaded from disk keeps matching by key; `handle_message` with op `upsert_marker` works end to end.
- `README.md`: the `upsert_marker` op, the matching key, the merge and relocation rules and the config keys.

#### Acceptance conditions

- a1: repeated sightings of one key refine a single marker POI [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_marker_upsert.py -q]
- a2: a moved marker relocates instead of averaging [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_marker_upsert.py -q]
- a3: user-edited name, note and status survive sightings [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_marker_upsert.py -q]
- a4: an invalid sighting changes nothing [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_marker_upsert.py -q]
- a5: the merge function exists in the store package [evidence: symbol: nodes/poi_store/poi_store/markers.py:merge_marker_sighting]
- a6: the README documents upsert_marker [evidence: command: grep -q 'upsert_marker' nodes/poi_store/README.md]

#### Files

- `nodes/poi_store/poi_store/models.py`
- `nodes/poi_store/poi_store/markers.py`
- `nodes/poi_store/poi_store/store.py`
- `nodes/poi_store/poi_store/config.py`
- `nodes/poi_store/poi_store/node.py`
- `nodes/poi_store/tests/test_marker_upsert.py`
- `nodes/poi_store/README.md`

#### Interfaces

- `/poi/command` op `upsert_marker`; `nodes/poi_store/poi_store/store.py:PoiStore.op_upsert_marker`; `nodes/poi_store/poi_store/markers.py:merge_marker_sighting` (part `poi-upsert-marker-op`).

#### Expected outcome

A detector can send every sighting; poi_store keeps one refined POI per marker.

#### Steps

- Write `nodes/poi_store/tests/test_marker_upsert.py`; run `cd nodes/poi_store && uv run --frozen pytest tests/test_marker_upsert.py -q`; see it fail.
- Implement the op, merge and config; rerun; run the verify command.
- Update the README; commit `feat(poi_store): upsert_marker op with pose refinement`.

#### Notes

- Commands are handled one at a time in the node callback, so matching in the store is atomic (design D12).
- Quaternion averaging: flip a sample whose dot product with the reference is negative, weighted sum, normalise (fine for the small spreads the relocation gate allows).

#### Ruling

Accepted by the orchestrator: verified and merged into main (marker_detector 45, poi_store 269 passed); marker-decode a4 and poi-marker-upsert a5 escalations were symbol-lookup false positives (render_marker_scene at synthetic.py:79, merge_marker_sighting at markers.py:41).

### S8: Web UI marker POIs

- Status: done
- Files: `nodes/web_ui/frontend/src/poi/types.ts`, `nodes/web_ui/frontend/src/poi/style.ts`, `nodes/web_ui/frontend/src/poi/style.test.ts`, `nodes/web_ui/frontend/src/poi/PoiLayer.tsx`, `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`, `nodes/web_ui/frontend/src/poi/PoiPanels.test.tsx`, `nodes/web_ui/frontend/src/poi/PoiLegend.tsx`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`
- Verified head: 89ec8fe1d92aaffd08b01472675a9e6257fc3d4b

#### Requirement

- `types.ts`: `PoiKind` adds `'marker'`; `Poi` gains `z?`, `orientation?: [number, number, number, number] | null`, `marker?: MarkerInfo | null` (kind, id, key, size_mm, params, payload, version, pose_samples); `isPointLike('marker')` true.
- `style.ts`: `KIND_SHAPE.marker = 'hexagon'` (new MarkerShape with 6 sides), its colour, `facingYaw(poi)` (yaw of +z projected on the floor, null when within 45 deg of vertical; same rule as mcp_server); the kind filter at line ~153 accepts markers.
- `PoiLayer.tsx`: markers drawn with the hexagon and a facing arrow when facingYaw is not null; not draggable.
- `PoiPanels.tsx`: a marker's panel shows marker kind, id, size (mm), height z (m), times seen, last seen, confidence; name, note and status editable; delete allowed; no position or radius editor.
- `PoiLegend.tsx`: a Marker legend entry.
- Tests first (`style.test.ts`, `PoiPanels.test.tsx`), seen failing: KIND_SHAPE and sides for marker; facingYaw for a wall marker facing +y (90 deg) and null for a floor marker; the marker panel renders the marker fields and has no radius/position inputs; editing the name sends only name; the legend lists Marker.
- `nodes/web_ui/README.md`: marker POIs on the map and in the panels.

#### Acceptance conditions

- a1: marker POIs have their own shape and a facing direction [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/style.test.ts]
- a2: the marker panel shows marker details, edits name, note and status only [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/poi/PoiPanels.test.tsx]
- a3: the frontend typechecks, lints and builds [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]
- a4: the web_ui README mentions marker POIs [evidence: command: grep -qi 'marker POI' nodes/web_ui/README.md]

#### Files

- `nodes/web_ui/frontend/src/poi/types.ts`
- `nodes/web_ui/frontend/src/poi/style.ts`
- `nodes/web_ui/frontend/src/poi/style.test.ts`
- `nodes/web_ui/frontend/src/poi/PoiLayer.tsx`
- `nodes/web_ui/frontend/src/poi/PoiPanels.tsx`
- `nodes/web_ui/frontend/src/poi/PoiPanels.test.tsx`
- `nodes/web_ui/frontend/src/poi/PoiLegend.tsx`
- `nodes/web_ui/README.md`

#### Interfaces

- `nodes/web_ui/frontend/src/poi/types.ts:Poi` marker fields; `nodes/web_ui/frontend/src/poi/style.ts:facingYaw`.

#### Expected outcome

Markers show on the web UI map with facing arrows and details.

#### Steps

- Write the tests; run `cd nodes/web_ui/frontend && npx vitest run src/poi/style.test.ts src/poi/PoiPanels.test.tsx`; see them fail.
- Implement types, style, layer, panels, legend; run the verify command.
- Update the README; commit `feat(web_ui): show marker POIs`.

#### Notes

- The UI never moves a marker: the detector owns its pose (design D13).

#### Ruling

Accepted by the orchestrator: verified and merged into main (marker_detector 45, poi_store 269 passed); marker-decode a4 and poi-marker-upsert a5 escalations were symbol-lookup false positives (render_marker_scene at synthetic.py:79, merge_marker_sighting at markers.py:41).

### S9: Marker pose IPPE

- Status: done
- Files: `nodes/marker_detector/marker_detector/pose.py`, `nodes/marker_detector/tests/test_pose.py`
- Verify: `cd nodes/marker_detector && uv run --frozen pytest tests/test_pose.py -q && uv run --frozen poe lint`
- Verified head: fb6ce92f5545e3cda254ca4865bbb50e14b3f0dd

#### Requirement

- `nodes/marker_detector/marker_detector/pose.py` (design D9):
  - `finder_model_points(size_mm: float, version: int) -> np.ndarray` (24x3, metres, marker frame of design D3: z = 0 plane, origin at the symbol centre, +x right, +y up): for each of the three finders (top-left, top-right, bottom-left) the four corners of the outer 7x7 square (module offsets 0 and 7) and of the inner 3x3 square (2 and 5), in a fixed documented order.
  - `predict_image_points(corners: np.ndarray, version: int) -> np.ndarray` (24x2) from the homography of the four symbol corners to module coordinates.
  - `refine_corners(grey, points, module_px) -> np.ndarray`: `cv2.cornerSubPix` with a window of about module_px/2 (at least 2 px), termination 30 iterations / 0.01 px.
  - `estimate_pose(grey, candidate: Candidate, K: np.ndarray, D: np.ndarray, cfg: PoseConfig, choose: Callable[[list[Solution]], Solution | None] | None = None) -> PoseResult`: `cv2.solvePnPGeneric(model, image, K, D, flags=cv2.SOLVEPNP_IPPE)`, each solution refined with `cv2.solvePnPRefineLM`, RMS reprojection error per solution; accept the best when `rms2 / rms1 >= cfg.ambiguity_ratio`; otherwise, when `choose` is given (the ToF task supplies `plane_chooser`), accept the solution it returns; otherwise reject. `PoseResult`: `ok`, `reason` (`ambiguous`, `rms`, `module_too_small`, `behind_camera`, `pnp_failed`), `rvec`, `tvec`, `rms_px`, `ratio`, `confidence` (in (0, 1]: product of terms from rms/max_rms, ratio and module_px, documented), `solutions` (both, for the chooser and the report).
- Tests first in `nodes/marker_detector/tests/test_pose.py` (synthetic scenes from `synthetic.py`, ground truth from the renderer's pose, never from the code under test), seen failing:
  - accuracy: 150 mm marker, STREAM_K, 0.3 px noise, distances 0.5/1.0/2.0 m and yaw 0/30/50 deg: distance error < 1 % of distance and normal error < 2 deg on all, over 5 seeds;
  - size scaling: the same image decoded with S declared 75 mm halves the distance (S drives the scale, design D3);
  - roll 0/90/180/270 deg gives the right in-plane rotation (no 90 deg ambiguity);
  - ambiguity: a small marker seen nearly frontally at long range with 1 px noise (ratio below threshold) is rejected `ambiguous` without a chooser, and accepted when a chooser picks the true solution;
  - rejections: module below `min_module_px` -> `module_too_small`, RMS above `max_rms_px` (corrupted corners) -> `rms`;
  - `finder_model_points` geometry: 24 distinct coplanar points, outer finder corners at +-S/2.

#### Acceptance conditions

- a1: synthetic poses are recovered within 1 percent distance and 2 degrees normal error from 0.5 to 2 m [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_pose.py -q]
- a2: the declared side length sets the scale and roll has no 90 degree ambiguity [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_pose.py -q]
- a3: ambiguous solutions are rejected unless a chooser settles them [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_pose.py -q]
- a4: small modules and high reprojection error are rejected with reasons [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_pose.py -q]

#### Files

- `nodes/marker_detector/marker_detector/pose.py`
- `nodes/marker_detector/tests/test_pose.py`

#### Interfaces

- `nodes/marker_detector/marker_detector/pose.py`: `finder_model_points`, `predict_image_points`, `refine_corners`, `estimate_pose`, `PoseResult`, `Solution` (part `marker-pose`).

#### Expected outcome

Each candidate yields an accepted pose with confidence, or a named rejection.

#### Steps

- Write `tests/test_pose.py`; run `cd nodes/marker_detector && uv run --frozen pytest tests/test_pose.py -q`; see it fail.
- Implement pose.py; rerun until green; `uv run --frozen poe lint`.
- Commit `feat(marker_detector): marker pose from refined finder corners with IPPE`.

#### Notes

- `SOLVEPNP_IPPE` (not `IPPE_SQUARE`) takes any >= 4 coplanar points and returns both solutions via solvePnPGeneric (OpenCV docs).
- The ambiguity is the classic planar flip; it grows with distance and frontal views, which is what the reject path and the ToF chooser handle.

- 2026-10-19 run printable-metadata-markers-t: dependency-failed on marker-decode

#### Ruling

Accepted by the orchestrator: verified and merged into main (marker_detector 150 passed); symbol-AC escalations were verifier false positives (the symbols exist: render_marker_scene, merge_marker_sighting, run_node at node.py:165).

### S10: Marker map pose POI

- Status: done
- Files: `nodes/marker_detector/marker_detector/geometry.py`, `nodes/marker_detector/tests/test_geometry.py`
- Verify: `cd nodes/marker_detector && uv run --frozen pytest tests/test_geometry.py -q && uv run --frozen poe lint`
- Verified head: 8cc044280756b1b35870c9a3891863057b2f5cd7

#### Requirement

- `nodes/marker_detector/marker_detector/geometry.py` (scipy Rotation for rotations):
  - `marker_in_map(rvec, tvec, map_from_camera: tuple[np.ndarray, np.ndarray]) -> tuple[np.ndarray, np.ndarray]`: compose map <- camera optical (translation, quaternion xyzw from TF) with camera <- marker; returns position (m) and quaternion xyzw.
  - `zoom_intrinsics(stream_K: np.ndarray, stream_size: tuple[int, int], full_size: tuple[int, int], crop: tuple[int, int, int, int], out_size: tuple[int, int]) -> np.ndarray`: scale K from the stream size to the full sensor size, shift the principal point by the crop origin, scale by out/crop (front camera epic D4/D5 contract).
  - `zoom_request(u: float, v: float, module_px: float, cfg: ZoomConfig, request_id: str) -> dict` and `zoom_factor(module_px, cfg) -> int` (smallest of 2/4/8 reaching `target_module_px`, else 8).
  - `sighting_command(candidate: Candidate, pose: PoseResult, position, quaternion, request_id: str) -> dict`: the `/poi/command` JSON `{"op": "upsert_marker", "request_id": ..., "poi": {"kind": "marker", "x", "y", "z", "orientation": [qx, qy, qz, qw], "marker": MarkerInfo dump (from `marker_info(candidate.payload)`), "confidence"}}` (contract owned by poi_store's `upsert_marker`).
  - `detection_report(stamp_ns: int, source: str, items: list[dict]) -> str`: the `/markers/detections` JSON (stamp, source `stream` or `zoom`, per marker key, payload, position, quaternion, rms_px, ratio, confidence, module_px, chooser `reprojection`/`tof`).
- Tests first in `nodes/marker_detector/tests/test_geometry.py`, seen failing: identity TF returns the camera pose unchanged; a known TF (translation + 90 deg yaw) maps a marker 1 m ahead in the optical frame to the expected map position and orientation; `zoom_intrinsics` for zoom 4 centred crop matches a hand-computed K (fx x4 then /1.125 for 1152->1024 out) and a projected test point lands at the same pixel as cropping the full-res projection; zoom factor choices; the sighting command validates as a poi_store-compatible dict (shape checked against the documented contract: keys, quaternion norm 1, marker block fields of `MarkerInfo`); the report JSON round-trips.

#### Acceptance conditions

- a1: camera-frame marker poses are composed into map poses through a TF transform [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_geometry.py -q]
- a2: zoom crop intrinsics follow the crop and output scaling [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_geometry.py -q]
- a3: a sighting becomes an upsert_marker command with MarkerInfo [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_geometry.py -q]

#### Files

- `nodes/marker_detector/marker_detector/geometry.py`
- `nodes/marker_detector/tests/test_geometry.py`

#### Interfaces

- `nodes/marker_detector/marker_detector/geometry.py`: `marker_in_map`, `zoom_intrinsics`, `zoom_factor`, `zoom_request`, `sighting_command`, `detection_report` (part `marker-geometry`).

#### Expected outcome

Pure functions turn a camera-frame pose into the POI command and the detection report.

#### Steps

- Write `tests/test_geometry.py`; run `cd nodes/marker_detector && uv run --frozen pytest tests/test_geometry.py -q`; see it fail.
- Implement geometry.py; rerun until green; `uv run --frozen poe lint`.
- Commit `feat(marker_detector): map pose, zoom intrinsics and POI sighting`.

#### Notes

- The zoom command fields follow the front camera epic (D4): request_id, u, v (normalised, top-left origin of the default view), zoom 2/4/8, max_px, af.

- `marker-payload` (`shared/ros2_common/marker_payload.py`: `decode_payload`, `MarkerInfo`, `marker_info`) is delivered by the blocker epic marker-and-calibration-board; it exists before this task runs.

- 2026-10-19 run printable-metadata-markers-t: dependency-failed on marker-pose-ippe, poi-marker-upsert

#### Ruling

Accepted by the orchestrator: verified and merged into main (marker_detector 150 passed); symbol-AC escalations were verifier false positives (the symbols exist: render_marker_scene, merge_marker_sighting, run_node at node.py:165).

### S11: Marker ToF plane

- Status: done
- Files: `nodes/marker_detector/marker_detector/tof_plane.py`, `nodes/marker_detector/tests/test_tof_plane.py`
- Verify: `cd nodes/marker_detector && uv run --frozen pytest tests/test_tof_plane.py -q && uv run --frozen poe lint`
- Verified head: ed15508879f54532ebc6142075a653efc365f32c

#### Requirement

- `nodes/marker_detector/marker_detector/tof_plane.py` (design D10b):
  - `fit_plane(depth_mm: np.ndarray, depth_K: np.ndarray, polygon_px: np.ndarray, image_size: tuple[int, int], cfg: TofConfig) -> Plane | None`: scale the marker polygon from the RGB image size to the depth image size, back-project the valid (non-zero) depth pixels inside it with depth_K, least-squares plane by SVD; None (with a reason) when fewer than `min_valid_px` valid pixels or the RMS residual exceeds `max_residual_mm`. `Plane`: unit normal oriented towards the camera, distance, residual_mm, n_px.
  - `plane_chooser(plane: Plane, cfg: TofConfig) -> Callable[[list[Solution]], Solution | None]`: returns the solution whose marker +z axis (camera frame) is within `max_angle_deg` of the plane normal when exactly one is; else None.
  - `nearest_depth(frames: Sequence[tuple[int, np.ndarray]], stamp_ns: int, max_dt_s: float)`: the depth frame nearest in time within the limit, else None.
- Tests first in `nodes/marker_detector/tests/test_tof_plane.py`, seen failing: a synthetic depth plane (288x162, mm) tilted 35 deg gives the normal within 1 deg; zeros inside the polygon are ignored; too few valid pixels and a non-planar patch (step) return None; given an ambiguous synthetic pair of IPPE solutions (from pose.py on a synthetic scene) and the matching depth plane, the chooser returns the true one and with a plane 90 deg off returns None; `nearest_depth` respects max_dt_s.

#### Acceptance conditions

- a1: the plane normal is recovered from registered depth inside the marker polygon [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_tof_plane.py -q]
- a2: sparse or non-planar depth yields no plane [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_tof_plane.py -q]
- a3: the chooser picks the IPPE solution matching the ToF normal and refuses when none or both match [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_tof_plane.py -q]

#### Files

- `nodes/marker_detector/marker_detector/tof_plane.py`
- `nodes/marker_detector/tests/test_tof_plane.py`

#### Interfaces

- `nodes/marker_detector/marker_detector/tof_plane.py`: `Plane`, `fit_plane`, `plane_chooser`, `nearest_depth` (part `marker-tof-plane`).

#### Expected outcome

Ambiguous poses with ToF coverage are resolved; others stay rejected.

#### Steps

- Write `tests/test_tof_plane.py`; run `cd nodes/marker_detector && uv run --frozen pytest tests/test_tof_plane.py -q`; see it fail.
- Implement tof_plane.py; rerun until green; `uv run --frozen poe lint`.
- Commit `feat(marker_detector): ToF plane breaks pose ambiguity`.

#### Notes

- `/head_rgbd/depth/image_raw` is 16UC1 mm in `head_camera_optical_frame` (fusion epic D1); its camera_info gives depth_K at 288x162.
- Normal sign: the marker's +z points out of the face, i.e. towards the camera; keep one convention in both modules (review focus).

- 2026-10-19 run printable-metadata-markers-t: dependency-failed on marker-pose-ippe

#### Ruling

Accepted by the orchestrator: verified and merged into main (marker_detector 150 passed); symbol-AC escalations were verifier false positives (the symbols exist: render_marker_scene, merge_marker_sighting, run_node at node.py:165).

### S12: Marker detector node

- Status: done
- Files: `nodes/marker_detector/marker_detector/pipeline.py`, `nodes/marker_detector/marker_detector/node.py`, `nodes/marker_detector/marker_detector/__main__.py`, `nodes/marker_detector/marker_detector/metrics.py`, `nodes/marker_detector/tests/ros_stubs.py`, `nodes/marker_detector/tests/test_pipeline.py`, `nodes/marker_detector/README.md`
- Verify: `cd nodes/marker_detector && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: 37c6fc88ebb23de8ea8b1d8586016b2da1bcf01b

#### Requirement

- `nodes/marker_detector/marker_detector/pipeline.py` (no ROS): `MarkerPipeline(cfg, clock)` with `on_camera_info(K, D, size, frame_id)`, `on_depth(stamp_ns, depth_mm, depth_K)`, `process_frame(stamp_ns, jpeg: bytes, source: str, lookup_tf: Callable[[str, int], tuple | None], K_override=None) -> FrameOutput` (decode -> pose (with the ToF chooser when an ambiguous pose has a depth frame within `tof.max_dt_s` and `tof.enabled`) -> TF (map <- camera frame at the image stamp) -> `min_confidence` gate -> commands, report and zoom proposals) and `process_zoom(frame: ZoomFrame, result: dict, lookup_tf)` (zoom intrinsics from the stored stream camera_info). `FrameOutput`: `commands` (upsert dicts), `report` (str or None), `zoom_request` (dict or None), `rejections` (reason counts). Placeholder rules: no camera_info -> nothing processed (one log line per state change); undecodable or invalid payload, ambiguous or failed pose, TF lookup failure (None) -> no command; a report only when at least one detection was accepted.
- `nodes/marker_detector/marker_detector/node.py` (rclpy, thin): subscriptions (image CompressedImage keep_last 1 best effort, camera_info, depth Image + camera_info when `tof.enabled`, zoom image/result when `zoom.process_stills` or `zoom.auto_zoom`), a timer at `process_hz` that processes only the newest unprocessed frame, `tf2_ros.Buffer`/`TransformListener` lookup at the image stamp with `tf_timeout_s`, publishers `/poi/command` (String JSON, reliable keep_last 20) and `/markers/detections` (String), zoom command publisher; metrics server via `ros2_metrics`. `__main__.py` loads `config_path()`. `metrics.py`: counters for frames processed, detections accepted, rejections by reason, zoom requests by status, and a processing-time histogram.
- Tests first in `nodes/marker_detector/tests/test_pipeline.py` (synthetic scenes; `tests/ros_stubs.py` stubs rclpy, tf2_ros, sensor_msgs, std_msgs for a node import smoke test), seen failing: a frame with one 150 mm marker at 1 m and identity TF gives exactly one `upsert_marker` command with the right key, map pose within 1 % and a report; without camera_info nothing is produced; with a TF lookup returning None nothing is produced; an invalid payload QR produces no command and a `payload` rejection; an ambiguous far marker produces no command without depth and one command with a matching synthetic depth plane (chooser `tof`); a 60 mm marker at 2.5 m (module < 6 px) yields no zoom request with the default `auto_zoom: false`, and an externally requested synthetic zoom still of it (result plus image, rendered with the crop intrinsics) yields one command whose pose matches within 2 %; with `auto_zoom: true` the same frame yields a zoom request and its still yields the command; confidence below `min_confidence` sends nothing; the node module imports with the stubs and wires the configured topics.
- `nodes/marker_detector/README.md`: pipeline, placeholder rules, topics and QoS, the report format, metrics, and the dependency on calibrated camera_info, head TF and (optional) `/head_rgbd/depth`.

#### Acceptance conditions

- a1: a synthetic detection produces exactly one upsert_marker command and a report [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_pipeline.py -q]
- a2: no camera_info, no TF, an invalid payload, an ambiguous pose or low confidence produce nothing [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_pipeline.py -q]
- a3: an ambiguous pose with matching ToF depth is accepted via the plane chooser [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_pipeline.py -q]
- a4: zoom stills (external by default, own requests only with auto_zoom on) produce the POI of a small distant marker [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_pipeline.py -q]
- a5: the node entry wires the configured topics [evidence: symbol: nodes/marker_detector/marker_detector/node.py:run_node]
- a6: the README documents topics, placeholder rules and dependencies [evidence: command: grep -q '/markers/detections' nodes/marker_detector/README.md && grep -q 'camera_info' nodes/marker_detector/README.md]

#### Files

- `nodes/marker_detector/marker_detector/pipeline.py`
- `nodes/marker_detector/marker_detector/node.py`
- `nodes/marker_detector/marker_detector/__main__.py`
- `nodes/marker_detector/marker_detector/metrics.py`
- `nodes/marker_detector/tests/ros_stubs.py`
- `nodes/marker_detector/tests/test_pipeline.py`
- `nodes/marker_detector/README.md`

#### Interfaces

- `nodes/marker_detector/marker_detector/pipeline.py:MarkerPipeline`, `nodes/marker_detector/marker_detector/node.py:run_node`, `python3 -m marker_detector` (part `marker-detector-node-entry`); topics `/poi/command` (op `upsert_marker`), `/markers/detections`, `/head_camera/zoom/command`.

#### Expected outcome

`python3 -m marker_detector` turns head camera sightings into POI upserts.

#### Steps

- Write `tests/test_pipeline.py` and the stubs; run `cd nodes/marker_detector && uv run --frozen pytest tests/test_pipeline.py -q`; see it fail.
- Implement pipeline.py, node.py, __main__.py, metrics.py; rerun the whole node suite; `uv run --frozen poe lint`.
- Update the README; commit `feat(marker_detector): detection node publishing marker POI sightings`.

#### Notes

- numpy dependency guard (CLAUDE.md): numpy is a direct dependency already; the deploy check in the runbook confirms no ModuleNotFoundError.
- Pattern for a ROS-free core plus a thin node: nodes/poi_store (store.py vs node.py) and its tests/ros_stubs.py.

- 2026-10-19 run printable-metadata-markers-t: dependency-failed on marker-tof-plane, marker-map-pose-poi

#### Ruling

Accepted by the orchestrator: verified and merged into main (marker_detector 150 passed); symbol-AC escalations were verifier false positives (the symbols exist: render_marker_scene, merge_marker_sighting, run_node at node.py:165).

### S13: Marker detector Ansible

- Status: done
- Files: `ansible/group_vars/client.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `ansible/README.md`, `.claude/skills/ansible-deploy/SKILL.md`, `tests/test_marker_detector_config.py`, `tests/test_node_metrics_wiring.py`, `nodes/README.md`
- Verify: `uv run pytest tests/test_marker_detector_config.py tests/test_node_metrics_wiring.py tests/test_node_lint_coverage.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: 8b50b398d578788bdbd7c24860c9977f01dbc390

#### Requirement

- `ansible/group_vars/client.yml`: `ros2_node_type_defaults.marker_detector` (native uv node: `node_launch_command: "python3 -m marker_detector"`, `node_src_dir: nodes/marker_detector`, `config_path: /etc/ros2/marker_detector`, env `MARKER_DETECTOR_CONFIG=/etc/ros2/marker_detector/config.yaml`, src_shared as other ros2-common users, CPU weight/memory protection like the other robot nodes, MemoryMax generous); a `ros2_nodes` entry `marker_detector` (present, enabled) after `poi_store` with a `config` block holding the defaults of design D8-D10 and `metrics_port` (first free port per tests/test_node_metrics_wiring.py); `monitoring_agent_node_targets` entry for it.
- `ansible/playbooks/deploy_nodes_client.yml`: `Deploy marker_detector` after `Deploy poi_store`, tags `[marker_detector, apt, python, build, config]` (pattern: Deploy costmap_zones).
- `tests/test_marker_detector_config.py` (pattern tests/test_costmap_zones_config.py), first, seen failing: node type defaults, entry order and config (loaded with `MarkerDetectorConfig` from the node package via sys.path), metrics target, playbook order and tags, node files exist, docs mention the node and its port.
- `tests/test_node_metrics_wiring.py`: the new port in `CONFIG_METRICS_PORTS`.
- `ansible/README.md` (node table with port), `.claude/skills/ansible-deploy/SKILL.md` (node list), `nodes/README.md` (node index line).

#### Acceptance conditions

- a1: the Ansible node type, enabled entry, playbook step and metrics target exist and the config validates [evidence: test: tests/test_marker_detector_config.py]
- a2: the metrics port is wired [evidence: test: tests/test_node_metrics_wiring.py]
- a3: Ansible lint and syntax checks pass [evidence: command: uv run poe test-ansible]
- a4: docs list the node [evidence: command: grep -q marker_detector ansible/README.md && grep -q marker_detector nodes/README.md && grep -q marker_detector .claude/skills/ansible-deploy/SKILL.md]

#### Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `ansible/README.md`
- `.claude/skills/ansible-deploy/SKILL.md`
- `tests/test_marker_detector_config.py`
- `tests/test_node_metrics_wiring.py`
- `nodes/README.md`

#### Interfaces

- `ros2_nodes` entry `marker_detector` (schema `name`, `node_type`, `present`, `enabled`, `config`); deploy tag `marker_detector`.

#### Expected outcome

`./scripts/deploy-nodes.sh client marker_detector` deploys the detector.

#### Steps

- Write `tests/test_marker_detector_config.py`; run `uv run pytest tests/test_marker_detector_config.py -q`; see it fail.
- Add the Ansible pieces and docs; rerun; run the verify command.
- Commit `feat(ansible): deploy marker_detector on the client`.

#### Notes

- Do not deploy in this task; the runbook deploys through scripts/deploy-nodes.sh after invoking the ansible-deploy skill.

- 2026-10-19 run printable-metadata-markers-t: dependency-failed on marker-detector-node

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1691 passed after the tier/order fix; marker_detector 154 passed); user-doc parking flags concern the design's rejected AprilTag alternative.

### S14: Marker detector bench

- Status: done
- Files: `nodes/marker_detector/marker_detector/bench.py`, `nodes/marker_detector/config/bench.yaml`, `nodes/marker_detector/tests/test_bench.py`, `scripts/marker_bench.sh`, `scripts/README.md`, `nodes/marker_detector/README.md`
- Verify: `cd nodes/marker_detector && uv run --frozen pytest tests/test_bench.py -q && uv run --frozen poe lint && cd ../.. && bash -n scripts/marker_bench.sh`
- Verified head: 7152020dafa44887d33f1b158ff746bb9410be07

#### Requirement

- `nodes/marker_detector/marker_detector/bench.py`: `python -m marker_detector.bench [config]` (config file `nodes/marker_detector/config/bench.yaml`, pydantic: `iterations` 200, `warmup` 20, `scenes` list of {`image_size` [1152, 648] or [576, 324] or [1024, 576] (zoom crop), `markers` 1-3 with sizes and distances}, `seed`): renders the scenes once with `synthetic.py`, then times per frame `decode_frame`, `estimate_pose` per candidate and the whole frame (`time.perf_counter_ns`), and prints JSON `{scene: {stage: {p50_ms, p95_ms, max_ms}}, "decoded_ratio": ..., "cv2": version, "zxingcpp": version, "machine": platform}`; `run_bench(cfg) -> dict` for tests.
- `scripts/marker_bench.sh`: runs the bench on the client in the deployed node venv over ssh (host and user from the ansible inventory, pattern `scripts/rtk_diag.sh`), copies the JSON to `.logs/marker_bench.json`, prints the summary; `--help` text.
- Tests first in `nodes/marker_detector/tests/test_bench.py`, seen failing: `run_bench` with 3 iterations on two scenes returns every stage with p50 <= p95 <= max, decoded_ratio 1.0 for the near scenes, and the versions.
- `scripts/README.md` gains a `marker_bench.sh` entry; the node README gains a Benchmark section (how to run, the budget of design D14, where results go).

#### Acceptance conditions

- a1: the bench reports p50 and p95 per stage and decode success on synthetic frames [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_bench.py -q]
- a2: the robot bench script is valid shell and documented [evidence: command: bash -n scripts/marker_bench.sh && grep -q 'marker_bench.sh' scripts/README.md]

#### Files

- `nodes/marker_detector/marker_detector/bench.py`
- `nodes/marker_detector/config/bench.yaml`
- `nodes/marker_detector/tests/test_bench.py`
- `scripts/marker_bench.sh`
- `scripts/README.md`
- `nodes/marker_detector/README.md`

#### Interfaces

- `python -m marker_detector.bench`, `nodes/marker_detector/marker_detector/bench.py:run_bench`, `scripts/marker_bench.sh`.

#### Expected outcome

One script measures decode and pose cost on the Pi 5 and stores the JSON in `.logs/`.

#### Steps

- Write `tests/test_bench.py`; run `cd nodes/marker_detector && uv run --frozen pytest tests/test_bench.py -q`; see it fail.
- Implement bench.py, bench.yaml and the script; rerun; run the verify command.
- Update both READMEs; commit `feat(marker_detector): decode and pose benchmark`.

#### Notes

- Do not run the script against the robot in this task; the runbook does.

- 2026-10-19 run printable-metadata-markers-t: dependency-failed on marker-detector-node

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1691 passed after the tier/order fix; marker_detector 154 passed); user-doc parking flags concern the design's rejected AprilTag alternative.

### S15: Markers user doc

- Status: done
- Files: `docs/markers.md`, `tests/README.md`, `MEMORY.md`
- Verify: `test -f docs/markers.md && grep -q '^## Payload format' docs/markers.md && grep -q 'test_marker_detector_config' tests/README.md && uv run poe lint`
- Verified head: 5c0334d34a99904ccabefbf2535b9eeead588ccc

#### Requirement

- `docs/markers.md` for a mid-level ROS 2 developer, with these sections: `## Payload format` (grammar, kinds table, version rule, examples, capacity per version/ECC), `## Size and frame` (S definition, module size, marker axes, POI position and orientation), `## Generating markers` (pointer to the marker-and-calibration-board generator: config, command, outputs, PDF printing at 100 % on a rigid plate, two-colour Bambu AMS printing from the two STLs or the 3MF, minimum module edge, recommended sizes: 150-200 mm for docking range, 60 mm for buttons near the arm), `## Calibration board` (the ChArUco spec, why 6x5 x 40 mm, used by scripts/head_rgbd_calibrate.py), `## Detection` (stream and zoom paths, pose pipeline, ambiguity and ToF, placeholder rules, `/markers/detections`), `## POIs` (kind marker, upsert and relocation rules, agent and web UI use), `## Range and fallback` (the computed range table: 100 mm v3 decodes to about 0.75 m in the stream and 3 m via zoom at 4 px/module; AprilTag tagStandard41h12 via apriltag_ros as the documented fallback if field range is insufficient).
- `tests/README.md`: every root test file this epic added or changed (test_marker_detector_config.py, test_ci_workflow.py, test_node_metrics_wiring.py; the generator's tests are listed by marker-and-calibration-board) with what each covers.
- `MEMORY.md`: a decision entry (QR Model 2 with P1 payload over AprilTag/ArUco, why; marker POIs in poi_store via upsert_marker; ChArUco spec owned by the generator).

#### Acceptance conditions

- a1: the user doc covers payload, size and frame, generation, calibration board, detection, POIs and range [evidence: doc-section: docs/markers.md#Payload format]
- a2: the doc states the AprilTag fallback and the range table [evidence: doc-section: docs/markers.md#Range and fallback]
- a3: tests README lists every new root test file [evidence: command: grep -q test_marker_detector_config tests/README.md]
- a4: MEMORY records the marker decisions [evidence: command: grep -q 'upsert_marker' MEMORY.md]

#### Files

- `docs/markers.md`
- `tests/README.md`
- `MEMORY.md`

#### Interfaces

- `docs/markers.md` sections as listed.

#### Expected outcome

The marker system is documented end to end.

#### Steps

- Write docs/markers.md from design D1-D14 and the delivered code; update tests/README.md and MEMORY.md; run the verify command.
- Commit `docs: printable metadata markers`.

#### Notes

- Documentation-only task (TDD exception: doc edit); keep it in sync with the actual defaults in the code.

- 2026-10-19 run printable-metadata-markers-t: dependency-failed on poi-marker-upsert, marker-detector-bench, marker-detector-ansible

- 2026-10-19 run printable-metadata-markers-t: dependency-failed on marker-detector-bench, marker-detector-ansible

#### Ruling

Accepted by the orchestrator: verified and merged into main (root 1691 passed after the tier/order fix; marker_detector 154 passed); user-doc parking flags concern the design's rejected AprilTag alternative.

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none
