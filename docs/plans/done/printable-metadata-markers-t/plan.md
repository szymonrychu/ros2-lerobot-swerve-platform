---
title: Printable metadata markers that become POIs
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-18
owner: Szymon Richert
rung: 1
---

# Printable metadata markers that become POIs

## What and why

What: Printed QR metadata markers seen by the head camera become POIs with pose and metadata
How: Detector node (decode, pose, ToF disambiguation, zoom stills, TF), poi_store marker kind and upsert, agent and web UI use
Why: Fixed places (base, charger, drop-off, buttons, tool station) become reliable, self-describing POIs

Printable square black/white markers (QR or another square fiducial) that carry their own metadata: the POI they stand for (kind, name, optional parameters) and their physical side length. The format, the codec and the generator (PDF and two-colour STL/3MF, plus the ChArUco calibration board) are the separate epic marker-and-calibration-board, which this epic builds on. When the robot's camera reads a marker, its pose (position and orientation, computed from the known size) and metadata become a POI in the POI store automatically. Examples: base, charger, drop-off container, button/switch marker, tool change station.

## Scope ladder

### Rung 1: agreed scope (migrated)

- 1.1: Detection on the robot: a node (or mcp_server/perception path) that finds markers in the head camera image (full-res zoom capture where needed), decodes metadata, estimates the 6-DoF pose from the known size and camera intrinsics, transforms it into the map frame via TF, and creates/updates the matching POI (kind/category, pose, metadata, last-seen time, confidence); repeated sightings refine, not duplicate; nothing published without real detections.
- 1.2: The POI kinds include base, charger, drop-off container, button/switch, tool change station (extensible); the agent can list marker POIs and use them as targets.
- 1.3: READMEs, tests, deployment of the detection part; verified on the robot with printed markers at known poses.

## Agreed scope

Rung 1, migrated from the epic format: the epic's goals were its agreed scope.

## Non-goals

- The payload format, codec and generator, and the calibration board output (epic marker-and-calibration-board).
- Printing the markers (done by the user).
- Coloured or non-square markers.

## Design

Migrated from the epic format; the design sections without a plan.md heading are under Migrated notes.

## Decisions

D1, D2 and D4-D7 (symbology, payload codec, generator, PDF, STL/3MF, ChArUco board) moved with the generator to
epic marker-and-calibration-board (a blocker of this epic); ids here are kept stable. This epic uses its codec
(`shared/ros2_common/marker_payload.py`: `decode_payload`, `MarkerInfo`) and its printed markers.

### D3: Side length and marker frame

`S` is the symbol edge from the outer edge of one finder to the outer edge of the opposite side, excluding the quiet
zone; module edge = S / (17 + 4 * version). Marker frame: origin at the symbol centre, +x along the top edge (the edge
holding the two upper finders, left to right as read), +y along the left edge towards the top, +z out of the printed
face towards a viewer. The POI position is the origin, its orientation this frame in `map`. (my call) Serves U2, U3, G3.

### D8: Detector: a Python node `nodes/marker_detector` on the client

Own uv venv (`opencv-python-headless>=4.9,<5`, `zxing-cpp>=2.2,<4`, segno, numpy, scipy, pydantic, pyyaml,
ros2-common, ros2-metrics), config file `/etc/ros2/marker_detector/config.yaml` via the `ros2_nodes` schema.
Subscribes `/head_camera/image_raw/compressed` (1152x648) with `/head_camera/camera_info`, processes at most
`process_hz` (default 2) frames per second (the newest; older ones dropped), decodes with zxing-cpp
(`read_barcodes`, QR only) on the grey image. No camera_info (uncalibrated camera) -> nothing processed, logged once.
Outputs: `upsert_marker` commands on `/poi/command` and a JSON report on `/markers/detections` (std_msgs/String, only
for frames with at least one accepted detection). Alternatives: inside mcp_server (agent-coupled, mcp_server stays
lean), C++ (apt OpenCV 4.6 lacks the needed detectors, zxing-cpp's C++ build would need a new Ansible build path; the
measured cost decides, see D14). (my call) Serves U3, G3.

### D9: Pose: 24 refined finder corners, IPPE, LM refinement, ambiguity check

Per decoded symbol: the homography from zxing-cpp's four symbol corners (ordered top-left, top-right, bottom-right,
bottom-left in the symbol's own orientation) to module coordinates predicts the 24 finder corners (outer 7x7 edge at
modules 0/7 and inner 3x3 square at 2/5, four corners each, three finders); `cv2.cornerSubPix` refines them (window
from the module size in pixels); `cv2.solvePnPGeneric(..., SOLVEPNP_IPPE)` on the 24 planar points returns both
solutions with reprojection errors; each is refined with `cv2.solvePnPRefineLM`. Accept the best when the second
solution's RMS divided by the best's is at least `ambiguity_ratio` (default 2.0); otherwise use the ToF plane (D10b);
otherwise reject (`ambiguous`). Also rejected: best RMS above `max_rms_px` (1.0), module below `pose_min_module_px`
(4 px), marker behind the camera. Confidence in (0, 1] from RMS, ratio and module size. Synthetic target: < 1 %
distance error and < 2 deg normal error at 0.5-2 m with 0.3 px noise. Serves U3, G3.

### D10: Zoom stills for small or distant markers and the ToF plane for ambiguity

(a) Zoom: the node processes every zoom still it sees (`zoom.process_stills` true): stills requested by the web UI
click-to-zoom, the agent's `get_camera_image` zoom or the node itself, pairing `/head_camera/zoom/image` with
`/head_camera/zoom/result` by `stamp_ns` and running D9 with intrinsics derived from the stream camera_info (scaled to
4608x2592, shifted by the crop origin, scaled by out/crop). Automatic requests are behind `zoom.auto_zoom`, default
false until the user decides (gate question): when true and a decoded symbol's module is below `zoom.min_module_px`
(6 px) or OpenCV `QRCodeDetectorAruco` finds a QR quad zxing-cpp could not decode, the node requests one zoom still on
`/head_camera/zoom/command` (front camera epic contract: `request_id`, normalised `u`, `v` of the quad centre, `zoom`
2/4/8 chosen so the module reaches `zoom.target_module_px` (12), `max_px` 1024, `af` true), rate limited to one per
`zoom.min_interval_s` (10 s), `zoom.cooldown_s` 30 per image region, one outstanding request. A result other than
`ok`/`af_failed` produces nothing. (b) ToF plane:
with `tof.enabled`, the registered depth `/head_rgbd/depth/image_raw` (16UC1 mm, `head_camera_optical_frame`) nearest
in time (<= `tof.max_dt_s` 0.15) gives a least-squares plane inside the marker polygon (scaled to the depth size,
>= 30 valid pixels, residual <= 10 mm); the IPPE solution whose +z normal lies within `tof.max_angle_deg` (20) of the
plane normal wins when the other does not. Serves G3.

### D11: POI contract: kind `marker` with pose and `MarkerInfo`

poi_store's `Poi` gains kind `marker` (point-like), `z` (m, default 0 for every kind), `orientation` (`[qx, qy, qz,
qw]`, required and normalised for markers, None for other kinds) and `marker: MarkerInfo | None` (required for markers,
forbidden otherwise). `MarkerInfo` (in `shared/ros2_common/marker_payload.py`): `kind`, `id` (N), `key`, `size_mm`,
`params`, `payload` (canonical text), `version`, `pose_samples` (sightings averaged since the last relocation). Marker
POIs reuse `times_seen`, `first_seen`, `last_seen`, `confidence`. No `category` field: the later agent-skills epic can
derive one from `marker.kind`. Stored files without the new fields load unchanged. Serves U3, U4, G3, G4.

### D12: Upsert and refinement in poi_store

New command op `upsert_marker` (`poi`: marker POI fields from one sighting, `confidence` the sighting's). The store,
which handles commands one at a time, matches an existing `marker` POI by `marker.key`: none -> add (created_by agent,
name `<kind> <N>` truncated to 60, times_seen 1); found and the sighting within `markers.relocate_m` (0.25) and
`markers.relocate_deg` (20) -> confidence-weighted mean of position and of the sign-aligned quaternion, old weight
min(pose_samples, `markers.max_pose_samples` 20) times the stored confidence; farther -> the marker moved: pose
replaced, pose_samples 1. Either way times_seen +1, last_seen now, size/params/payload from the latest decode, and
name, note, status untouched. Doing it in the store (not the detector) keeps it atomic and survives detector restarts.
Serves U3, G3.

### D13: Consumers: mcp_server and web UI

mcp_server: `PoiView` gains `z`, `marker` (kind, id, size_mm, params, times_seen, last_seen, confidence),
`facing_yaw_deg` (map yaw of the marker's +z projected on the floor; null when it tilts more than 45 deg from
horizontal, i.e. a marker lying flat) and `approach_pose` (`x`, `y`, `yaw` at `markers.standoff_m` 0.6 in front of a
vertical marker, facing it; null otherwise); `list_pois` gains an optional `marker_kind` filter and documents kind
`marker`; approach targets accept a marker POI id and take its `z` when `z_m` is omitted. Web UI: kind `marker` with
its own shape (hexagon) and a facing arrow, a details block (marker kind, id, size, height, times seen, last seen,
confidence), editable name/note/status, deletable, not draggable (the detector owns the pose). Serves U4, G4.

### D14: Deployment, Pi 5 benchmark and robot verification

CI and lint wiring for the new node, an Ansible node type and enabled client entry after poi_store, metrics on the
first free port, monitoring target, docs. `python -m marker_detector.bench` times decode, pose and zoom-crop decode on
synthetic frames (1152x648 and 576x324, 1-3 markers) and prints p50/p95 per stage as JSON; `scripts/marker_bench.sh`
runs it on the client in the deployed venv and saves `.logs/marker_bench.json`. Budget (my call): decode + pose p95
<= 100 ms per 1152x648 frame on the Pi 5, node CPU p95 <= 0.3 core at `process_hz` 2; a miss lowers `process_hz`
in client.yml and is recorded. The runbook places printed markers at tape-measured poses and checks POIs: position
error <= 3 cm and facing error <= 5 deg at <= 1.5 m (localisation included), one POI per marker after repeated
sightings, a zoom-detected marker at >= 2.5 m, nothing published without camera_info or TF. Serves G6.

## Open questions

none

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| research-prior-art-scout (markers-research.md) | ran | zxing-cpp 3.x QR decode with corner positions (https://pypi.org/project/zxing-cpp/); segno with `boost_error=False` (https://pypi.org/project/segno/); OpenCV IPPE + solvePnPGeneric for the two planar solutions (https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html); Bambu object-per-filament and model_settings.config (https://forum.bambulab.com/t/merging-multiple-stl-files-in-to-one/102274); print at 100 % on a rigid plate (https://github.com/adujardin/kalibr/wiki/calibration-targets); legacy ChArUco even-row change (https://forum.opencv.org/t/working-with-commercial-charuco-boards/13624) | pyzbar (unmaintained, 2022); WeChat QR as primary (contrib, mixed results, https://learnopencv.com/wechat-qr-code-scanner-in-opencv); OpenSCAD lazy-union 3MF (experimental); zbar_ros and qr_code_detector_ros (no pose or ROS1) |
| research-paper-scout (markers-research.md) | ran | AprilTag 3 range data used to size markers and justify zoom stills (https://april.eecs.umich.edu/media/pdfs/krogius2019iros.pdf) | recursive AprilTag hosting a QR (2x2 hole, same source); QR + companion fiducial (https://www.mdpi.com/2076-3417/13/24/13079) |
| research-docs-purist | ran inside the same brief (OpenCV, segno, reportlab, build123d docs) | Ubuntu 24.04 OpenCV 4.6 lacks QRCodeDetectorAruco/CharucoDetector -> uv venv (https://packages.ubuntu.com/noble/python3-opencv); QR capacities (https://www.thonky.com/qr-code-tutorial/character-capacities) | build123d Mesher (heavy OpenCascade dependency for two extruded bodies) |
| research-code-internal | planner read the code instead of a separate reuse-hunter run (scope is one repo, parts named in the brief) | PoiStore op dispatch (nodes/poi_store/poi_store/store.py:PoiStore.apply), Poi sighting fields (nodes/poi_store/poi_store/models.py:Poi), PoiView (nodes/mcp_server/mcp_server/perception_models.py:PoiView), poi_lookup (nodes/mcp_server/mcp_server/approach_tools.py:poi_lookup), ros2_common shared models precedent (shared/ros2_common/photos.py:PhotoSelector) | none |
| research-practitioner | skipped: the prior-art brief already carries the 3D-printing and print-scaling practice | - | - |

### Research inputs

| Source | Entry | Lands in |
|---|---|---|
| markers-research.md reuse | PoiStore op_add/op_update | poi-marker-upsert |
| markers-research.md reuse | Poi model (needs pose and payload) | poi-marker-model |
| markers-research.md reuse | PoiView/PoiResult | mcp-marker-pois |
| markers-research.md reuse | ChArUco board definition from the fusion design D3 | rejected here: delivered by marker-and-calibration-board |
| markers-research.md do-not-build | QR encoder: segno | rejected here: delivered by marker-and-calibration-board |
| markers-research.md do-not-build | QR decoder/locator: zxing-cpp, OpenCV QRCodeDetectorAruco second opinion | marker-decode |
| markers-research.md do-not-build | PnP and corner refinement: OpenCV | marker-pose-ippe |
| markers-research.md do-not-build | ChArUco image generation: cv2.aruco.CharucoBoard | rejected here: delivered by marker-and-calibration-board |
| markers-research.md do-not-build | 3MF writer: build123d or trimesh | rejected here: delivered by marker-and-calibration-board |
| markers-research.md do-not-build | PDF exact-size layout: reportlab | rejected here: delivered by marker-and-calibration-board |
| markers-research.md do-not-build | AprilTag pose node fallback: apriltag_ros | rejected: not built; documented as the fallback in markers-user-doc |
| markers-research.md fix-as-ac | OpenCV 4.6 lacks QRCodeDetectorAruco/ArucoDetector | marker-detector-skeleton |
| markers-research.md fix-as-ac | segno silent EC boost | rejected here: delivered by marker-and-calibration-board |
| markers-research.md fix-as-ac | print scaling | rejected here: delivered by marker-and-calibration-board |
| markers-research.md fix-as-ac | side length ambiguity | marker-pose-ippe (definition delivered by marker-and-calibration-board) |
| markers-research.md fix-as-ac | module too small for FDM; flush inlay | rejected here: delivered by marker-and-calibration-board |
| markers-research.md fix-as-ac | IPPE ambiguity | marker-pose-ippe, marker-tof-plane |
| markers-research.md fix-as-ac | Poi model has no z/orientation/marker kind; dedup | poi-marker-model, poi-marker-upsert |
| markers-research.md fix-as-ac | ChArUco bed fit | rejected here: delivered by marker-and-calibration-board |
| markers-research.md fix-as-ac | no placeholder POIs | marker-detector-node |
| markers-research.md ideas adopted | payload `P1/K:../N:../S:..` | rejected here: delivered by marker-and-calibration-board |
| markers-research.md ideas adopted | zoom still for distant markers | marker-zoom-planner, marker-detector-node |
| markers-research.md ideas adopted | ToF plane breaks IPPE ambiguity | marker-tof-plane |
| markers-research.md ideas adopted | flush two-body inlay for AMS | rejected here: delivered by marker-and-calibration-board |
| markers-research.md ideas rejected | AprilTag/ArUco alone; QR + companion AprilTag; recursive AprilTag; Micro QR/rMQR/JAB/CCTag; pyzbar; WeChat primary; OpenSCAD lazy-union | rejected: reasons in Research and D1 |
| markers-research.md synthesis | Pi 5 QR timing unpublished, must be measured | marker-detector-bench, markers-final-runbook |

## Irreversible operations

None. poi_store files without marker fields load unchanged; no data is deleted.

## Migrated notes

### README.md: Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.

Verdict: pass (2026-10-22)

Closed by user decision 2026-10-22 (answer A): every remaining task is done, verified and merged. The one leftover
live runbook task markers-final-runbook moved, requirement and acceptance conditions verbatim, to the follow-up epic printable-markers-runbook (docs/plans/todo/printable-markers-runbook),
blocked by drivable-ground-tof-filterin. The runbook moved to printable-markers-runbook.

### design.md: Summary

Printed QR markers (format, codec and generator: epic marker-and-calibration-board) become POIs. A new Python node
`nodes/marker_detector` on the client (own uv venv with OpenCV >= 4.9 and zxing-cpp) decodes markers in the head camera
stream and in any full-resolution zoom still (automatic zoom requests behind a config switch `auto_zoom`, off by
default), estimates the 6-DoF pose from the decoded side length (24 refined finder corners, IPPE + LM, ambiguity broken
by reprojection error or the registered ToF plane), transforms it into `map` through TF and sends one `upsert_marker`
command to poi_store, which keeps one POI of the new kind `marker` per payload key and refines its pose over sightings.
mcp_server and the web UI show and use marker POIs.

### design.md: Intent and why

Fixed places the robot must find again (its base, the charger, a drop-off container, buttons and switches, a tool
change station) are today hand-placed POIs that drift from reality. The user wants printed, self-describing squares:
stick one on the place, and the robot turns it into a correct POI with position, orientation and meaning, with no
manual mapping step (U2, U3). Markers must be printable on paper and in two colours on the user's Bambu printer with
AMS (U5), and the same tool should print the RGB-D calibration board the head fusion epic needs (U6).

### design.md: Goals

Ids follow the original epic; G1, G2 and G5 moved to marker-and-calibration-board.

- G3: on the robot, a sighted marker becomes a `marker` POI in `map` with pose, payload fields, last-seen time and
  confidence; repeated sightings refine it, never duplicate it; nothing is sent without a decoded, valid payload and an
  unambiguous pose.
- G4: kinds base, charger, drop-off, button, switch, tool station are known; unknown kinds are accepted (extensible);
  the agent lists marker POIs (filterable by marker kind) and uses them as approach and navigation targets.
- G6: READMEs, tests, deployment of the detector, a Pi 5 decode benchmark, and a robot runbook with printed markers at
  measured poses.

### design.md: Non-goals

- The payload format, codec, generator and calibration board (epic marker-and-calibration-board).
- Printing the markers (the user prints).
- Coloured or non-square markers; Micro QR, rMQR, AprilTag or ArUco as the primary marker.
- A POI `category` field (the agent-skills epic adds it later; a marker POI carries its marker kind in `marker.kind`).
- Broadcasting marker TF frames or a marker topic other than the detection report.
- Calibrating the head camera (the head RGB-D fusion epic owns calibration).

### design.md: Anti-goals

- A POI written from a placeholder: no payload, an ambiguous pose, a missing camera_info, a failed TF lookup.
- Duplicate POIs for one physical marker, or user edits (name, note, status) overwritten by sightings.
- Zoom requests spamming the head camera (each costs a 0.5-0.8 s stream gap): automatic requests stay off until the
  user enables `auto_zoom`.
- Robot nodes throttled by tight cgroup caps (MEMORY: guarantee headroom).

### design.md: Constraints

- CLAUDE.md: established libraries (segno, zxing-cpp, OpenCV, reportlab, trimesh + manifold3d, pydantic), config files
  over CLI arguments, type hints, docstrings with types, constants at the top, TDD, no placeholder data on topics,
  Ansible kept in sync through the `ros2_nodes` schema, deploy only through `scripts/deploy-nodes.sh`.
- Ubuntu 24.04 apt OpenCV is 4.6 (no `QRCodeDetectorAruco`, no `CharucoDetector`): detection and the generator run in
  uv environments with `opencv-python-headless>=4.9,<5`, which shadows the system package in the
  `--system-site-packages` node venv.
- Bambu Lab bed 256x256 mm; FDM module edge >= 1.5 mm (0.4 mm nozzle).
- The detector runs on the client Pi 5 next to head_camera_cpp, head_rgbd_cpp and poi_store; it needs a calibrated
  `/head_camera/camera_info` (fusion epic) and head TF (pan/tilt epic).
- web_ui and mcp_server stay Python (MEMORY).

### design.md: Decision coverage

| Decision | Tasks |
|---|---|
| D3 Side length and marker frame | marker-pose-ippe, markers-user-doc |
| D8 Detector node | marker-detector-skeleton, marker-decode, marker-detector-node |
| D9 Pose pipeline | marker-pose-ippe, marker-map-pose-poi |
| D10 Zoom stills and ToF plane | marker-zoom-planner, marker-tof-plane, marker-detector-node |
| D11 POI marker contract | poi-marker-model |
| D12 Upsert and refinement | poi-marker-upsert, marker-map-pose-poi |
| D13 Consumers | mcp-marker-pois, web-ui-marker-pois |
| D14 Deployment, benchmark, runbook | marker-detector-ci-wiring, marker-detector-ansible, marker-detector-bench, markers-user-doc, markers-final-runbook |

### design.md: Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `poi-marker-contract` | poi-marker-model | poi-marker-upsert, mcp-marker-pois, web-ui-marker-pois | `nodes/poi_store/poi_store/models.py` |
| `poi-upsert-marker-op` | poi-marker-upsert | marker-map-pose-poi | `nodes/poi_store/poi_store/store.py` |
| `marker-detector-config` | marker-detector-skeleton | marker-decode, marker-zoom-planner, marker-pose-ippe, marker-tof-plane, marker-map-pose-poi, marker-detector-node, marker-detector-bench, marker-detector-ansible | `nodes/marker_detector/marker_detector/config.py` |
| `marker-synthetic-scenes` | marker-decode | marker-pose-ippe, marker-tof-plane, marker-detector-node, marker-detector-bench | `nodes/marker_detector/marker_detector/synthetic.py` |
| `marker-decode` | marker-decode | marker-detector-node, marker-detector-bench | `nodes/marker_detector/marker_detector/decode.py` |
| `marker-pose` | marker-pose-ippe | marker-tof-plane, marker-map-pose-poi, marker-detector-node, marker-detector-bench | `nodes/marker_detector/marker_detector/pose.py` |
| `marker-tof-plane` | marker-tof-plane | marker-detector-node | `nodes/marker_detector/marker_detector/tof_plane.py` |
| `marker-geometry` | marker-map-pose-poi | marker-detector-node | `nodes/marker_detector/marker_detector/geometry.py` |
| `marker-zoom` | marker-zoom-planner | marker-detector-node | `nodes/marker_detector/marker_detector/zoom.py` |
| `marker-detector-node-entry` | marker-detector-node | marker-detector-ansible, markers-final-runbook | `nodes/marker_detector/marker_detector/node.py` |

### design.md: Review focus

- pose.py: corner ordering from zxing-cpp in the symbol's own orientation, the 24 model points, the ambiguity ratio and
  the rejection paths; synthetic accuracy tests must use independent ground truth, not the code under test.
- tof_plane.py: the normal sign convention (+z out of the face vs camera-facing plane normal).
- geometry.py: zoom-crop intrinsics (scale then shift order), TF composition order (map <- optical <- marker).
- store.py upsert: atomic match by key, user fields preserved, quaternion sign alignment, relocation path.
- Placeholder rules: no command without camera_info, TF, payload and an unambiguous pose; `/markers/detections` only
  with accepted detections.
- Zoom: no automatic request while `auto_zoom` is false; external stills still processed.

### design.md: Left to the builder

- Internal helpers of the generator and detector modules, within the named public functions.
- Mesh construction method (manifold3d unions or a merged-run extrusion), provided bodies are watertight, disjoint and
  the module footprint equals the QR matrix.
- The metrics port (first free one per tests/test_node_metrics_wiring.py) and the exact Prometheus metric names, as
  long as accepted and rejected detections (by reason) and zoom requests are counted.
- The Python node count in tests/test_ci_workflow.py (whatever the tree holds when the task lands).

### design.md: Assumptions and unknowns

- zxing-cpp returns corners in the symbol's own orientation (top-left first) for rotated symbols: confirmed by
  marker-decode's rotation tests; if not, the finder positions reorder them.
- Lens position changes with zoom autofocus shift the intrinsics slightly; the runbook measures the zoom-path pose
  error separately.
- Pi 5 decode cost is unknown until marker-detector-bench runs on the robot.
- The detector needs the fusion epic's head camera calibration and the pan/tilt head TF at runtime, and the payload
  codec from marker-and-calibration-board at build time; this epic is blocked by both.
- Whether automatic zoom requests are wanted is the user's call (gate question); planned off by default.

### approvals.json

- G3: Detection on the robot: find and decode markers in the head camera image (zoom stills where needed), estimate the 6-DoF pose from the known size and intrinsics, transform into map via TF, create/update the matching POI (kind, pose, metadata, last-seen, confidence); repeated sightings refine, not duplicate; nothing published without real detections. [decisions: D3, D8, D9, D10, D11, D12] [tasks: poi-marker-model, poi-marker-upsert, marker-detector-skeleton, marker-decode, marker-pose-ippe, marker-tof-plane, marker-map-pose-poi, marker-zoom-planner, marker-detector-node] [status: approved]
- G4: POI kinds include base, charger, drop-off container, button/switch and tool change station (extensible); the agent can list marker POIs and use them as targets. [decisions: D11, D13] [tasks: poi-marker-model, mcp-marker-pois, web-ui-marker-pois] [status: approved]
- G6: READMEs, tests and deployment of the detection part; verified on the robot with printed markers at known poses. [decisions: D14] [tasks: marker-detector-ci-wiring, marker-detector-bench, marker-detector-ansible, markers-user-doc, markers-final-runbook] [status: approved]
- U2: Metadata in the marker allows automatic POI conversion and states its size (always square). [decisions: D3, D9] [tasks: marker-decode, marker-pose-ippe] [status: approved]
- U3: A detected marker's orientation, placement and metadata automatically become a POI. [decisions: D8, D9, D11, D12] [tasks: marker-pose-ippe, marker-map-pose-poi, poi-marker-model, poi-marker-upsert, marker-detector-node, markers-final-runbook] [status: approved]
- U4: Example kinds: base, charger, drop-off container, button/switch marker, tool change station (extensible). [decisions: D11, D13] [tasks: poi-marker-model, mcp-marker-pois, web-ui-marker-pois] [status: approved]
