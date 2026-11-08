# Printable metadata markers

A printed QR square stuck on a place (base, charger, drop-off, button, switch, tool station) becomes a POI in `map` with position, orientation and meaning. The head camera sees it, the `marker_detector` node decodes it and estimates its 6-DoF pose, and `poi_store` keeps one `marker` POI per payload, refining its pose over sightings. This page is for a developer who prints, places and uses markers. Code: payload codec `shared/ros2_common/marker_payload.py`, generator `scripts/markers_generate.py` (`scripts/markers/`), detector `nodes/marker_detector/`, POIs `nodes/poi_store/`.

## Payload format

A marker is a QR Model 2 symbol (not AprilTag/ArUco: it carries text, so the marker is self-describing) whose text is the P1 payload, restricted to the QR alphanumeric set (`0-9 A-Z space $ % * + - . / :`), which packs densest.

```
P<major>[.<minor>]/K:<KIND>/N:<NAME>/S:<MM>[/<KEY>:<VAL>...]
```

| Segment | Meaning | Rule |
|---|---|---|
| `P1` / `P1.2` | payload version | first segment; only major 1 is supported, a minor is optional and accepted |
| `K` | kind | `[A-Z0-9-]{1,16}` |
| `N` | name, unique per kind | `[A-Z0-9-]{1,20}` |
| `S` | symbol edge in mm | 20..1000, at most one decimal (`150`, `42.5`) |
| extra `KEY:VAL` | free parameters | key `[A-Z0-9]{1,8}` and not `P K N S`; value `[A-Z0-9 $%*+.-]{0,24}`; no duplicate keys |

Known kinds and the POI marker kind they produce (`marker.kind`); any other K is accepted and becomes its lower-case text.

| K | POI `marker.kind` |
|---|---|
| `BASE` | `base` |
| `CHARGER` | `charger` |
| `DROPOFF` | `drop_off` |
| `BUTTON` | `button` |
| `SWITCH` | `switch` |
| `TOOLSTATION` | `tool_station` |

The POI key is `<marker.kind>/<N>`, for example `charger/DOCK-1`; it identifies the physical marker, so two prints of the same payload are the same POI.

Version rule: a payload with an unsupported major version, a missing or duplicate key, an empty segment, a character outside the alphanumeric set or an invalid field is rejected by the detector (rejection reason `payload`) and never reaches `poi_store`. Encoding is canonical (P, K, N, S, then parameters sorted by key).

Examples:

- `P1/K:BASE/N:HOME/S:150` (22 characters)
- `P1/K:CHARGER/N:DOCK-1/S:150` (27)
- `P1/K:BUTTON/N:LIGHT-1/S:60/H:1200` (33, a parameter `H`)

Capacity (alphanumeric characters) per QR version and error correction; the generator picks the smallest version between `min_version` (2) and `max_version` (3) that fits, with the shipped ECC `q` (no silent ECC boost). Modules per edge are `17 + 4 * version`.

| Version | Modules | ECC M | ECC Q |
|---|---|---|---|
| 2 | 25 | 38 | 29 |
| 3 | 29 | 61 | 47 |
| 4 | 33 | 90 | 67 |

A payload longer than the capacity of `max_version` is an error naming the length and the capacity. `min_version` and `max_version` accept 2..4.

## Size and frame

- `S` is the symbol edge from the outer edge of one finder pattern to the outer edge of the opposite side, quiet zone excluded. The detector trusts `S` from the payload to size the pose, so it must equal the printed edge (check it with the 100 mm ruler on the PDF).
- Module edge = `S / (17 + 4 * version)`: 150 mm v2 is 6.0 mm, 100 mm v3 is 3.45 mm.
- Marker frame: origin at the symbol centre, +x along the top edge (the edge holding the two upper finders, left to right as read), +y along the left edge towards the top, +z out of the printed face towards a viewer.
- The POI position is the frame origin and its orientation is this frame expressed in `map` (unit quaternion `[qx, qy, qz, qw]`, with `z` the height above the map plane). The face normal (+z) is the direction the marker faces; a marker on a wall faces the room, a marker on the floor faces up.

## Generating markers

The generator `scripts/markers_generate.py` writes print-ready files; the format owner is `shared/ros2_common/marker_payload.py`.

1. Edit `scripts/markers_generate.yaml`: `output_dir` (default `.logs/markers`, gitignored, never commit print files), `defaults` (`ecc`, `min_version`, `max_version`, `quiet_zone_modules`, `page`), `stl` (`base_thickness_mm` 4, `inlay_mm` 1.2, `margin_mm`, `min_module_mm` 1.5, `warn_module_mm` 2) and `markers[]` (`kind_code`, `name`, `size_mm`, optional `params`, `outputs`, `page`). The shipped list has HOME, DOCK-1, BIN-1, LIGHT-1, FAN-1 and TS-1.
2. Run `uv run python -m scripts.markers_generate` (optional argument: a config path). Exit 1 on an invalid config (nothing written) or a marker that cannot be produced (named; the others are still written).
3. Outputs per marker, stem `<kind>-<name>` in lower case: `charger-dock-1.pdf`, `charger-dock-1-base.stl`, `charger-dock-1-modules.stl`, `charger-dock-1.3mf`.

Printing on paper: print the PDF at 100 % (actual size, "fit to page" off) and mount it flat on a rigid plate (foam board, acrylic, wood); a curled marker breaks the planar pose. Verify the printed 100 mm ruler and measure the symbol edge: it must match `S`. Keep the white quiet zone (4 modules) clear.

Printing in two colours on a Bambu printer with AMS: either load the two STLs (`-base.stl` white, `-modules.stl` black) as separate objects and assign one filament each, or open the `.3mf`, which holds the same two bodies with extruders 1 and 2 already assigned. The black modules are inlaid flush into the base (1.2 mm black is opaque in PETG; 4 mm base). The module edge must be at least 1.5 mm (the generator refuses less and warns under 2 mm); a 20 mm marker is therefore too small for STL, 100 mm modules are 3.4-4 mm. The bed is 256 x 256 mm.

Recommended sizes: 150-200 mm for docking range (base, charger, drop-off seen from 1-2 m), 100 mm for stations, 60 mm for buttons and switches near the arm (seen from well under 1 m). See [Range and fallback](#range-and-fallback).

## Calibration boards

The same generator prints the two ChArUco boards of the head and arm calibrations. Their specs are the `boards:` list
of `scripts/markers_generate.yaml`, owned by the generator; the calibration side holds the same boards as `FULL_BOARD`
and `ARM_BOARD` (`shared/ros2_common/head_rgbd_calibration/config.py`), and tests pin the node config
(`board`, `fit.board`, `fit.plate`) and these constants to the generator entries. Boards sharing a dictionary must use
disjoint marker ids (the generator and the node refuse an overlap), so both boards can be detected in one image without
mixing their corners.

| Key | FULL | ARM |
|---|---|---|
| `name` (file stem) | `charuco-6x5-40mm` | `charuco-arm-6x5-28mm` |
| squares | 6 x 5 | 6 x 5 |
| `square_mm` / `marker_mm` | 40 / 30 | 28 / 21 |
| `dictionary`, `ids_offset` (marker ids) | `DICT_5X5_100`, 0 (ids 0-14) | `DICT_5X5_100`, 15 (ids 15-29) |
| plate | 250 x 210 mm, 4 mm thick, 5 mm margin | 178 x 150 mm, 4 mm thick, 5 mm margin, plus the edge grip handle |
| `pattern_face` | `top` | `top` (printed back down, pattern up, with the handle) |
| exact raster | 1.4 px/mm | 2.0 px/mm (square 56 px, marker 42 px, marker cell 6 px) |
| outputs | PDF (A4 landscape), STL pair, 3MF | PDF (A4 landscape), STL pair, 3MF, `<name>.json` |
| role | D435 intrinsics check, CSI camera intrinsics | head from arm and arm offsets, held in the gripper |

Why 6x5: the row count stays odd so the legacy OpenCV ChArUco layout (even-row change) cannot flip the board origin.
The FULL plate (6 x 40 mm plus margins = 250 mm) fits the 256 mm Bambu bed and its 40 mm squares are large enough for
the 640x360 head D435 colour image to resolve corners at intrinsics distances. The ARM board is smaller so it fits
in front of the head camera on the arm and its plate plus handle (150 + 55.3 = 205.3 mm) stays on the bed. Its 28 mm
squares make the PDF raster exact at 2.0 px/mm (every square, marker, marker cell and the square-to-marker gap is a
whole number of pixels). Detection margin on synthetic D435 frames (640x360, fx 423, `tests/test_arm_board_detection.py`):
the ARM board facing the camera at 0.34 m (the base pose distance) gives at least 12 corners (the test gate); at 0.6 m
it still gives 20 corners (reported, not gated: 0.34 m is the working distance). Both boards in one 1280x720 image at
0.6 m are each detected with their own marker ids (ids 0-29 all found, each board's corners inside its own outline).

Print at 100 % and mount rigid, like the markers. The PDFs carry the label "Print at 100 %"; the fit takes the board
geometry from the config, not from the print, so check a square with a caliper.

The files to print are committed: `markers/` holds the full set (PDF, `-base.stl`, `-modules.stl`, 3MF and the ARM
JSON) and `models/` the PDF and 3MF the web UI offers for download (Calibrate page, "Board needed" lines). The
generator writes to `output_dir` (`.logs/markers`); copy a regenerated board to `markers/` and `models/` only when its
geometry changed (the 3MF object ids and the PDF date change on every run).

### ARM board edge handle

The ARM board is held in the SO-101 follower gripper by a handle printed in one piece with the plate (user m540,
decision 297). Geometry (the `handle:` block of the ARM entry, `scripts/markers/handle.py`):

- Tab: a 10 mm thick trapezoid off the middle of the plate's LONG bottom edge (board y max, outside the pattern),
  100 mm wide at the plate edge (`tab_root_width_mm`, spreads the cantilever moment into the 4 mm plate) and 40 mm at
  its far end (`tab_width_mm`), 55.3 mm long (`post_offset_mm` 43 plus half the post width). Its bed side is flat and
  coplanar with the plate back; its top stands 6 mm above the pattern face.
- Grip post: stands on the tab top on the PATTERN side, its axis 43 mm off the plate edge. It is the jaw-fit grip:
  a shoulder block (`neck_mm` 6) that stops the moving finger tip, then a 30 mm grip (`length_mm`) whose top plane
  rises from 19.7 mm (`thickness_tip_mm`, at the shoulder) to 23.4 mm (`thickness_end_mm`, at the free end) so it
  touches the moving jaw's 2 mm stepped lands, with a tapered finger channel (12.6 to 16.6 mm, finger width plus
  `clearance_mm` 0.3 per wall) between two 4 x 5 mm rails. The post top stands 42 mm above the pattern face.
- Gusset: one triangular web, 8 mm thick, 36 mm up the post and 30 mm along the tab, off the rail wall facing the
  plate (the side the jaws do not reach), its top face within 45 deg of vertical, ending 0.7 mm before the plate edge.
- Seated: the gripper holds the post from the pattern side, so the pattern faces the wrist and the head camera (its
  normal is -TOOL_AXIS, `tests/test_markers_handle.py::test_pattern_faces_back_toward_the_wrist` and
  `nodes/head_rgbd_calibration/tests/test_board_facing.py`). The jaws close along the plate edge (board x) and the
  plate lies beside them along `gripper_link` +y; the pattern plane is about 20 mm past the URDF tool point and the
  board centre about 0.12 m off the tool axis, as the old hand-measured stick mount. The whole gripper stays about
  24 mm below the first square row seen along the pattern normal.
- Grip: the numbers come from the repo's gripper meshes (`nodes/web_ui/urdf/so101_arm.urdf`, `scripts/markers/jaw_fit.py`).
  `contact_rad` 0.070279 is the gripper angle where both jaws touch the NOMINAL grip; `grip_rad` = contact -
  `squeeze_rad` 0.02 = 0.050279 is the commanded grip. The insert opening (0.4 rad) clears the post by about 11.9 mm.
- Print allowance: `clamp_clearance_mm` 0.15 is taken off each clamp face (the grip "bed face", the handle-frame name of the fixed-jaw
  clamp face, and the channel floor under the moving jaw, user decisions 282 and 283). The `thickness_*` numbers are the nominal grip (the
  part as printed after FDM growth); the printed solid is 0.3 mm thinner across the clamp faces. `contact_rad` is solved
  on the nominal surfaces and is not a measured contact; `modelled_contact_rad` 0.066202 is the contact on the printed
  solid pressed on the fixed jaw. The board prints back down, so both clamp faces are vertical
  walls of the post (not faces on the bed) and FDM growth brings a real print toward nominal; the true contact still
  lies between `modelled_contact_rad` and `contact_rad`; `grip_rad` closes past both.
- Mass and stiffness: 211.7 g (170.75 cm3 of PLA at 1.24 g/cm3), centre of mass toward the tab on the pattern side.
  Under 1 g normal to the plate (whole mass lumped at the centre, E 2000 MPa, conservative) the board centre sags
  0.039 mm in the tab and 0.209 mm in the plate, 0.248 mm in total; the tab root sees about 2 MPa under a 10 g jerk.
  At the base pose the plate is near vertical, so the load is mostly in-plane.
- Checks before writing: the generator refuses a board whose printed solid is not watertight or has a face steeper
  than 45 deg overhang (`scripts/markers/mesh.py` `print_violations`); nothing is written then. The ARM board passes:
  one body, the bed flat, the post faces lean about 7 deg from vertical, no supports.

`<name>.json` (ARM only): `board` (the `BoardConfig` fields), `plate_mm`, `volume_cm3`, `mass_g`, `centre_of_mass_m`
(board frame), `board_mount` (`xyz_init`, `rpy_init`: the board frame in `gripper_link`, the CAD seed of the fit),
`handle_in_board` (`gripper_link` in the board frame, `tool_point_m`, `tool_axis`, `open_axis`), `grip`
(`contact_rad`, `squeeze_rad`, `grip_rad`, `clearance_clamp_mm`, `modelled_contact_rad`), `handle_envelope_mm`
(`height_mm` 42 and the `post`, `gusset` and `tab` boxes in the board frame, mm, all on the pattern side, z < 0: the
collision boxes of the pose and park checks) and `stiffness`. head_rgbd_calibration, mcp_server and the client.yml
blocks copy the mount, tool point, grip angles, mass and boxes at 6 decimals; tests pin them to the JSON.

Design history (MEMORY.md, "ARM calibration board with an edge grip handle"): an in-line paddle (decision 260), the
0.15 mm clamp allowance (282, 283), twin ribs on the back (291, 292), a right-angle post on the back (296), rejected
because the pattern faced away from the camera (user m540), then this edge handle on the pattern side printed face up
(297), on the long edge (T-77 comment 221).

## Detection

`marker_detector` runs on the client next to `poi_store`; defaults are in `nodes/marker_detector/README.md`.

- Stream path: the newest frame of the head D435 colour `/head/d435/color/image_raw/compressed` (640x360) at most `process_hz` (2) times per second, decoded with zxing-cpp on the grey image (QR only), with the factory intrinsics of `/head/d435/color/camera_info`. The front and back CSI cameras (`/front_camera/image_raw/compressed`, `/back_camera/image_raw/compressed`, 1152x648) run the same path in their own pipeline and newest-frame slot, with the intrinsics applied from the Calibrate page. A camera without a valid camera_info is skipped.
- Pose pipeline: 24 finder corners (outer 7x7 and inner 3x3 square of three finders) are refined with `cornerSubPix`, `solvePnPGeneric` (IPPE) returns the two planar solutions, each is refined with Levenberg-Marquardt, the pose is transformed through TF (`map <- camera frame` at the image stamp) and sent as one `upsert_marker` command on `/poi/command`.
- Ambiguity and depth plane: the best solution is accepted when the second one's reprojection RMS is at least `pose.ambiguity_ratio` (2.0) times larger. Otherwise, with `depth_plane.enabled` (default true), the D435 depth aligned to the colour (`/head_rgbd/depth/image_raw` + camera_info, decimated by head_rgbd_cpp) within `depth_plane.max_dt_s` (0.15 s) gives a plane inside the marker polygon (at least `depth_plane.min_valid_px` (30) valid pixels, fit RMS at most `depth_plane.max_residual_mm` (10 mm)), and the solution whose +z is within `depth_plane.max_angle_deg` (20) of the plane normal wins when the other is not. Otherwise the detection is rejected as `ambiguous`. Also rejected: RMS above `pose.max_rms_px` (1.0), module under `pose.min_module_px` (4 px), marker behind the camera, confidence under `pose.min_confidence` (0.3).
- Placeholder rules: nothing is sent without a decoded valid payload, calibrated camera_info, a successful TF lookup and an unambiguous pose. A bad frame, uncalibrated camera or failed TF produces no command and no report.
- `/markers/detections` (std_msgs/String JSON): `{"stamp_ns", "source": "stream", "markers": [{"key", "payload", "position", "quaternion", "rms_px", "ratio", "confidence", "module_px", "chooser"}]}`, published only for frames with at least one accepted marker. Metrics (`marker_frames_processed_total{camera}`, `marker_detections_accepted_total{camera}`, `marker_rejections_total{camera,reason}`, `marker_process_seconds{camera}`; `camera` is the `cameras:` entry name) are on port 19118.

## POIs

- A marker POI has kind `marker`, `x`, `y`, `z`, `orientation`, a `marker` block (`kind`, `id`, `key`, `size_mm`, `params`, `payload`, `version`, `pose_samples`) and the sighting fields `times_seen`, `first_seen`, `last_seen`, `confidence`. See `nodes/poi_store/README.md`.
- Upsert: `upsert_marker` matches by `marker.key`. No match adds a POI named `<marker.kind> <id>`, created by `agent`. A match within `markers.relocate_m` (0.25 m) and `markers.relocate_deg` (20 deg) merges the pose as a confidence-weighted mean of position and sign-aligned quaternion (history weight capped at `markers.max_pose_samples`, 20). A match farther away means the marker moved: the pose is replaced and `pose_samples` restarts at 1. Either way `times_seen` grows and size, params and payload come from the latest decode; `name`, `note` and `status` you edited are never overwritten. A marker POI is never duplicated by repeated sightings.
- Agent (mcp_server): `list_pois(marker_kind=...)` lists marker POIs; their views carry `z`, `marker`, `facing_yaw_deg` (null for a marker lying flat) and `approach_pose` (`markers.standoff_m` 0.6 m in front of an upright marker, facing it). A marker `poi_id` is an approach target for `suggest_approach_pose` and `approach_for_grasp` and supplies its own `z` when `z_m` is omitted.
- Web UI: marker POIs are drawn as a hexagon with a facing arrow; the panel shows marker kind, id, size, height, times seen, last seen and confidence, lets you edit name, note and status and delete the POI, and does not allow dragging (the detector owns the pose).

## Range and fallback

Pose needs about 4 px per module (`pose.min_module_px`); zxing-cpp alone decodes down to about 1.4 px per module on sharp synthetic images. The head D435 colour stream has f = 457.8 px at 640 px width (factory intrinsics, docs/runbooks/d435-switchover.md). Range in metres at 4 px per module, on-axis:

| Marker (version) | Module | Stream |
|---|---|---|
| 60 mm v2 | 2.40 mm | 0.27 |
| 60 mm v3 | 2.07 mm | 0.24 |
| 100 mm v2 | 4.00 mm | 0.46 |
| 100 mm v3 | 3.45 mm | 0.39 |
| 150 mm v2 | 6.00 mm | 0.69 |
| 150 mm v3 | 5.17 mm | 0.59 |
| 200 mm v3 | 6.90 mm | 0.79 |

So a 100 mm v3 marker decodes to about 0.4 m in the D435 stream. Off-axis viewing and blur reduce this; the robot runbook measures the real range and pose error. Use larger or lower-version (shorter payload) markers for anything the robot must see from far.

Fallback: if the field range is insufficient, use AprilTag `tagStandard41h12` detected with `apriltag_ros` (id-only markers, a separate id-to-meaning table, a larger range per printed size). It replaces only the decode and pose stages; the `upsert_marker` POI contract is unchanged.

Measured field ranges: not yet measured. The robot runbook `docs/runbooks/printable-markers.md` (step 9) fills this section after the D435 calibration (D435 S59) is done. The table above is computed, not measured.
