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

## Calibration board

The same generator prints the ChArUco board that `scripts/head_rgbd_calibrate.py` uses to calibrate the head RGB-D pair. The spec lives in the `charuco` block of `scripts/markers_generate.yaml` and is owned by the generator; the `board` block of `scripts/head_rgbd_calibrate.yaml` must equal it (a test pins this).

| Key | Value |
|---|---|
| squares | 6 x 5 (`squares_x` x `squares_y`) |
| `square_mm` / `marker_mm` | 40 / 30 |
| `dictionary` | `DICT_5X5_100` |
| plate | 250 x 210 mm, 4 mm thick, 5 mm margin, black flush inlay 1.2 mm |
| outputs | PDF (A4 landscape), STL pair and 3MF, stem `charuco-6x5-40mm` |

Why 6x5 with 40 mm squares: the plate must fit the 256 mm Bambu bed, so 6 x 40 mm plus margins is 250 mm; 40 mm squares are large enough for the 576x324 mapping image and the low-resolution ToF amplitude image to resolve corners; the row count stays odd so the legacy OpenCV ChArUco layout (even-row change) cannot flip the board origin. Print at 100 % and mount rigid, like the markers.

## Detection

`marker_detector` runs on the client next to `poi_store`; defaults are in `nodes/marker_detector/README.md`.

- Stream path: the newest frame of `/head_camera/image_raw/compressed` (1152x648) at most `process_hz` (2) times per second, decoded with zxing-cpp on the grey image (QR only). Without a calibrated `/head_camera/camera_info` nothing is processed.
- Zoom path: every full-resolution zoom still on `/head_camera/zoom/image` (paired with `/head_camera/zoom/result` by `stamp_ns`, requested by the web UI click-to-zoom, the agent's `get_camera_image` zoom or the node itself) is decoded with intrinsics scaled to the full sensor and shifted by the crop. Automatic zoom requests are off by default (`zoom.auto_zoom: false`) because each costs a 0.5-0.8 s gap in the stream; when enabled, a small (module under `zoom.min_module_px` 6 px) or undecodable QR quad triggers one request, rate limited by `zoom.min_interval_s` (10 s) and a 30 s cooldown per region.
- Pose pipeline: 24 finder corners (outer 7x7 and inner 3x3 square of three finders) are refined with `cornerSubPix`, `solvePnPGeneric` (IPPE) returns the two planar solutions, each is refined with Levenberg-Marquardt, the pose is transformed through TF (`map <- camera frame` at the image stamp) and sent as one `upsert_marker` command on `/poi/command`.
- Ambiguity and ToF: the best solution is accepted when the second one's reprojection RMS is at least `pose.ambiguity_ratio` (2.0) times larger. Otherwise, with `tof.enabled` (default true), a registered depth image within `tof.max_dt_s` (0.15 s) gives a plane inside the marker polygon, and the solution whose +z is within `tof.max_angle_deg` (20) of the plane normal wins when the other is not. Otherwise the detection is rejected as `ambiguous`. Also rejected: RMS above `pose.max_rms_px` (1.0), module under `pose.min_module_px` (4 px), marker behind the camera, confidence under `pose.min_confidence` (0.3). ToF is not used for zoom stills.
- Placeholder rules: nothing is sent without a decoded valid payload, calibrated camera_info, a successful TF lookup and an unambiguous pose. A bad frame, uncalibrated camera or failed TF produces no command and no report.
- `/markers/detections` (std_msgs/String JSON): `{"stamp_ns", "source": "stream"|"zoom", "markers": [{"key", "payload", "position", "quaternion", "rms_px", "ratio", "confidence", "module_px", "chooser"}]}`, published only for frames with at least one accepted marker. Metrics (`marker_detections_accepted_total`, `marker_rejections_total{reason}`, `marker_zoom_requests_total{status}`) are on port 19118.

## POIs

- A marker POI has kind `marker`, `x`, `y`, `z`, `orientation`, a `marker` block (`kind`, `id`, `key`, `size_mm`, `params`, `payload`, `version`, `pose_samples`) and the sighting fields `times_seen`, `first_seen`, `last_seen`, `confidence`. See `nodes/poi_store/README.md`.
- Upsert: `upsert_marker` matches by `marker.key`. No match adds a POI named `<marker.kind> <id>`, created by `agent`. A match within `markers.relocate_m` (0.25 m) and `markers.relocate_deg` (20 deg) merges the pose as a confidence-weighted mean of position and sign-aligned quaternion (history weight capped at `markers.max_pose_samples`, 20). A match farther away means the marker moved: the pose is replaced and `pose_samples` restarts at 1. Either way `times_seen` grows and size, params and payload come from the latest decode; `name`, `note` and `status` you edited are never overwritten. A marker POI is never duplicated by repeated sightings.
- Agent (mcp_server): `list_pois(marker_kind=...)` lists marker POIs; their views carry `z`, `marker`, `facing_yaw_deg` (null for a marker lying flat) and `approach_pose` (`markers.standoff_m` 0.6 m in front of an upright marker, facing it). A marker `poi_id` is an approach target for `suggest_approach_pose` and `approach_for_grasp` and supplies its own `z` when `z_m` is omitted.
- Web UI: marker POIs are drawn as a hexagon with a facing arrow; the panel shows marker kind, id, size, height, times seen, last seen and confidence, lets you edit name, note and status and delete the POI, and does not allow dragging (the detector owns the pose).

## Range and fallback

Pose needs about 4 px per module (`pose.min_module_px`); zxing-cpp alone decodes down to about 1.4 px per module on sharp synthetic images. The head camera stream has f = 887 px at 1152 px width; a zoom of Nx multiplies the focal length by N. Range in metres at 4 px per module, on-axis:

| Marker (version) | Module | Stream | Zoom 2x | Zoom 4x | Zoom 8x |
|---|---|---|---|---|---|
| 60 mm v2 | 2.40 mm | 0.53 | 1.06 | 2.13 | 4.26 |
| 60 mm v3 | 2.07 mm | 0.46 | 0.92 | 1.84 | 3.67 |
| 100 mm v2 | 4.00 mm | 0.89 | 1.77 | 3.55 | 7.10 |
| 100 mm v3 | 3.45 mm | 0.76 | 1.53 | 3.06 | 6.12 |
| 150 mm v2 | 6.00 mm | 1.33 | 2.66 | 5.32 | 10.64 |
| 150 mm v3 | 5.17 mm | 1.15 | 2.29 | 4.59 | 9.18 |
| 200 mm v3 | 6.90 mm | 1.53 | 3.06 | 6.12 | 12.23 |

So a 100 mm v3 marker decodes to about 0.75 m in the stream and about 3 m through a 4x zoom still. Off-axis viewing, blur and autofocus shift reduce this; the robot runbook measures the real range and pose error. Use larger or lower-version (shorter payload) markers for anything the robot must see from far.

Fallback: if the field range is insufficient, use AprilTag `tagStandard41h12` detected with `apriltag_ros` (id-only markers, a separate id-to-meaning table, a larger range per printed size). It replaces only the decode and pose stages; the `upsert_marker` POI contract is unchanged.

Measured field ranges: not yet measured. The robot runbook `docs/runbooks/printable-markers.md` (step 10) fills this section; it is blocked on the head camera calibration (fusion epic) and on the printed markers. The table above is computed, not measured.
