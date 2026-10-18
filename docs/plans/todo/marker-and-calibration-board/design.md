# Marker and calibration board generator design

## Summary

Split from printable-metadata-markers-t so the boards and markers can be printed before the head RGB-D calibration
(epic head-rgb-d-fusion-of-camera) and before marker detection exist. Markers are QR Model 2 symbols (version 2-3,
EC Q, alphanumeric) carrying `P1/K:<KIND>/N:<NAME>/S:<MM>[/<KEY>:<VAL>...]`; the codec lives in
`shared/ros2_common/marker_payload.py`. A root `scripts/markers/` package driven by `scripts/markers_generate.yaml`
writes per marker an exact-size PDF and a two-colour Bambu AMS print set (base STL, module STL, two-object 3MF), and the
ChArUco board (6x5 squares of 40 mm, 30 mm DICT_5X5_100 markers, plate 250x210 mm on a 256 mm bed) from the single
board spec the fusion calibration tool must equal.

## Intent and why

The user wants self-describing printed markers for fixed places and the same tool to print the head RGB-D calibration
board (U1, U2, U5, U6). The fusion epic's runbook prints the board, and the detection epic needs the payload codec, so
this generator lands first.

## Goals

- G1: a documented, versioned payload format and codec that round-trips and rejects unknown major versions.
- G2: one config-driven command writes, per marker, a PDF whose symbol edge equals the encoded size within 0.1 mm, two
  STLs (base, modules) and a 3MF with both; tests check sizes, module grid and decode the rendered outputs.
- G3: the ChArUco board PDF and two-colour STL/3MF from the single board spec, fitting a 256x256 mm bed.
- G4: READMEs and tests for codec and generator.

## Non-goals

- Marker detection, pose, POIs (printable-metadata-markers-t).
- Printing (the user prints); coloured or non-square markers; camera calibration (fusion epic).

## Anti-goals

- A second board definition drifting from the one the calibration detects.
- Silent rescaling: a PDF whose printed size differs from the encoded size.
- Print files committed to git (outputs go to the gitignored `.logs/markers`).

## Constraints

- CLAUDE.md: established libraries, config files over CLI arguments, type hints, TDD.
- Root uv env (Python 3.12, opencv-python-headless >= 4.9 already present); Bambu bed 256x256 mm; FDM module >= 1.5 mm.

## Decisions

### D1: Symbology: QR Model 2, version 2-3, EC Q, alphanumeric

QR Model 2 encoded by segno with `mode="alphanumeric"`, `error="q"` (config may choose `m` for paper),
`boost_error=False` (segno otherwise raises the EC level silently), version picked as the smallest in
[`min_version` 2, `max_version` 3] that fits; a payload that does not fit raises naming the capacity. Quiet zone 4
modules (config, minimum 2). Orientation is intrinsic (three finders), so the pose has no 90-degree ambiguity.
Alternatives: AprilTag/ArUco (about 12 bits of id, not self-describing), QR plus a companion AprilTag (two markers per
place and a mapping between them, the DataMatrix-ArUco hybrid paper's motivation), a recursive AprilTag hosting a QR
(2x2-cell hole), Micro QR (21 alphanumeric characters), rMQR/JAB/CCTag (non-square, colour, circular). AprilTag
tagStandard41h12 via apriltag_ros stays the documented fallback if the runbook shows QR range is insufficient.
Serves U1, G1.

### D2: Payload P1 and its codec in shared

`P1/K:<KIND>/N:<NAME>/S:<MM>[/<KEY>:<VAL>...]` in the QR alphanumeric set (0-9 A-Z space $%*+-./:). `P<major>[.<minor>]`
first; major 1 only (any other major rejected with a named error); minor ignored. Required keys `K` (kind,
`[A-Z0-9-]{1,16}`), `N` (name, `[A-Z0-9-]{1,20}`, unique per kind), `S` (symbol edge in mm, 20-1000, at most one
decimal). Further `KEY:VAL` pairs (`[A-Z0-9]{1,8}` : `[A-Z0-9 $%*+.-]{0,24}`) are kept as params; duplicate keys are
rejected. Encoding writes P, K, N, S, then params sorted by key, so a payload has one canonical text. Known kinds map
to POI marker kinds: BASE->base, CHARGER->charger, DROPOFF->drop_off, BUTTON->button, SWITCH->switch,
TOOLSTATION->tool_station; any other valid K maps to its lower-case form (extensible). The marker key is `<kind>/<N>`.
Example `P1/K:CHARGER/N:DOCK-1/S:150` (27 characters, fits version 2-Q, capacity 29). The codec is
`shared/ros2_common/marker_payload.py` (pydantic, no ROS), used by the generator, the detector and poi_store, beside
`MarkerInfo`, the POI sub-model built from a payload (printable-metadata-markers-t D11 consumes it). Serves U2, G1, G4.

### D3: Side length and marker frame

`S` is the symbol edge from the outer edge of one finder to the outer edge of the opposite side, excluding the quiet
zone; module edge = S / (17 + 4 * version). Marker frame: origin at the symbol centre, +x along the top edge (the edge
holding the two upper finders, left to right as read), +y along the left edge towards the top, +z out of the printed
face towards a viewer. The POI position is the origin, its orientation this frame in `map`. (my call) Serves U2, G1.

### D4: Generator: root `scripts/markers/` package, one config file

`scripts/markers/` (package: `config.py`, `symbol.py`, `pdf.py`, `mesh.py`, `charuco.py`) with the entry
`uv run python -m scripts.markers_generate [config]`; the only CLI argument is an optional config path, default
`scripts/markers_generate.yaml` (pydantic, unknown keys forbidden): `output_dir` (default `.logs/markers`, gitignored;
my call), defaults (`ecc`, `min_version`, `max_version`, `quiet_zone_modules`, `page`, `stl` options), the `markers`
list (`kind`, `name`, `size_mm`, `params`, `outputs` subset of pdf/stl/3mf, optional `page`) and the `charuco` spec.
The shipped file lists one example per known kind (base, charger, drop-off, button, switch, tool station). Root dev
dependencies: segno, reportlab, pypdfium2 (tests rasterise PDFs), trimesh, manifold3d, zxing-cpp. Alternatives: a
separate tools package with its own uv.lock (a 24th environment for an offline tool that already fits the root dev
env, which carries OpenCV, pydantic and numpy), build123d (OpenCascade wheels, heavy). (my call) Serves G2.

### D5: PDF at exact physical size

reportlab, page size equal to the media (A4 portrait default, A3/Letter/custom by config), no scaling; modules drawn
as vector rectangles (horizontal runs merged), white quiet zone, a thin grey cut square outside it, a label (kind,
name, size, payload text), a 100 mm ruler bar and the line "Print at 100 % (actual size), no fit to page; mount on a
rigid plate". A symbol plus quiet zone plus label that does not fit the page raises. Tests rasterise the PDF with
pypdfium2 at 600 dpi and measure the symbol edge and ruler within 0.1 mm, then decode the raster with zxing-cpp.
Serves U5, G2.

### D6: Two-colour STL and 3MF for Bambu AMS

trimesh + manifold3d. Base body (white): a plate of side S + 2 * quiet zone + 2 * `margin_mm` (default 3) and
`base_thickness_mm` (default 2.0). Module body (black): the dark modules, adjacent ones unioned, rising
`relief_mm` (default 0.6, three 0.2 mm layers) above the base top as the user chose (U5); an optional `inlay_mm`
(default 0) sinks the modules into matching pockets of the base so `relief_mm: 0` gives a flush inlay. The bodies never
overlap. Outputs `<stem>-base.stl` and `<stem>-modules.stl` in one coordinate frame (Bambu Studio offers "one object,
multiple parts" when both are loaded) and `<stem>.3mf` with two named objects `base` and `modules` plus
`Metadata/model_settings.config` assigning filament 1 to base and 2 to modules (Bambu ignores 3MF colours). The
generator refuses a module edge below 1.5 mm and warns below 2 mm. Alternative: flush inlay as the default (the 3D
printing guide's preference) is available but not default, because the user asked for raised modules. Serves U5, G2.

### D7: ChArUco board owned by the generator; 6x5 squares of 40 mm

The `charuco` spec in `scripts/markers_generate.yaml` is the single board definition: `squares_x` 6, `squares_y` 5,
`square_mm` 40.0, `marker_mm` 30.0, `dictionary` DICT_5X5_100, `margin_mm` 5, `thickness_mm` 4.0, `relief_mm` 0.6,
`bed_mm` 256. Board 240x200 mm; plate 250x210 mm fits the 256 mm bed; PDF on A4 landscape. The fusion epic's 7x5
default (280 mm) does not fit the bed, so its default became 6x5 with the same square and marker sizes (ToF
detectability unchanged); the fusion epic's `rgbd-calibration-frames` (which lands after this epic) asserts that its yaml `board` equals this spec, so the printed board is the one the calibration detects. The
odd row count avoids the OpenCV 4.6-to-4.7 even-row layout change (`setLegacyPattern`). Board image and geometry come
from `cv2.aruco.CharucoBoard` (same API as the fusion tool), STL bodies from D6's mesh helpers; the spec validator
refuses a plate larger than `bed_mm` and `marker_mm >= square_mm`. Alternative: 7x5 at 35 mm (245 mm plus margins
leaves under 6 mm on the bed, smaller ToF cells). PDF on a rigid plate is the recommended print; the STL is the
alternative. Serves U6, G3.

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| research-prior-art-scout (markers-research.md) | ran | zxing-cpp 3.x QR decode with corner positions (https://pypi.org/project/zxing-cpp/); segno with `boost_error=False` (https://pypi.org/project/segno/); OpenCV IPPE + solvePnPGeneric for the two planar solutions (https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html); Bambu object-per-filament and model_settings.config (https://forum.bambulab.com/t/merging-multiple-stl-files-in-to-one/102274); print at 100 % on a rigid plate (https://github.com/adujardin/kalibr/wiki/calibration-targets); legacy ChArUco even-row change (https://forum.opencv.org/t/working-with-commercial-charuco-boards/13624) | pyzbar (unmaintained, 2022); WeChat QR as primary (contrib, mixed results, https://learnopencv.com/wechat-qr-code-scanner-in-opencv); OpenSCAD lazy-union 3MF (experimental); zbar_ros and qr_code_detector_ros (no pose or ROS1) |
| research-paper-scout (markers-research.md) | ran | AprilTag 3 range data used to size markers and justify zoom stills (https://april.eecs.umich.edu/media/pdfs/krogius2019iros.pdf) | recursive AprilTag hosting a QR (2x2 hole, same source); QR + companion fiducial (https://www.mdpi.com/2076-3417/13/24/13079) |
| research-docs-purist | ran inside the same brief (OpenCV, segno, reportlab, build123d docs) | Ubuntu 24.04 OpenCV 4.6 lacks QRCodeDetectorAruco/CharucoDetector -> uv venv (https://packages.ubuntu.com/noble/python3-opencv); QR capacities (https://www.thonky.com/qr-code-tutorial/character-capacities) | build123d Mesher (heavy OpenCascade dependency for two extruded bodies) |
| research-code-internal | planner read the code instead of a separate reuse-hunter run (one repo, parts named in the brief) | shared pydantic model precedent (shared/ros2_common/photos.py:PhotoSelector); root scripts imported as `scripts.<module>` (tests/test_nav_footprint.py) | none |
| research-practitioner | skipped: the prior-art brief already carries the 3D-printing and print-scaling practice | - | - |


## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| markers-research.md reuse | ChArUco board definition (fusion design D3) | charuco-board-output (spec owned here; the fusion task asserts equality) |
| markers-research.md do-not-build | QR encoder: segno | marker-qr-symbol |
| markers-research.md do-not-build | ChArUco image generation: cv2.aruco.CharucoBoard | charuco-board-output |
| markers-research.md do-not-build | 3MF writer: build123d or trimesh | marker-stl-3mf-output (trimesh + manifold3d) |
| markers-research.md do-not-build | PDF exact-size layout: reportlab | marker-pdf-output |
| markers-research.md fix-as-ac | segno silent EC boost | marker-qr-symbol |
| markers-research.md fix-as-ac | print scaling | marker-pdf-output |
| markers-research.md fix-as-ac | side length ambiguity | marker-payload-codec |
| markers-research.md fix-as-ac | module too small for FDM; flush inlay | marker-stl-3mf-output (refuse < 1.5 mm, warn < 2 mm; raised default per U5, inlay option) |
| markers-research.md fix-as-ac | ChArUco bed fit | charuco-board-output |
| markers-research.md ideas adopted | payload `P1/K:../N:../S:..` | marker-payload-codec |
| markers-research.md ideas adopted | flush two-body inlay for AMS | marker-stl-3mf-output (two bodies; inlay optional) |
| markers-research.md ideas rejected | Micro QR/rMQR/JAB/CCTag; OpenSCAD lazy-union; AprilTag/ArUco alone | rejected: reasons in D1 and Research |
| markers-research.md (other entries) | decoder, PnP, POI model, detector items | rejected here: delivered by printable-metadata-markers-t |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 Symbology QR Model 2 | marker-qr-symbol |
| D2 Payload P1 codec | marker-payload-codec |
| D3 Side length and marker frame | marker-payload-codec |
| D4 Generator package and config | marker-qr-symbol, markers-generate-cli |
| D5 PDF output | marker-pdf-output |
| D6 Two-colour STL and 3MF | marker-stl-3mf-output |
| D7 ChArUco board | charuco-board-output, markers-generate-cli |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `marker-payload` | marker-payload-codec | marker-qr-symbol | `shared/ros2_common/marker_payload.py` |
| `markers-config` | marker-qr-symbol | marker-pdf-output, marker-stl-3mf-output, charuco-board-output, markers-generate-cli | `scripts/markers/config.py` |
| `markers-symbol` | marker-qr-symbol | marker-pdf-output, marker-stl-3mf-output, markers-generate-cli | `scripts/markers/symbol.py` |
| `markers-pdf` | marker-pdf-output | charuco-board-output, markers-generate-cli | `scripts/markers/pdf.py` |
| `markers-mesh` | marker-stl-3mf-output | charuco-board-output, markers-generate-cli | `scripts/markers/mesh.py` |
| `charuco-spec` | charuco-board-output | markers-generate-cli | `scripts/markers/charuco.py` |

`marker-payload` is also consumed across epics by printable-metadata-markers-t (marker-decode, poi-marker-model,
marker-map-pose-poi), and `charuco-spec` by head-rgb-d-fusion-of-camera (rgbd-calibration-frames equality test).

## Review focus

- Physical size checks measure the rendered raster, not layout numbers.
- segno `boost_error=False`; canonical payload order; mesh bodies watertight and disjoint.

## Left to the builder

- Mesh construction method (manifold3d unions or merged-run extrusion) within D6's checks.
- Internal helper structure of `scripts/markers/`.

## Assumptions and unknowns

- trimesh writes multi-object 3MF and Bambu Studio honours `Metadata/model_settings.config`: the structure is tested;
  the filament assignment is confirmed by the user in Bambu Studio (printable-metadata-markers-t runbook step 1).

## Irreversible operations in scope

None.
