"""End-to-end steps of scripts/head_rgbd_calibrate.py: calibration run, output files, report, validation session."""

import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

from scripts.head_rgbd_calibration.charuco import Detection, detect, make_board
from scripts.head_rgbd_calibration.config import CalibrationConfig, HeadCameraConfig
from scripts.head_rgbd_calibration.extrinsics import (
    ExtrinsicResult,
    PlaneCheck,
    load_extrinsics_yaml,
    plane_depth_bias,
    stereo_calibrate,
    write_extrinsics_yaml,
)
from scripts.head_rgbd_calibration.frames import (
    NS_PER_S,
    FrameSet,
    TopicNames,
    normalise_amplitude,
    read_frames,
    still_pairs,
)
from scripts.head_rgbd_calibration.intrinsics import (
    KIND_RGB,
    KIND_TOF,
    IntrinsicsResult,
    calibrate,
    calibrate_tof,
    compare_sdk,
    write_camera_calibration_yaml,
)
from scripts.head_rgbd_calibration.validate import (
    RgbSample,
    TargetResult,
    TofSample,
    ValidationResults,
    check_targets,
    depth_accuracy,
    edge_alignment,
    registration_bias,
    time_offset,
    valid_ratio,
)

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
METADATA = "metadata.yaml"
VALIDATION_SUFFIX_RE = re.compile(r"-(\d+\.\d+m|sweep|wall)$")
SWEEP_SUFFIX = "sweep"
WALL_SUFFIX = "wall"
REGISTERED_TOPICS = TopicNames(
    rgb="/head_rgbd/rgb/image_raw/compressed",
    rgb_info="/head_rgbd/rgb/camera_info",
    depth="/head_rgbd/depth/image_raw",
    tof_info="/head_rgbd/depth/camera_info",
)
MEASURED_ELSEWHERE = ("motion edge alignment", "CPU p95", "placeholder messages")
CALIBRATION_TARGETS = ("RGB intrinsic RMS", "ToF intrinsic RMS", "stereo RMS", "plane depth bias")
WALL_TARGETS = ("registration bias", "edge alignment", "valid-pixel ratio")
SWEEP_TARGETS = ("time offset",)
MM_PER_M = 1000.0
DEPTH_COLUMNS = 3


def resolve_path(path: str) -> Path:
    """A config path as absolute: relative paths are relative to the repository root."""
    p = Path(path)
    return p if p.is_absolute() else REPO_ROOT / p


def newest_session(local_dir: Path) -> str | None:
    """Name of the newest calibration bag (a directory with metadata.yaml, validation bags excluded) or None."""
    bags = [
        d
        for d in local_dir.glob("*")
        if d.is_dir() and (d / METADATA).exists() and not VALIDATION_SUFFIX_RE.search(d.name)
    ]
    return max(bags, key=lambda d: (d.stat().st_mtime_ns, d.name)).name if bags else None


def session_name(cfg: CalibrationConfig) -> str:
    """The configured session, else the newest calibration bag under capture.local_dir.

    Raises:
        FileNotFoundError: neither is available.
    """
    local_dir = resolve_path(cfg.capture.local_dir)
    name = cfg.session or newest_session(local_dir)
    if name is None:
        raise FileNotFoundError(f"no calibration bag under {local_dir}; run the capture mode or set session")
    return name


def bag_path(cfg: CalibrationConfig, name: str) -> Path:
    """Bag directory name under capture.local_dir, which must exist."""
    path = resolve_path(cfg.capture.local_dir) / name
    if not path.is_dir():
        raise FileNotFoundError(f"bag {path} not found")
    return path


def lens_position(head: HeadCameraConfig) -> float:
    """lens_position from the head camera node config of head.config_file (its default when the node sets none).

    Raises:
        FileNotFoundError: the vars file does not exist.
        ValueError: the node has no ros2_nodes entry.
    """
    data = yaml.safe_load(resolve_path(head.config_file).read_text())
    entry = next((n for n in data.get("ros2_nodes", []) if n.get("name") == head.node), None)
    if entry is None:
        raise ValueError(f"{head.config_file} has no ros2_nodes entry {head.node}")
    node_cfg = yaml.safe_load(entry.get("config") or "") or {}
    return float(node_cfg.get("lens_position", head.default_lens_position))


def load_intrinsics_yaml(path: Path, kind: str) -> IntrinsicsResult:
    """Read a ROS camera_calibration YAML written by write_camera_calibration_yaml (RMS is not stored: 0)."""
    data = yaml.safe_load(path.read_text())
    return IntrinsicsResult(
        k=np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3),
        d=np.array(data["distortion_coefficients"]["data"], dtype=np.float64),
        rms=0.0,
        per_view_errors=np.zeros(0),
        view_count=0,
        image_size=(int(data["image_width"]), int(data["image_height"])),
        kind=kind,
    )


@dataclass(frozen=True)
class CalibrationRun:
    """Everything calibrate computes before any file is written."""

    rgb: IntrinsicsResult
    tof: IntrinsicsResult
    extrinsic: ExtrinsicResult
    plane: PlaneCheck
    sdk_diff_pct: dict[str, float]
    pair_count: int


def run_calibration(frames: FrameSet, cfg: CalibrationConfig) -> CalibrationRun:
    """Detect the board in the still pairs and fit RGB intrinsics, ToF intrinsics and the extrinsic.

    Raises:
        ValueError: no ToF camera_info, too few views or pairs (the message names the count), or no plane measurement.
    """
    if frames.tof_info is None:
        raise ValueError("the bag has no ToF camera_info; cannot compare with the SDK")
    pairs = still_pairs(frames, cfg.selection)
    board = make_board(cfg.board)
    rgb_dets: list[Detection | None] = [detect(p.rgb, board) for p in pairs]
    tof_dets: list[Detection | None] = []
    for p in pairs:
        image = normalise_amplitude(p.amplitude, cfg.selection.amplitude_percentiles)
        tof_dets.append(None if image is None else detect(image, board))
    rgb_size = (pairs[0].rgb.shape[1], pairs[0].rgb.shape[0]) if pairs else (0, 0)
    tof_size = (pairs[0].amplitude.shape[1], pairs[0].amplitude.shape[0]) if pairs else (0, 0)
    rgb = calibrate([d for d in rgb_dets if d is not None], board, rgb_size, cfg.limits)
    tof = calibrate_tof([d for d in tof_dets if d is not None], board, tof_size, cfg.limits)
    both = [(r, t, p) for r, t, p in zip(rgb_dets, tof_dets, pairs, strict=True) if r is not None and t is not None]
    extrinsic = stereo_calibrate([(r, t) for r, t, _ in both], board, rgb, tof, cfg.limits)
    with_depth = [(r, t, p.depth) for r, t, p in both if p.depth is not None]
    plane = plane_depth_bias(with_depth, board, rgb, tof, extrinsic)
    return CalibrationRun(rgb, tof, extrinsic, plane, compare_sdk(tof, frames.tof_info), len(pairs))


def failed_limits(run: CalibrationRun, cfg: CalibrationConfig) -> list[str]:
    """Descriptions of every exceeded limit, each naming its limits.* key."""
    limits = cfg.limits
    checks = [
        ("limits.rgb_max_rms_px", "RGB RMS", run.rgb.rms, limits.rgb_max_rms_px),
        ("limits.tof_max_rms_px", "ToF RMS", run.tof.rms, limits.tof_max_rms_px),
        ("limits.stereo_max_rms_px", "stereo RMS", run.extrinsic.stereo_rms, limits.stereo_max_rms_px),
    ]
    return [f"{key}: {label} {value:.3f} px exceeds {limit} px" for key, label, value, limit in checks if value > limit]


def report_markdown(run: CalibrationRun, cfg: CalibrationConfig, session: str, lens: float, date: str) -> str:
    """The calibration run report as markdown."""
    lines = [
        f"# Head RGB-D calibration {session}",
        "",
        f"Date: {date}",
        f"Head camera lens position: {lens:g} dioptres (changing lens_position invalidates the RGB calibration)",
        f"Still pairs: {run.pair_count}",
        "",
        "| Item | Value | Limit |",
        "|---|---|---|",
        f"| RGB intrinsic RMS | {run.rgb.rms:.3f} px | {cfg.limits.rgb_max_rms_px} px |",
        f"| RGB views | {run.rgb.view_count} | >= {cfg.limits.min_views} |",
        f"| ToF intrinsic RMS | {run.tof.rms:.3f} px | {cfg.limits.tof_max_rms_px} px |",
        f"| ToF views | {run.tof.view_count} | >= {cfg.limits.min_views} |",
        f"| ToF zero-distortion RMS | {run.tof.zero_distortion_rms:.3f} px | "
        f"{cfg.limits.tof_zero_distortion_rms_px} px |",
        f"| Stereo RMS | {run.extrinsic.stereo_rms:.3f} px | {cfg.limits.stereo_max_rms_px} px |",
        f"| Stereo pairs | {run.extrinsic.pair_count} | >= {cfg.limits.min_views} |",
        f"| Board plane depth bias (median) | {run.plane.median_mm:.1f} mm | |",
        "",
        "ToF intrinsics versus the SDK /tof/camera_info (percent difference):",
        "",
        *[f"- {name}: {value:+.2f} %" for name, value in run.sdk_diff_pct.items()],
        "",
        "Extrinsic (tof_optical_frame in head_camera_optical_frame), metres: "
        + ", ".join(f"{v:.5f}" for v in run.extrinsic.t),
        "",
    ]
    return "\n".join(lines)


def write_calibration(run: CalibrationRun, cfg: CalibrationConfig, session: str, now: datetime) -> list[Path]:
    """Write the three output files and the markdown report; call only after failed_limits is empty.

    Returns:
        The written paths (rgb calibration, ToF calibration, extrinsics, report).
    """
    out = cfg.outputs
    paths = [resolve_path(p) for p in (out.rgb_calibration, out.tof_calibration, out.extrinsics)]
    lens = lens_position(cfg.head_camera)
    date = now.isoformat(timespec="seconds")
    write_camera_calibration_yaml(paths[0], run.rgb, "head_camera", run.rgb.image_size, cfg.limits)
    write_camera_calibration_yaml(paths[1], run.tof, "tof", run.tof.image_size, cfg.limits)
    meta = {
        "session": session,
        "lens_position": lens,
        "board": cfg.board.model_dump(),
        "date": date,
        "rgb_rms_px": float(run.rgb.rms),
        "tof_rms_px": float(run.tof.rms),
        "tof_sdk_diff_pct": run.sdk_diff_pct,
    }
    write_extrinsics_yaml(paths[2], run.extrinsic, run.plane, meta, cfg.limits)
    report = resolve_path(out.report_dir) / f"{session}-calibration.md"
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(report_markdown(run, cfg, session, lens, date))
    return [*paths, report]


def central_window(shape: tuple[int, ...], roi_px: int) -> tuple[int, int, int, int]:
    """(x, y, width, height) of the central roi_px square of an image of the given (rows, cols, ...) shape."""
    return (shape[1] - roi_px) // 2, (shape[0] - roi_px) // 2, roi_px, roi_px


def nearest(items: list, stamp_ns: int):  # noqa: ANN201
    """The item (with .stamp_ns) closest in time to stamp_ns."""
    return min(items, key=lambda item: abs(item.stamp_ns - stamp_ns))


def fov_mask(
    shape: tuple[int, int],
    depth_mm: float,
    tof: IntrinsicsResult,
    rgb: IntrinsicsResult,
    extrinsic: ExtrinsicResult,
    scale: int,
) -> np.ndarray:
    """Pixels of the registered depth image (rgb size / scale) covered by the ToF field of view at depth_mm."""
    width, height = tof.image_size
    corners = np.array([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]], dtype=np.float64)
    rays = cv2.undistortPoints(corners.reshape(-1, 1, 2), tof.k, tof.d).reshape(-1, 2)
    points = np.column_stack([rays, np.ones(4)]) * (depth_mm / MM_PER_M)
    points_rgb = points @ extrinsic.r.T + extrinsic.t
    pixels = cv2.projectPoints(points_rgb, np.zeros(3), np.zeros(3), rgb.k, rgb.d)[0].reshape(-1, 2)
    polygon = np.rint((pixels + 0.5) / scale - 0.5).astype(np.int32)
    mask = np.zeros(shape, dtype=np.uint8)
    cv2.fillConvexPoly(mask, polygon, 1)
    return mask > 0


@dataclass(frozen=True)
class Calibration:
    """Calibration files read back for validation."""

    rgb: IntrinsicsResult
    tof: IntrinsicsResult
    extrinsic: ExtrinsicResult
    meta: dict[str, object]


def load_calibration(cfg: CalibrationConfig) -> Calibration | None:
    """The calibration files of cfg.outputs, or None when the extrinsics file does not exist."""
    out = cfg.outputs
    extrinsics = resolve_path(out.extrinsics)
    if not extrinsics.exists():
        return None
    return Calibration(
        load_intrinsics_yaml(resolve_path(out.rgb_calibration), KIND_RGB),
        load_intrinsics_yaml(resolve_path(out.tof_calibration), KIND_TOF),
        load_extrinsics_yaml(extrinsics),
        yaml.safe_load(extrinsics.read_text()),
    )


def wall_metrics(bag: Path, cfg: CalibrationConfig, calibration: Calibration) -> dict[str, float]:
    """Registration bias, edge alignment median and valid ratio from a bag of the registered output on a wall board.

    Raises:
        ValueError: no registered frames, or the board is never found.
    """
    limit = cfg.validation.frames_per_distance
    raw, registered = read_frames(bag, TopicNames()), read_frames(bag, REGISTERED_TOPICS)
    if not registered.depth or not registered.rgb or not raw.depth:
        raise ValueError(f"{bag.name} has no registered depth, registered rgb or raw depth frames")
    board = make_board(cfg.board)
    depth_frames = registered.depth[:limit]
    shape = depth_frames[0].image.shape
    scale = max(1, round(registered.rgb[0].image.shape[1] / shape[1]))
    medians, ratios = [], []
    for depth in depth_frames:
        rgb = nearest(registered.rgb, depth.stamp_ns).image
        try:
            medians.append(edge_alignment(depth.image, rgb, board, calibration.rgb, scale).median_px)
        except ValueError:
            continue
        valid = depth.image[depth.image > 0]
        if valid.size:
            mask = fov_mask(
                shape, float(np.median(valid)), calibration.tof, calibration.rgb, calibration.extrinsic, scale
            )
            ratios.append(valid_ratio(depth.image, mask))
    if not medians:
        raise ValueError(f"board outline never found with depth steps in {bag.name}")
    raw_images = [f.image for f in raw.depth[:limit]]
    registered_images = [f.image for f in depth_frames]
    roi = central_window(raw_images[0].shape, cfg.validation.roi_px)
    bias = registration_bias(raw_images, registered_images, roi, central_window(shape, cfg.validation.roi_px))
    return {"edge": float(np.median(medians)), "bias": bias, "valid": float(np.mean(ratios)) if ratios else 0.0}


def sweep_offset(bag: Path, cfg: CalibrationConfig, calibration: Calibration) -> float:
    """Median ToF-RGB time offset in ms from a pan-sweep bag (ValueError when no corner pairs qualify)."""
    frames = read_frames(bag, TopicNames())
    if not frames.rgb or not frames.amplitude or not frames.depth:
        raise ValueError(f"{bag.name} lacks rgb, amplitude or depth frames")
    board = make_board(cfg.board)
    t0 = frames.rgb[0].stamp_ns
    rgb_tracks = []
    for frame in frames.rgb:
        detection = detect(frame.image, board)
        if detection is not None:
            rgb_tracks.append(RgbSample((frame.stamp_ns - t0) / NS_PER_S, detection))
    tof_tracks = []
    for frame in frames.amplitude:
        image = normalise_amplitude(frame.image, cfg.selection.amplitude_percentiles)
        detection = None if image is None else detect(image, board)
        if detection is None:
            continue
        depth = nearest(frames.depth, frame.stamp_ns).image
        cols = np.clip(np.rint(detection.points[:, 0]).astype(int), 0, depth.shape[1] - 1)
        rows = np.clip(np.rint(detection.points[:, 1]).astype(int), 0, depth.shape[0] - 1)
        tof_tracks.append(TofSample((frame.stamp_ns - t0) / NS_PER_S, detection, depth[rows, cols] / MM_PER_M))
    return time_offset(rgb_tracks, tof_tracks, calibration.extrinsic, calibration.rgb, calibration.tof).median_ms


def validation_results(cfg: CalibrationConfig, session: str) -> tuple[ValidationResults, set[str]]:
    """Measure everything the available bags allow.

    Returns:
        (results, skipped target names): skipped targets have no input here (measured elsewhere, no calibration
        files, or no wall or sweep bag) and do not gate the exit code.

    Raises:
        FileNotFoundError: a configured distance bag is missing.
        ValueError: a bag lacks the frames its metric needs.
    """
    depth = {}
    for distance in cfg.validation.distances_m:
        frames = read_frames(bag_path(cfg, f"{session}-{distance:.1f}m"), TopicNames()).depth
        if not frames:
            raise ValueError(f"{session}-{distance:.1f}m has no raw ToF depth frames")
        images = [f.image for f in frames[: cfg.validation.frames_per_distance]]
        depth[distance] = depth_accuracy(images, distance, cfg.validation.roi_px)
    skipped = set(MEASURED_ELSEWHERE)
    values: dict[str, float | None] = {}
    calibration = load_calibration(cfg)
    local = resolve_path(cfg.capture.local_dir)
    if calibration is None:
        skipped |= set(CALIBRATION_TARGETS)
    else:
        values = {
            "rgb_rms_px": calibration.meta["rgb_rms_px"],
            "tof_rms_px": calibration.meta["tof_rms_px"],
            "stereo_rms_px": calibration.meta["stereo_rms_px"],
            "plane_bias_mm": calibration.meta["plane_bias_mm_median"],
        }
    wall, sweep = local / f"{session}-{WALL_SUFFIX}", local / f"{session}-{SWEEP_SUFFIX}"
    if (wall.is_dir() or sweep.is_dir()) and calibration is None:
        raise FileNotFoundError("wall and sweep metrics need the calibration files; run the calibrate mode first")
    if wall.is_dir():
        measured = wall_metrics(wall, cfg, calibration)
        values |= {
            "registration_bias_mm": measured["bias"],
            "edge_median_px": measured["edge"],
            "valid_ratio": measured["valid"],
        }
    else:
        skipped |= set(WALL_TARGETS)
    if sweep.is_dir():
        values["time_offset_ms"] = sweep_offset(sweep, cfg, calibration)
    else:
        skipped |= set(SWEEP_TARGETS)
    return ValidationResults(depth=depth, **values), skipped


def target_rows(targets: list[TargetResult], skipped: set[str]) -> list[tuple[str, str, str, str]]:
    """(name, measured, limit, status) per target; status is PASS, FAIL or SKIP."""
    rows = []
    for t in targets:
        sign = ">=" if t.lower_bound else "<="
        measured = "n/a" if t.measured is None else f"{t.measured:.3f} {t.unit}"
        status = "SKIP" if t.name in skipped else ("PASS" if t.passed else "FAIL")
        rows.append((t.name, measured, f"{sign} {t.limit:g} {t.unit}", status))
    return rows


def validation_table(rows: list[tuple[str, str, str, str]]) -> str:
    """Markdown table of target_rows."""
    lines = ["| Target | Measured | Limit | Status |", "|---|---|---|---|"]
    lines += [f"| {name} | {measured} | {limit} | {status} |" for name, measured, limit, status in rows]
    return "\n".join(lines)


def run_validation(cfg: CalibrationConfig, session: str) -> tuple[str, bool]:
    """Compute, tabulate and write the validation of a session.

    Returns:
        (markdown table, True when no gating target failed). The table is also written to
        report_dir/<session>-validation.md.
    """
    results, skipped = validation_results(cfg, session)
    targets = check_targets(results)
    rows = target_rows(targets, skipped)
    table = validation_table(rows)
    report = resolve_path(cfg.outputs.report_dir) / f"{session}-validation.md"
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(f"# Head RGB-D validation {session}\n\n{table}\n")
    return table, all(row[3] != "FAIL" for row in rows)
