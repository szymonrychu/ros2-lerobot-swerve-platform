"""Tests for the head RGB-D calibration core in shared/ros2_common/head_rgbd_calibration."""

import ast
import difflib
import subprocess
import sys
from collections.abc import Iterator
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import yaml
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration import charuco as core_charuco
from ros2_common.head_rgbd_calibration.calibrate import (
    CalibrationRun,
    PoseView,
    calibrate_detections,
    detect_tof,
    failed_limits,
    report_markdown,
)
from ros2_common.head_rgbd_calibration.charuco import detect, make_board
from ros2_common.head_rgbd_calibration.config import BoardConfig, LimitsConfig, SelectionConfig
from ros2_common.head_rgbd_calibration.extrinsics import ExtrinsicResult, PlaneCheck, stereo_calibrate
from ros2_common.head_rgbd_calibration.frames import (
    CameraInfoData,
    FrameSet,
    StampedImage,
    decode_camera_info,
    decode_compressed,
    decode_mono16,
    rgb_difference,
    stamp_ns,
    still_pairs,
)
from ros2_common.head_rgbd_calibration.intrinsics import KIND_RGB, KIND_TOF, IntrinsicsResult, calibrate
from ros2_common.head_rgbd_calibration.urdf_text import NOTE_MARKER, Origin, rewrite_tof_joint

from scripts.head_rgbd_calibration.config import DEFAULT_CONFIG, load_calibration_config
from scripts.head_rgbd_calibration.pipeline import run_calibration
from tests.rgbd_synthetic import board_poses, render_rgb, rgb_camera
from tests.test_head_rgbd_calib_extrinsics import BOARD, RGB, TOF, TRUE, detection_pairs, rotation_error_deg
from tests.test_head_rgbd_calibrate_cli import RGB_SIZE_RC, TOF_SIZE_RC, scene

ROOT = Path(__file__).resolve().parent.parent
CORE_DIR = ROOT / "shared" / "ros2_common" / "head_rgbd_calibration"
URDF_TEXT = CORE_DIR / "urdf_text.py"
HEAD_URDF = ROOT / "nodes" / "web_ui" / "urdf" / "head.urdf"
FIXED_ORIGIN = Origin(xyz=(0.01, -0.02, 0.03), rpy=(0.1, -0.2, 0.3))
CALIBRATE_YAML = ROOT / "scripts" / "head_rgbd_calibrate.yaml"
FORBIDDEN_MODULES = {"scripts", "rosbags", "subprocess"}


def imported_roots(path: Path) -> set[str]:
    """Top-level module names imported by a Python file."""
    roots: set[str] = set()
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Import):
            roots.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            roots.add(node.module.split(".")[0])
    return roots


def test_core_imports_no_scripts_rosbags_or_subprocess() -> None:
    files = sorted(CORE_DIR.glob("*.py"))
    assert {f.name for f in files} >= {"__init__.py", "config.py", "charuco.py", "frames.py", "intrinsics.py"}
    for f in files:
        assert not imported_roots(f) & FORBIDDEN_MODULES, f.name


@pytest.mark.parametrize(
    ("model", "block"), [(BoardConfig, "board"), (SelectionConfig, "selection"), (LimitsConfig, "limits")]
)
def test_core_config_defaults_match_cli_yaml(model: type, block: str) -> None:
    with pytest.raises(ValidationError):
        model(bogus=1)
    yaml_block = yaml.safe_load(CALIBRATE_YAML.read_text())[block]
    assert model.model_validate(yaml_block) == model()


def test_core_detect_finds_synthetic_board() -> None:
    cfg = BoardConfig()
    camera = rgb_camera()
    image = render_rgb(camera, board_poses(1, camera, cfg)[0], cfg)
    detection = core_charuco.detect(image, core_charuco.make_board(cfg))
    assert detection is not None
    assert len(detection.ids) >= core_charuco.MIN_CORNERS
    assert detection.points.shape == (len(detection.ids), 2)
    assert np.isfinite(detection.points).all()


NS = 1_000_000_000
FLICKER_AMPLITUDE = 0.05
FLICKER_PERIOD_ROWS = 40
INTRINSICS_VIEWS = 20
INTRINSICS_DISTANCE_M = (0.4, 0.9)


def textured_frame(seed: int, shape: tuple[int, int] = (120, 160)) -> np.ndarray:
    """A smooth random texture as uint8 BGR."""
    noise = np.random.default_rng(seed).uniform(60, 180, shape).astype(np.float32)
    grey = cv2.GaussianBlur(noise, (0, 0), 3)
    return cv2.cvtColor(np.rint(grey).astype(np.uint8), cv2.COLOR_GRAY2BGR)


def test_core_still_pairs_one_pair_per_window() -> None:
    still_a, still_b = textured_frame(1), textured_frame(2)
    amp_still = np.full((18, 24), 1000, dtype=np.uint16)
    amp_still[:, :12] = 3000
    # stills at 2-5 s (scene A) and 8-11 s (scene B); one different frame between them, 10 Hz ToF, 2 Hz RGB
    rgb = []
    for i in range(24):
        t = i * 0.5
        image = still_a if 2 <= t < 5 else still_b if 8 <= t < 11 else textured_frame(100 + i)
        rgb.append(StampedImage(int(t * NS) + 5_000_000, image))
    amplitude = []
    for i in range(120):
        t = i * 0.1
        moving = not (2 <= t < 5 or 8 <= t < 11)
        amp = amp_still if not moving else np.roll(amp_still, i % 12, axis=1)
        amplitude.append(StampedImage(int(t * NS), amp))
    pairs = still_pairs(FrameSet(rgb=rgb, amplitude=amplitude), SelectionConfig())
    assert len(pairs) == 2
    assert 2 * NS <= pairs[0].rgb_stamp_ns < 5 * NS
    assert 8 * NS <= pairs[1].rgb_stamp_ns < 11 * NS


def test_core_rgb_difference_ignores_flicker_bands() -> None:
    a = cv2.cvtColor(textured_frame(3), cv2.COLOR_BGR2GRAY)
    rows = np.arange(a.shape[0])
    gain = 1 + FLICKER_AMPLITUDE * np.sin(2 * np.pi * rows / FLICKER_PERIOD_ROWS)
    b = np.clip(np.rint(a * gain[:, None]), 0, 255).astype(np.uint8)
    assert np.mean(np.abs(a.astype(float) - b.astype(float))) > 3
    assert rgb_difference(a, b) < 1.0


def test_core_intrinsics_recover_synthetic_camera() -> None:
    cfg = BoardConfig()
    camera = rgb_camera()
    board = core_charuco.make_board(cfg)
    poses = board_poses(INTRINSICS_VIEWS, camera, cfg, seed=3, distance_m=INTRINSICS_DISTANCE_M)
    found = [core_charuco.detect(render_rgb(camera, pose, cfg, seed=i), board) for i, pose in enumerate(poses)]
    detections = [d for d in found if d is not None]
    assert len(detections) >= 18
    result = calibrate(detections, board, camera.size)
    assert result.k[0, 0] == pytest.approx(camera.k[0, 0], rel=0.01)
    assert result.k[1, 1] == pytest.approx(camera.k[1, 1], rel=0.01)
    assert abs(result.k[0, 2] - camera.k[0, 2]) < 3
    assert abs(result.k[1, 2] - camera.k[1, 2]) < 3
    assert result.rms < 0.5


def test_urdf_text_is_stdlib_only() -> None:
    assert imported_roots(URDF_TEXT) <= set(sys.stdlib_module_names)


def test_urdf_text_loads_by_path_without_numpy() -> None:
    code = (
        "import importlib.util, sys\n"
        f"spec = importlib.util.spec_from_file_location('urdf_text', {str(URDF_TEXT)!r})\n"
        "module = importlib.util.module_from_spec(spec)\n"
        "sys.modules['urdf_text'] = module\n"
        "spec.loader.exec_module(module)\n"
        "assert 'numpy' not in sys.modules\n"
        "assert callable(module.rewrite_tof_joint)\n"
    )
    subprocess.run([sys.executable, "-I", "-c", code], check=True)


def test_core_rewrite_tof_joint_touches_only_tof_joint() -> None:
    before = HEAD_URDF.read_text()
    after = rewrite_tof_joint(before, FIXED_ORIGIN, "2026-01-01 rms 0.1")
    diff = [
        line
        for line in difflib.unified_diff(before.splitlines(), after.splitlines(), lineterm="", n=0)
        if line[0] in "+-" and not line.startswith(("+++", "---"))
    ]
    assert diff
    assert all("<origin" in line or NOTE_MARKER in line for line in diff)
    assert sum(NOTE_MARKER in line for line in diff) <= 2
    assert "0.010000000 -0.020000000 0.030000000" in after


def test_core_stereo_extrinsic_recovers_synthetic_pose() -> None:
    result = stereo_calibrate(detection_pairs(), BOARD, RGB, TOF)
    assert np.linalg.norm(result.t - TRUE.t) < 0.002
    assert rotation_error_deg(result.r, TRUE.r) < 0.2


T0_NS = 1_700_000_000 * 10**9
TOF_HZ = 10
RGB_EVERY = 5
START_S = 2.5
CYCLE_S = 3.0
WINDOW_S = 2.0
FIT_TOL = 1e-9
SDK_K = np.array([191.0, 0.0, 120.0, 0.0, 191.0, 90.0, 0.0, 0.0, 1.0])
GOLDEN_REPORT = """# Head RGB-D calibration calib-test

Date: 2026-01-02T03:04:05
Head camera lens position: 4.5 dioptres (changing lens_position invalidates the RGB calibration)
Still pairs: 15

| Item | Value | Limit |
|---|---|---|
| RGB intrinsic RMS | 0.250 px | 0.5 px |
| RGB views | 14 | >= 12 |
| ToF intrinsic RMS | 0.100 px | 0.3 px |
| ToF views | 13 | >= 12 |
| ToF zero-distortion RMS | 0.120 px | 0.3 px |
| Stereo RMS | 0.600 px | 1.0 px |
| Stereo pairs | 12 | >= 12 |
| Board plane depth bias (median) | -3.5 mm | 15.0 mm |

ToF intrinsics versus the SDK /tof/camera_info (percent difference):

- fx: +0.50 %
- fy: -0.25 %

Extrinsic (tof_optical_frame in head_camera_optical_frame), metres: 0.01000, -0.02000, 0.03000
"""


def scene_frames() -> FrameSet:
    """The CLI synthetic scene in memory: one still window per view, moving noise between windows."""
    views = scene()
    rng = np.random.default_rng(0)
    noise_depth = views[0][2]
    frames = FrameSet(tof_info=CameraInfoData(240, 180, "plumb_bob", np.zeros(5), SDK_K, np.zeros(12)))
    for i in range(int((START_S + len(views) * CYCLE_S) * TOF_HZ)):
        t = i / TOF_HZ
        ns = T0_NS + int(t * 1e9)
        index = int((t - START_S) // CYCLE_S) if t >= START_S else -1
        still = index >= 0 and (t - START_S) - index * CYCLE_S < WINDOW_S
        if still:
            rgb, amp, depth = views[index]
        else:
            rgb = rng.integers(0, 255, RGB_SIZE_RC + (3,), dtype=np.uint8)
            amp = rng.integers(500, 4000, TOF_SIZE_RC, dtype=np.uint16)
            depth = noise_depth
        frames.amplitude.append(StampedImage(ns, amp))
        frames.depth.append(StampedImage(ns, depth))
        if i % RGB_EVERY == 0:
            frames.rgb.append(StampedImage(ns + 5_000_000, rgb))
    return frames


def fixed_run() -> CalibrationRun:
    """A hand-built run for the report golden."""
    k = np.eye(3)
    rgb = IntrinsicsResult(k, np.zeros(5), 0.25, np.zeros(14), 14, (576, 324), KIND_RGB)
    tof = IntrinsicsResult(k, np.zeros(5), 0.1, np.zeros(13), 13, (240, 180), KIND_TOF, zero_distortion_rms=0.12)
    extrinsic = ExtrinsicResult(np.eye(3), np.array([0.01, -0.02, 0.03]), 0.6, 12)
    return CalibrationRun(rgb, tof, extrinsic, PlaneCheck([-3.5], -3.5), {"fx": 0.5, "fy": -0.25}, 15)


@pytest.fixture
def single_thread_cv2() -> Iterator[None]:
    """Multithreaded cv2 reductions are not bit-reproducible; one thread makes two fits comparable at 1e-9."""
    previous = cv2.getNumThreads()
    cv2.setNumThreads(1)
    yield
    cv2.setNumThreads(previous)


@pytest.mark.usefixtures("single_thread_cv2")
def test_calibrate_detections_matches_run_calibration() -> None:
    cfg = load_calibration_config(DEFAULT_CONFIG)
    frames = scene_frames()
    expected = run_calibration(frames, cfg)
    board = make_board(cfg.board)
    pairs = still_pairs(frames, cfg.selection)
    views = [PoseView(detect(p.rgb, board), detect_tof(p.amplitude, board, cfg.selection), p.depth) for p in pairs]
    rgb_size = (pairs[0].rgb.shape[1], pairs[0].rgb.shape[0])
    tof_size = (pairs[0].amplitude.shape[1], pairs[0].amplitude.shape[0])
    run = calibrate_detections(views, rgb_size, tof_size, frames.tof_info, cfg.board, cfg.limits)
    assert run.pair_count == expected.pair_count == len(views)
    assert run.rgb.view_count == expected.rgb.view_count
    assert run.tof.view_count == expected.tof.view_count
    assert run.extrinsic.pair_count == expected.extrinsic.pair_count
    for got, want in ((run.rgb, expected.rgb), (run.tof, expected.tof)):
        assert abs(got.rms - want.rms) < FIT_TOL
        np.testing.assert_allclose(got.k, want.k, atol=FIT_TOL, rtol=0)
    assert abs(run.extrinsic.stereo_rms - expected.extrinsic.stereo_rms) < FIT_TOL
    np.testing.assert_allclose(run.extrinsic.t, expected.extrinsic.t, atol=1e-9, rtol=0)
    assert abs(run.plane.median_mm - expected.plane.median_mm) < FIT_TOL
    assert run.sdk_diff_pct == pytest.approx(expected.sdk_diff_pct, abs=FIT_TOL)


def test_calibrate_detections_requires_tof_info() -> None:
    cfg = load_calibration_config(DEFAULT_CONFIG)
    with pytest.raises(ValueError, match="no ToF camera_info"):
        calibrate_detections([], (1, 1), (1, 1), None, cfg.board, cfg.limits)


def test_failed_limits_names_limit_keys() -> None:
    run = fixed_run()
    assert failed_limits(run, LimitsConfig()) == []
    tight = LimitsConfig(rgb_max_rms_px=0.1, tof_max_rms_px=0.05, stereo_max_rms_px=0.5)
    failures = failed_limits(run, tight)
    assert [f.split(":")[0] for f in failures] == [
        "limits.rgb_max_rms_px",
        "limits.tof_max_rms_px",
        "limits.stereo_max_rms_px",
    ]
    assert failures[0] == "limits.rgb_max_rms_px: RGB RMS 0.250 px exceeds 0.1 px"


def test_failed_limits_flags_plane_bias_over_limit() -> None:
    run = fixed_run()
    assert LimitsConfig().plane_max_bias_mm == 15.0
    assert failed_limits(run, LimitsConfig(plane_max_bias_mm=3.5)) == []
    failures = failed_limits(run, LimitsConfig(plane_max_bias_mm=3.0))
    assert failures == ["limits.plane_max_bias_mm: plane depth bias -3.5 mm exceeds 3.0 mm"]
    over = replace(run, plane=PlaneCheck([20.0], 20.0))
    assert [f.split(":")[0] for f in failed_limits(over, LimitsConfig())] == ["limits.plane_max_bias_mm"]
    under = replace(run, plane=PlaneCheck([-20.0], -20.0))
    assert [f.split(":")[0] for f in failed_limits(under, LimitsConfig())] == ["limits.plane_max_bias_mm"]


def test_report_markdown_golden() -> None:
    text = report_markdown(fixed_run(), LimitsConfig(), "calib-test", 4.5, "2026-01-02T03:04:05")
    assert text == GOLDEN_REPORT


def test_moved_modules_left_scripts() -> None:
    """The four transitional re-export modules are gone and nothing imports them."""
    root = Path(__file__).resolve().parent.parent
    paths = [
        f"scripts/head_rgbd_calibration/{name}.py" for name in ("charuco", "intrinsics", "extrinsics", "urdf_handoff")
    ]
    tracked = subprocess.run(["git", "ls-files", *paths], cwd=root, capture_output=True, text=True, check=True)
    assert tracked.stdout.strip() == ""
    pattern = r"scripts\.head_rgbd_calibration(\.| import )(charuco|intrinsics|extrinsics|urdf_handoff)\b"
    found = subprocess.run(
        ["git", "grep", "-nE", pattern, "--", "*.py", ":!tests/test_head_rgbd_calib_core.py"],
        cwd=root,
        capture_output=True,
        text=True,
    )
    assert found.returncode == 1, found.stdout


def stamped_msg(sec: int = 3, nanosec: int = 250, **fields: object) -> SimpleNamespace:
    """A duck-typed message with a header stamp and the given fields."""
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)), **fields)


def mono16_msg(image: np.ndarray, bigendian: bool, pad: int = 0, encoding: str = "mono16") -> SimpleNamespace:
    """A sensor_msgs/Image look-alike holding a uint16 image with pad bytes of row padding."""
    rows = image.astype(">u2" if bigendian else "<u2").view(np.uint8).reshape(image.shape[0], -1)
    rows = np.pad(rows, ((0, 0), (0, pad)))
    return stamped_msg(
        height=image.shape[0],
        width=image.shape[1],
        step=rows.shape[1],
        is_bigendian=int(bigendian),
        encoding=encoding,
        data=rows.tobytes(),
    )


def test_stamp_ns_combines_sec_and_nanosec() -> None:
    assert stamp_ns(stamped_msg(sec=3, nanosec=250)) == 3_000_000_250


@pytest.mark.parametrize("bigendian", [False, True])
@pytest.mark.parametrize("encoding", ["mono16", "16UC1"])
def test_decode_mono16_both_endiannesses_and_step(bigendian: bool, encoding: str) -> None:
    image = np.arange(12, dtype=np.uint16).reshape(3, 4) * 1000 + 7
    decoded = decode_mono16(mono16_msg(image, bigendian, pad=6, encoding=encoding))
    assert decoded.dtype == np.uint16
    np.testing.assert_array_equal(decoded, image)


def test_decode_mono16_rejects_other_encoding_and_short_data() -> None:
    image = np.ones((2, 2), dtype=np.uint16)
    with pytest.raises(ValueError, match="encoding"):
        decode_mono16(mono16_msg(image, False, encoding="rgb8"))
    short = mono16_msg(image, False)
    short.data = short.data[:-2]
    with pytest.raises(ValueError, match="do not hold"):
        decode_mono16(short)


def test_decode_compressed_returns_bgr_and_names_topic_on_failure() -> None:
    bgr = np.full((8, 8, 3), (10, 120, 200), dtype=np.uint8)
    ok, png = cv2.imencode(".png", bgr)
    assert ok
    np.testing.assert_array_equal(decode_compressed(stamped_msg(data=png.tobytes()), "/rgb"), bgr)
    with pytest.raises(ValueError, match="/rgb") as err:
        decode_compressed(stamped_msg(data=b"not an image"), "/rgb")
    assert "3000000250" in str(err.value)


def test_decode_camera_info_reduces_fields() -> None:
    msg = SimpleNamespace(
        width=640,
        height=480,
        distortion_model="plumb_bob",
        d=[0.1, 0.0, 0.0, 0.0, 0.0],
        k=list(range(9)),
        p=list(range(12)),
    )
    info = decode_camera_info(msg)
    assert (info.width, info.height, info.distortion_model) == (640, 480, "plumb_bob")
    assert info.k.shape == (9,) and info.p.shape == (12,) and info.d.dtype == np.float64


def test_core_decoders() -> None:
    """Core decoders handle mono16 (both endiannesses), compressed images, camera_info and stamps."""
    assert stamp_ns(stamped_msg(sec=1, nanosec=2)) == 1_000_000_002
    image = np.arange(6, dtype=np.uint16).reshape(2, 3) + 300
    for bigendian in (False, True):
        np.testing.assert_array_equal(decode_mono16(mono16_msg(image, bigendian)), image)
    bgr = np.full((4, 4, 3), (10, 120, 200), dtype=np.uint8)
    ok, png = cv2.imencode(".png", bgr)
    assert ok
    np.testing.assert_array_equal(decode_compressed(stamped_msg(data=png.tobytes())), bgr)
    info = decode_camera_info(
        SimpleNamespace(width=4, height=2, distortion_model="plumb_bob", d=[0.0], k=[0.0] * 9, p=[0.0] * 12)
    )
    assert (info.width, info.height) == (4, 2)
