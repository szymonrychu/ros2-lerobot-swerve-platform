"""Tests for the head RGB-D calibration core in shared/ros2_common/head_rgbd_calibration."""

import ast
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from ros2_common.head_rgbd_calibration import charuco as core_charuco
from ros2_common.head_rgbd_calibration.config import BoardConfig
from ros2_common.head_rgbd_calibration.frames import (
    decode_camera_info,
    decode_compressed,
    decode_mono16,
    rgb_difference,
    stamp_ns,
)
from ros2_common.head_rgbd_calibration.intrinsics import calibrate

from tests.rgbd_synthetic import board_poses, render_rgb, rgb_camera

ROOT = Path(__file__).resolve().parent.parent
CORE_DIR = ROOT / "shared" / "ros2_common" / "head_rgbd_calibration"
URDF_TEXT = CORE_DIR / "urdf_text.py"
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
        "assert callable(module.rewrite_joint_origin)\n"
    )
    subprocess.run([sys.executable, "-I", "-c", code], check=True)


def test_moved_modules_left_scripts() -> None:
    """The four transitional re-export modules are gone and nothing imports them."""
    root = Path(__file__).resolve().parent.parent
    package = Path("scripts") / "head_rgbd_calibration"
    paths = [str(package / f"{name}.py") for name in ("charuco", "intrinsics", "extrinsics", "urdf_handoff")]
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
