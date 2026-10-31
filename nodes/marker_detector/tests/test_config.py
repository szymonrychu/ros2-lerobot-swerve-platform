"""Tests for the marker_detector config models, the retired zoom stage and the venv's OpenCV."""

import subprocess
from pathlib import Path

import cv2
import pytest
from pydantic import ValidationError

from marker_detector.config import MarkerDetectorConfig, config_path, load_config

NODE_DIR = Path(__file__).resolve().parent.parent
ZOOM_MODULE = "marker_detector/zoom.py"
ZOOM_TESTS = "tests/test_zoom.py"


def write(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "config.yaml"
    path.write_text(text)
    return path


def test_defaults_from_empty_yaml(tmp_path: Path) -> None:
    cfg = load_config(write(tmp_path, ""))
    assert cfg.topics.detections == "/markers/detections"
    assert cfg.process_hz == 2.0
    assert cfg.map_frame == "map"
    assert cfg.pose.ambiguity_ratio == 2.0
    assert cfg.depth_plane.max_residual_mm == 10
    assert cfg.metrics_port is None


def test_default_camera_is_the_d435_head(tmp_path: Path) -> None:
    cfg = load_config(write(tmp_path, ""))
    assert [c.model_dump() for c in cfg.cameras] == [
        {
            "name": "head",
            "image": "/head/d435/color/image_raw/compressed",
            "camera_info": "/head/d435/color/camera_info",
            "depth_plane": True,
        }
    ]
    assert cfg.topics.depth == "/head_rgbd/depth/image_raw"
    assert cfg.topics.depth_info == "/head_rgbd/depth/camera_info"


@pytest.mark.parametrize("key", ["image", "camera_info"])
def test_topics_image_and_camera_info_are_rejected(tmp_path: Path, key: str) -> None:
    with pytest.raises(ValidationError, match=key):
        load_config(write(tmp_path, f"topics:\n  {key}: /x\n"))


def camera(name: str, depth_plane: bool = False) -> str:
    return (
        f"  - name: {name}\n    image: /{name}/image_raw/compressed\n    camera_info: /{name}/camera_info\n"
        f"    depth_plane: {str(depth_plane).lower()}\n"
    )


def test_cameras_list_validates(tmp_path: Path) -> None:
    cfg = load_config(write(tmp_path, "cameras:\n" + camera("head", True) + camera("front") + camera("back")))
    assert [(c.name, c.image, c.camera_info, c.depth_plane) for c in cfg.cameras] == [
        ("head", "/head/image_raw/compressed", "/head/camera_info", True),
        ("front", "/front/image_raw/compressed", "/front/camera_info", False),
        ("back", "/back/image_raw/compressed", "/back/camera_info", False),
    ]
    with pytest.raises(ValidationError, match="unique"):
        load_config(write(tmp_path, "cameras:\n" + camera("front") + camera("front")))
    with pytest.raises(ValidationError, match="depth_plane"):
        load_config(write(tmp_path, "cameras:\n" + camera("head", True) + camera("front", True)))
    with pytest.raises(ValidationError, match="bogus"):
        load_config(write(tmp_path, "cameras:\n" + camera("head") + "    bogus: 1\n"))
    with pytest.raises(ValidationError, match="cameras"):
        load_config(write(tmp_path, "cameras: []\n"))
    with pytest.raises(ValidationError, match="name"):
        load_config(write(tmp_path, "cameras:\n  - image: /a\n    camera_info: /b\n"))


def test_tof_section_rejected(tmp_path: Path) -> None:
    with pytest.raises(ValidationError, match="tof"):
        load_config(write(tmp_path, "tof:\n  enabled: true\n"))


def test_full_yaml(tmp_path: Path) -> None:
    cfg = load_config(
        write(
            tmp_path,
            "process_hz: 5\nmetrics_port: 9100\npose:\n  min_module_px: 5\n",
        )
    )
    assert cfg.process_hz == 5
    assert cfg.metrics_port == 9100
    assert cfg.pose.min_module_px == 5
    assert isinstance(cfg, MarkerDetectorConfig)


@pytest.mark.parametrize(
    ("text", "key"),
    [
        ("zoom:\n  auto_zoom: false\n", "zoom"),
        ("topics:\n  zoom_command: /head_camera/zoom/command\n", "zoom_command"),
        ("topics:\n  zoom_image: /head_camera/zoom/image\n", "zoom_image"),
        ("topics:\n  zoom_result: /head_camera/zoom/result\n", "zoom_result"),
    ],
)
def test_zoom_keys_are_rejected(tmp_path: Path, text: str, key: str) -> None:
    with pytest.raises(ValidationError, match=key):
        load_config(write(tmp_path, text))


def test_zoom_module_is_gone() -> None:
    tracked = subprocess.run(
        ["git", "ls-files", ZOOM_MODULE, ZOOM_TESTS], cwd=NODE_DIR, capture_output=True, text=True, check=True
    ).stdout
    assert tracked == ""


def test_unknown_key_rejected_naming_key(tmp_path: Path) -> None:
    with pytest.raises(ValidationError, match="bogus_key"):
        load_config(write(tmp_path, "pose:\n  bogus_key: 1\n"))


def test_process_hz_range(tmp_path: Path) -> None:
    with pytest.raises(ValidationError, match="process_hz"):
        load_config(write(tmp_path, "process_hz: 50\n"))


def test_env_override(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.delenv("MARKER_DETECTOR_CONFIG", raising=False)
    assert config_path() == Path("/etc/ros2/marker_detector/config.yaml")
    monkeypatch.setenv("MARKER_DETECTOR_CONFIG", str(tmp_path / "x.yaml"))
    assert config_path() == tmp_path / "x.yaml"


def test_venv_opencv_is_modern() -> None:
    major, minor = (int(x) for x in cv2.__version__.split(".")[:2])
    assert (major, minor) >= (4, 8)
    assert hasattr(cv2, "QRCodeDetectorAruco")
    assert hasattr(cv2.aruco, "CharucoDetector")
    import zxingcpp

    assert zxingcpp.__name__ == "zxingcpp"
