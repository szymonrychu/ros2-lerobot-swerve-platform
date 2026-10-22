"""Camera settings in the mcp_server config: intrinsics sources, mount parent frames, arm base offset and reach."""

import logging
import re
from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from mcp_server.camera_scene import CameraNotCalibratedError, camera_setup, intrinsics_cache, warned
from mcp_server.config import REPO_ROOT, McpServerConfig
from mcp_server.head_kinematics import HeadKinematics

MOUNT = {"parent_frame": "gripper_link", "x": 0.0, "y": 0.0, "z": 0.05, "roll": 0.0, "pitch": 0.1, "yaw": 0.0}


def test_default_cameras_are_not_calibrated() -> None:
    cams = McpServerConfig().cameras
    for cam in (cams.gripper, cams.front):
        assert cam.intrinsics is None and cam.mount is None
    assert cams.gripper.parent_frame == "gripper_link"
    assert cams.front.parent_frame == "head_link"  # the front camera is the head RGB camera
    assert str(cams.calibration_dir) == "/var/lib/ros2/camera_calibration"


def test_hfov_intrinsics_need_width_and_height() -> None:
    cfg = McpServerConfig.model_validate(
        {"cameras": {"front": {"intrinsics": {"hfov_deg": 66.0, "width": 640, "height": 480}}}}
    )
    assert cfg.cameras.front.intrinsics is not None and cfg.cameras.front.intrinsics.hfov_deg == 66.0
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"cameras": {"front": {"intrinsics": {"hfov_deg": 66.0}}}})


def test_intrinsics_calibration_file_excludes_hfov() -> None:
    McpServerConfig.model_validate({"cameras": {"front": {"intrinsics": {"calibration_file": "/etc/cam.yaml"}}}})
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate(
            {
                "cameras": {
                    "front": {"intrinsics": {"calibration_file": "/x.yaml", "hfov_deg": 60.0, "width": 1, "height": 1}}
                }
            }
        )
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"cameras": {"front": {"intrinsics": {}}}})


def test_mount_parent_frame_is_adopted_and_must_match() -> None:
    cfg = McpServerConfig.model_validate({"cameras": {"gripper": {"mount": MOUNT}}})
    assert cfg.cameras.gripper.mount is not None and cfg.cameras.gripper.mount.z == 0.05
    with pytest.raises(ValidationError, match="parent_frame"):
        McpServerConfig.model_validate({"cameras": {"gripper": {"parent_frame": "wrist_link", "mount": MOUNT}}})


def test_front_view_camera_must_be_mounted_on_a_head_urdf_link() -> None:
    with pytest.raises(ValidationError, match="head URDF"):
        McpServerConfig.model_validate({"cameras": {"front": {"mount": MOUNT}}})
    ok = McpServerConfig.model_validate({"cameras": {"front": {"mount": {**MOUNT, "parent_frame": "head_link"}}}})
    assert ok.cameras.front.mount is not None and ok.cameras.front.parent_frame == "head_link"


def test_unknown_camera_keys_rejected() -> None:
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"cameras": {"rear": {}}})


def test_arm_base_offset_and_reach_defaults() -> None:
    arm = McpServerConfig().arm
    assert arm.base_in_base_link is not None  # the measured mount
    assert 0.0 < arm.reach_inner_m < arm.reach_outer_m
    cfg = McpServerConfig.model_validate(
        {"arm": {"arm_base_height_m": 0.15, "base_in_base_link": {"x": 0.1, "z": 0.15, "yaw": 3.14}}}
    )
    assert cfg.arm.base_in_base_link is not None and cfg.arm.base_in_base_link.x == 0.1
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"arm": {"reach_inner_m": 0.5, "reach_outer_m": 0.3}})


CLIENT_YML = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
HEAD_CAMERA_DEFAULT_SIZE = (1152, 648)  # nodes/head_camera_cpp/include/head_camera_cpp/config.hpp StreamConfig
CALIB_576 = {
    "image_width": 576,
    "image_height": 324,
    "camera_matrix": {"rows": 3, "cols": 3, "data": [400.0, 0.0, 290.0, 0.0, 410.0, 160.0, 0.0, 0.0, 1.0]},
    "distortion_coefficients": {"rows": 1, "cols": 5, "data": [0.1, -0.2, 0.0, 0.0, 0.05]},
}


def client_node_config(name: str) -> dict:
    nodes = yaml.safe_load(CLIENT_YML.read_text())["ros2_nodes"]
    return yaml.safe_load(next(n for n in nodes if n["name"] == name)["config"])


def front_cfg(path: Path, size: tuple[int, int]) -> McpServerConfig:
    return McpServerConfig.model_validate(
        {
            "cameras": {
                "front": {
                    "parent_frame": "head_camera_link",
                    "intrinsics": {"calibration_file": str(path), "image_width": size[0], "image_height": size[1]},
                    "mount": {
                        "parent_frame": "head_camera_link",
                        "x": 0,
                        "y": 0,
                        "z": 0,
                        "roll": 0,
                        "pitch": 0,
                        "yaw": 0,
                    },
                }
            }
        }
    )


@pytest.fixture(autouse=True)
def fresh_caches() -> None:
    intrinsics_cache.clear()
    warned.clear()


def write_calib(path: Path, data: dict) -> Path:
    path.write_text(yaml.safe_dump(data))
    return path


def test_client_yml_front_block_validates_with_a_head_urdf_parent() -> None:
    cfg = McpServerConfig.model_validate(client_node_config("mcp_server"))
    front = cfg.cameras.front
    assert front.parent_frame == "head_camera_link" and front.mount is not None
    assert HeadKinematics(cfg.head.urdf_path).link_pose(0.0, 0.0, front.parent_frame) is not None
    assert front.intrinsics is not None and front.intrinsics.calibration_file is not None


def test_client_yml_front_image_size_is_the_head_camera_stream() -> None:
    head_cfg = client_node_config("head_camera_cpp")
    size = (head_cfg.get("width", HEAD_CAMERA_DEFAULT_SIZE[0]), head_cfg.get("height", HEAD_CAMERA_DEFAULT_SIZE[1]))
    intr = McpServerConfig.model_validate(client_node_config("mcp_server")).cameras.front.intrinsics
    assert intr is not None and (intr.image_width, intr.image_height) == size
    assert re.search(
        r"int width = 1152;\s+int height = 648;",
        (REPO_ROOT / "nodes/head_camera_cpp/include/head_camera_cpp/config.hpp").read_text(),
    )


def test_front_intrinsics_scaled_to_stream_size(tmp_path: Path) -> None:
    path = write_calib(tmp_path / "c.yaml", CALIB_576)
    intr = camera_setup(front_cfg(path, HEAD_CAMERA_DEFAULT_SIZE), "front").intrinsics
    assert (intr.width, intr.height) == HEAD_CAMERA_DEFAULT_SIZE
    assert (intr.fx, intr.cx, intr.fy, intr.cy) == (800.0, 580.0, 820.0, 320.0)
    assert intr.distortion == [0.1, -0.2, 0.0, 0.0, 0.05]


def test_same_size_keeps_the_calibration(tmp_path: Path) -> None:
    path = write_calib(tmp_path / "c.yaml", CALIB_576)
    intr = camera_setup(front_cfg(path, (576, 324)), "front").intrinsics
    assert (intr.fx, intr.cx, intr.fy, intr.cy) == (400.0, 290.0, 410.0, 160.0)


def test_different_aspect_ratio_is_uncalibrated_and_warns_with_both_sizes(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    path = write_calib(tmp_path / "c.yaml", CALIB_576)
    cfg = front_cfg(path, (640, 480))
    with caplog.at_level(logging.WARNING), pytest.raises(CameraNotCalibratedError):
        camera_setup(cfg, "front")
    with pytest.raises(CameraNotCalibratedError):
        camera_setup(cfg, "front")
    warnings = [r.getMessage() for r in caplog.records if "aspect" in r.getMessage()]
    assert len(warnings) == 1 and "576x324" in warnings[0] and "640x480" in warnings[0]


def test_missing_file_is_uncalibrated_warns_once_and_never_crashes_at_startup(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    cfg = front_cfg(tmp_path / "absent.yaml", HEAD_CAMERA_DEFAULT_SIZE)
    with caplog.at_level(logging.WARNING):
        for _ in range(2):
            with pytest.raises(CameraNotCalibratedError):
                camera_setup(cfg, "front")
    assert len([r for r in caplog.records if "missing" in r.getMessage()]) == 1


def test_an_apply_is_picked_up_without_a_restart(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    cfg = front_cfg(path, (576, 324))
    with pytest.raises(CameraNotCalibratedError):
        camera_setup(cfg, "front")
    write_calib(path, CALIB_576)
    assert camera_setup(cfg, "front").intrinsics.fx == 400.0
    changed = {
        **CALIB_576,
        "camera_matrix": {**CALIB_576["camera_matrix"], "data": [500.0, 0, 290, 0, 410, 160, 0, 0, 1]},
    }
    write_calib(path, changed)
    import os

    os.utime(path, ns=(path.stat().st_atime_ns, path.stat().st_mtime_ns + 10_000_000))
    assert camera_setup(cfg, "front").intrinsics.fx == 500.0


def test_front_intrinsics_missing_or_mismatched_uncalibrated(tmp_path: Path) -> None:
    missing = front_cfg(tmp_path / "absent.yaml", HEAD_CAMERA_DEFAULT_SIZE)
    with pytest.raises(CameraNotCalibratedError):
        camera_setup(missing, "front")
    path = write_calib(tmp_path / "c.yaml", CALIB_576)
    with pytest.raises(CameraNotCalibratedError):
        camera_setup(front_cfg(path, (640, 480)), "front")
