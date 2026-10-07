"""Camera settings in the mcp_server config: intrinsics sources, mount parent frames, arm base offset and reach."""

import pytest
from pydantic import ValidationError

from mcp_server.config import McpServerConfig

MOUNT = {"parent_frame": "gripper_link", "x": 0.0, "y": 0.0, "z": 0.05, "roll": 0.0, "pitch": 0.1, "yaw": 0.0}


def test_default_cameras_are_not_calibrated() -> None:
    cams = McpServerConfig().cameras
    for cam in (cams.gripper, cams.front):
        assert cam.intrinsics is None and cam.mount is None
    assert cams.gripper.parent_frame == "gripper_link"
    assert cams.front.parent_frame == "base_link"
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


def test_front_camera_must_be_mounted_on_base_link() -> None:
    with pytest.raises(ValidationError, match="base_link"):
        McpServerConfig.model_validate({"cameras": {"front": {"mount": MOUNT}}})
    ok = McpServerConfig.model_validate({"cameras": {"front": {"mount": {**MOUNT, "parent_frame": "base_link"}}}})
    assert ok.cameras.front.mount is not None


def test_unknown_camera_keys_rejected() -> None:
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"cameras": {"rear": {}}})


def test_arm_base_offset_and_reach_defaults() -> None:
    arm = McpServerConfig().arm
    assert arm.base_in_base_link is None
    assert 0.0 < arm.reach_inner_m < arm.reach_outer_m
    cfg = McpServerConfig.model_validate({"arm": {"base_in_base_link": {"x": 0.1, "z": 0.165, "yaw": 3.14}}})
    assert cfg.arm.base_in_base_link is not None and cfg.arm.base_in_base_link.x == 0.1
    with pytest.raises(ValidationError):
        McpServerConfig.model_validate({"arm": {"reach_inner_m": 0.5, "reach_outer_m": 0.3}})
