"""Camera settings in the mcp_server config: intrinsics sources, mount parent frames, arm base offset and reach."""

import logging
from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from mcp_server.camera_scene import CameraNotCalibratedError, camera_setup, intrinsics_cache, warned
from mcp_server.config import REPO_ROOT, IntrinsicsSettings, McpServerConfig
from mcp_server.head_kinematics import HeadKinematics

MOUNT = {"parent_frame": "gripper_link", "x": 0.0, "y": 0.0, "z": 0.05, "roll": 0.0, "pitch": 0.1, "yaw": 0.0}


def test_default_gripper_not_calibrated_front_is_the_d435_colour() -> None:
    cams = McpServerConfig().cameras
    assert cams.gripper.intrinsics is None and cams.gripper.mount is None
    assert cams.gripper.parent_frame == "gripper_link"
    # The front camera is the head D435 colour: factory CameraInfo intrinsics, zero mount on d435_color_frame.
    assert cams.front.parent_frame == "d435_color_frame"
    assert cams.front.intrinsics == IntrinsicsSettings(camera_info_topic="/head/d435/color/camera_info")
    assert cams.front.mount is not None and cams.front.mount.parent_frame == "d435_color_frame"
    assert [getattr(cams.front.mount, k) for k in ("x", "y", "z", "roll", "pitch", "yaw")] == [0.0] * 6
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


def test_camera_info_topic_is_an_exclusive_source() -> None:
    topic = "/head/d435/color/camera_info"

    def intrinsics(**fields: object) -> McpServerConfig:
        return McpServerConfig.model_validate({"cameras": {"front": {"intrinsics": fields}}})

    alone = intrinsics(camera_info_topic=topic).cameras.front.intrinsics
    assert alone is not None and alone.camera_info_topic == topic
    assert alone.calibration_file is None and alone.hfov_deg is None
    scaled = intrinsics(camera_info_topic=topic, image_width=1280, image_height=720).cameras.front.intrinsics
    assert scaled is not None and (scaled.image_width, scaled.image_height) == (1280, 720)
    for other in (
        {"calibration_file": "/x.yaml"},
        {"hfov_deg": 60.0, "width": 640, "height": 360},
        {"hfov_deg": 60.0},
        {"width": 640, "height": 360},
    ):
        with pytest.raises(ValidationError, match="camera_info_topic"):
            intrinsics(camera_info_topic=topic, **other)
    with pytest.raises(ValidationError):
        intrinsics(camera_info_topic="")


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
STREAM_SIZE = (1152, 648)  # a stream of the calibration's aspect at twice its size
D435_TOPICS = REPO_ROOT / "nodes" / "bridges" / "realsense_d435" / "config" / "topics.yaml"
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
                    "parent_frame": "d435_color_frame",
                    "intrinsics": {"calibration_file": str(path), "image_width": size[0], "image_height": size[1]},
                    "mount": {
                        "parent_frame": "d435_color_frame",
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
    assert front.parent_frame == "d435_color_frame" and front.mount is not None
    assert HeadKinematics(cfg.head.urdf_path).link_pose(0.0, 0.0, front.parent_frame) is not None
    assert front.intrinsics is not None and front.intrinsics.camera_info_topic is not None


def test_client_yml_front_intrinsics_are_the_live_d435_colour_camera_info() -> None:
    """The front intrinsics come from the live D435 colour CameraInfo at its own size (no rescale to another stream)."""
    intr = McpServerConfig.model_validate(client_node_config("mcp_server")).cameras.front.intrinsics
    live = yaml.safe_load(D435_TOPICS.read_text())["topics"]
    assert intr is not None and intr.camera_info_topic == "/head/d435/color/camera_info"
    assert live[intr.camera_info_topic]["type"] == "sensor_msgs/msg/CameraInfo"
    assert (intr.image_width, intr.image_height) == (None, None)


def test_front_intrinsics_scaled_to_stream_size(tmp_path: Path) -> None:
    path = write_calib(tmp_path / "c.yaml", CALIB_576)
    intr = camera_setup(front_cfg(path, STREAM_SIZE), "front").intrinsics
    assert (intr.width, intr.height) == STREAM_SIZE
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
    cfg = front_cfg(tmp_path / "absent.yaml", STREAM_SIZE)
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
    missing = front_cfg(tmp_path / "absent.yaml", STREAM_SIZE)
    with pytest.raises(CameraNotCalibratedError):
        camera_setup(missing, "front")
    path = write_calib(tmp_path / "c.yaml", CALIB_576)
    with pytest.raises(CameraNotCalibratedError):
        camera_setup(front_cfg(path, (640, 480)), "front")
