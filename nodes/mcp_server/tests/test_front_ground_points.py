"""The head (D435 colour) camera gives floor points through the applied intrinsics and the head chain (design D8)."""

import asyncio
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import yaml
from mcp.server.mcpserver.exceptions import ToolError
from ros2_common.camera_geometry import CameraIntrinsics, optical_from_mount, project_raw

from mcp_server.camera_scene import default_picture_size, intrinsics_cache, warned
from mcp_server.config import McpServerConfig
from mcp_server.head_kinematics import HeadKinematics

from .test_camera_tools import make, text_json
from .test_tools import call

STREAM = (1152, 648)  # head stream size under test (16:9)
CALIBRATED = (576, 324)  # calibration image size, half the stream
DEFAULT_PICTURE = (384, 216)  # get_camera_image default: 384 px on the longest side, 16:9
PAN_DEG, TILT_DEG = 20.0, 30.0
# head.urdf frame of the D435 colour camera (design D4); its optical frame is CAMERA_FRAME.
FRONT_FRAME = "d435_color_frame"
FLOOR_POINT = (1.0, 0.3, 0.0)  # base_link, on the floor
CALIB = {
    "image_width": CALIBRATED[0],
    "image_height": CALIBRATED[1],
    "camera_matrix": {"rows": 3, "cols": 3, "data": [400.0, 0.0, 290.0, 0.0, 410.0, 160.0, 0.0, 0.0, 1.0]},
    "distortion_coefficients": {"rows": 1, "cols": 5, "data": [0.1, -0.2, 0.0, 0.0, 0.05]},
}
ZERO_MOUNT = {"parent_frame": FRONT_FRAME, "x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0}


@pytest.fixture(autouse=True)
def fresh_caches() -> None:
    intrinsics_cache.clear()
    warned.clear()


def front_config(calib: Path) -> McpServerConfig:
    """Config whose head camera hangs on the D435 colour frame with an applied-style calibration file."""
    return McpServerConfig.model_validate(
        {
            "cameras": {
                "head": {
                    "parent_frame": FRONT_FRAME,
                    "intrinsics": {"calibration_file": str(calib), "image_width": STREAM[0], "image_height": STREAM[1]},
                    "mount": ZERO_MOUNT,
                }
            }
        }
    )


@pytest.fixture
def front(tmp_path: Path) -> tuple[Any, Any, McpServerConfig]:
    calib = tmp_path / "front_calibration.yaml"
    calib.write_text(yaml.safe_dump(CALIB))
    server, robot, cfg = make(tmp_path, front_config(calib))
    robot.head_backend.positions.update({"head_pan": math.radians(PAN_DEG), "head_tilt": math.radians(TILT_DEG)})
    return server, robot, cfg


def stream_pixel(cfg: McpServerConfig) -> tuple[float, float]:
    """Pixel (stream size) of FLOOR_POINT through the intrinsics and the head chain at PAN_DEG / TILT_DEG."""
    k = CALIB["camera_matrix"]["data"]
    scale = STREAM[0] / CALIBRATED[0]
    stream = CameraIntrinsics(
        width=STREAM[0],
        height=STREAM[1],
        fx=k[0] * scale,
        fy=k[4] * scale,
        cx=k[2] * scale,
        cy=k[5] * scale,
        distortion=CALIB["distortion_coefficients"]["data"],
    )
    link = HeadKinematics(cfg.head.urdf_path).link_pose(math.radians(PAN_DEG), math.radians(TILT_DEG), FRONT_FRAME)
    u, v, _ = project_raw(stream, optical_from_mount(link), np.array(FLOOR_POINT))
    assert 0 <= u < STREAM[0] and 0 <= v < STREAM[1]
    return float(u), float(v)


def test_front_pixel_to_ground_uses_applied_intrinsics(front: tuple[Any, Any, McpServerConfig]) -> None:
    server, _, cfg = front
    u, v = stream_pixel(cfg)
    data = call(server, "pixel_to_ground", {"camera": "head", "u": u, "v": v}).structured_content
    assert data["ground_base_link"]["x"] == pytest.approx(FLOOR_POINT[0], abs=0.01)
    assert data["ground_base_link"]["y"] == pytest.approx(FLOOR_POINT[1], abs=0.01)
    assert data["ground_base_link"]["z"] == 0.0
    assert data["image"] == {"width": STREAM[0], "height": STREAM[1]}
    assert "approximate" not in data["uncertainty_note"]


def test_front_default_picture_size_accepted(front: tuple[Any, Any, McpServerConfig]) -> None:
    server, _, cfg = front
    assert default_picture_size(*STREAM, cfg.limits.default_camera_px) == DEFAULT_PICTURE
    u, v = stream_pixel(cfg)
    full = call(server, "pixel_to_ground", {"camera": "head", "u": u, "v": v}).structured_content
    scale = STREAM[0] / DEFAULT_PICTURE[0]
    small = call(
        server,
        "pixel_to_ground",
        {
            "camera": "head",
            "u": u / scale,
            "v": v / scale,
            "image_width": DEFAULT_PICTURE[0],
            "image_height": DEFAULT_PICTURE[1],
        },
    ).structured_content
    for axis in ("x", "y", "z"):
        assert small["ground_base_link"][axis] == pytest.approx(full["ground_base_link"][axis], abs=1e-3)
    assert small["pixel_input"]["image_width"] == DEFAULT_PICTURE[0]


def test_front_wrong_aspect_names_default_size(front: tuple[Any, Any, McpServerConfig]) -> None:
    server, _, _ = front
    with pytest.raises(ToolError, match="384x216"):
        call(
            server,
            "pixel_to_ground",
            {"camera": "head", "u": 100.0, "v": 100.0, "image_width": 384, "image_height": 288},
        )


def test_tool_descriptions_name_each_cameras_default_picture(front: tuple[Any, Any, McpServerConfig]) -> None:
    server, _, _ = front
    tools = {t.name: t for t in call_list(server)}
    assert "384x216" in tools["pixel_to_ground"].description
    assert "384x288" in tools["pixel_to_ground"].description or "gripper" not in tools["pixel_to_ground"].description
    assert "384x216" in tools["mark_candidate_points"].description


def call_list(server: Any) -> list[Any]:
    return asyncio.run(server.list_tools())


def test_mark_candidate_points_front(front: tuple[Any, Any, McpServerConfig]) -> None:
    server, _, cfg = front
    res = call(server, "mark_candidate_points", {"camera": "head", "spacing_px": 80, "max_points": 200})
    table = text_json(res)
    assert table["approximate_intrinsics"] is False
    assert table["image"] == {"width": STREAM[0], "height": STREAM[1]}
    assert table["points"], "the tilted head sees the floor"
    for point in table["points"][:5]:
        direct = call(server, "pixel_to_ground", {"camera": "head", "u": point["u"], "v": point["v"]})
        for axis in ("x", "y"):
            assert point["ground_base_link"][axis] == pytest.approx(
                direct.structured_content["ground_base_link"][axis], abs=2e-3
            )


def test_uncalibrated_front_stays_not_calibrated(tmp_path: Path) -> None:
    server, _, _ = make(tmp_path, front_config(tmp_path / "absent.yaml"))
    for tool, args in (
        ("pixel_to_ground", {"u": 100.0, "v": 100.0}),
        ("mark_candidate_points", {}),
    ):
        with pytest.raises(ToolError, match="head not calibrated"):
            call(server, tool, {"camera": "head", **args})
