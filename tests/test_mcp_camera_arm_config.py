"""Wiring of the mcp_server camera-to-arm tools (pixel_to_point, camera_point_to_arm): topics, defaults, gate, docs."""

import sys
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
D435_TOPICS = REPO_ROOT / "nodes" / "bridges" / "realsense_d435" / "config" / "topics.yaml"
D435_CONFIG = REPO_ROOT / "nodes" / "bridges" / "realsense_d435" / "config" / "realsense_d435.yaml"
MCP_README = REPO_ROOT / "nodes" / "mcp_server" / "README.md"
DEPTH_TOPIC = "/head/d435/aligned_depth_to_color/image_raw"
DEPTH_INFO_TOPIC = "/head/d435/aligned_depth_to_color/camera_info"
COLOUR_OPTICAL_FRAME = "d435_color_optical_frame"
USER_GATE_MM = 5.0  # user decision 2026-10-29: direct grasps are gated on 5 mm unless configured otherwise

sys.path[:0] = [str(REPO_ROOT / "nodes" / "mcp_server"), str(REPO_ROOT / "shared")]
try:
    from mcp_server.config import CameraArmSettings, HeadSettings, TopicSettings
    from ros2_common.head_rgbd_calibration.config import HeadArmFitLimits
finally:
    del sys.path[:2]


def mcp_config() -> dict[str, Any]:
    """Parse the mcp_server `config: |` block of group_vars/client.yml.

    Returns:
        dict[str, Any]: The node config.
    """
    entry = next(n for n in yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"] if n["name"] == "mcp_server")
    return yaml.safe_load(entry["config"])


def test_depth_topics_are_the_registered_depth_the_d435_publishes() -> None:
    topics = mcp_config()["topics"]
    live = yaml.safe_load(D435_TOPICS.read_text())["topics"]
    assert topics["head_depth"] == DEPTH_TOPIC == TopicSettings().head_depth
    assert topics["head_depth_info"] == DEPTH_INFO_TOPIC == TopicSettings().head_depth_info
    assert live[DEPTH_TOPIC]["type"] == "sensor_msgs/msg/Image"
    assert live[DEPTH_INFO_TOPIC]["type"] == "sensor_msgs/msg/CameraInfo"
    # Registered to the colour image: both are stamped in the colour optical frame, which is the frame the tools use.
    assert live[DEPTH_TOPIC]["frame_id"] == live[DEPTH_INFO_TOPIC]["frame_id"] == COLOUR_OPTICAL_FRAME


def test_the_driver_aligns_depth_to_colour() -> None:
    assert yaml.safe_load(D435_CONFIG.read_text())["align_depth.enable"] is True


def test_client_yml_camera_arm_block_equals_the_code_defaults() -> None:
    assert mcp_config()["camera_arm"] == CameraArmSettings().model_dump()


def test_client_yml_head_history_keys_equal_the_code_defaults() -> None:
    head, defaults = mcp_config()["head"], HeadSettings()
    for key in ("history_s", "stamp_window_s", "max_motion_rad", "max_sample_gap_s"):
        assert head[key] == getattr(defaults, key), key


def test_the_grasp_gate_is_5_mm_by_default_and_in_client_yml() -> None:
    assert CameraArmSettings().max_residual_mm == USER_GATE_MM
    assert mcp_config()["camera_arm"]["max_residual_mm"] == USER_GATE_MM


def test_a_head_arm_fit_that_passes_is_inside_the_grasp_gate() -> None:
    """Apply only accepts a passed fit, whose held-out board error is within fit.limits.holdout_board_mm: so every
    applied head-from-arm chain satisfies the camera_arm gate and the page's Apply cannot produce an ungraspable one."""
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    node = yaml.safe_load(next(n for n in nodes if n["name"] == "head_rgbd_calibration")["config"])
    limit = node["fit"]["limits"]["holdout_board_mm"]
    assert limit <= mcp_config()["camera_arm"]["max_residual_mm"] <= USER_GATE_MM
    assert HeadArmFitLimits().holdout_board_mm <= USER_GATE_MM  # the code default too


def test_depth_error_model_is_the_d435_stereo_model() -> None:
    camera_arm = mcp_config()["camera_arm"]
    assert (camera_arm["baseline_m"], camera_arm["subpixel_px"]) == (0.05, 0.08)


def test_readme_documents_the_camera_to_arm_tools_the_frames_and_the_gate() -> None:
    text = MCP_README.read_text()
    for needle in (
        "## Camera to arm",
        "`pixel_to_point",
        "`camera_point_to_arm",
        "head_pose_at_capture",
        "position_source",
        "max_residual_mm",
        "d435_color_optical_frame",
        "d435_depth_optical_frame",
        "Calibrate page",
    ):
        assert needle in text, needle
