"""The packaged robot body holds the head envelope: its head cylinder contains arm_pose_check.head_envelope."""

from pathlib import Path

import yaml
from arm_collision.settings import load_robot_body
from ros2_common.head_rgbd_calibration.arm_pose_check import PoseCheckConfig, head_envelope

from mcp_server.config import REPO_ROOT, McpServerConfig

CLIENT_YML = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
CONTAIN_TOLERANCE_M = 1e-9


def deployed_pose_check() -> PoseCheckConfig:
    """The head_rgbd_calibration pose check of client.yml (deployed head_radius_m / head_height_m).

    Returns:
        PoseCheckConfig: The deployed pose check settings.
    """
    nodes = yaml.safe_load(CLIENT_YML.read_text())["ros2_nodes"]
    node = next(n for n in nodes if n["name"] == "head_rgbd_calibration")
    return PoseCheckConfig.model_validate(yaml.safe_load(node["config"])["arm_sequence"]["pose_check"])


def test_body_head_cylinder_contains_the_head_envelope() -> None:
    """The body's head cylinder contains the envelope of the head URDF at every pan/tilt (deployed head sizes)."""
    envelope = head_envelope(Path(McpServerConfig().head.urdf_path).read_text(), deployed_pose_check())
    head = next(c for c in load_robot_body().cylinders if c.name == "head")
    offset = ((head.x - envelope.x) ** 2 + (head.y - envelope.y) ** 2) ** 0.5
    assert offset + envelope.radius <= head.radius + CONTAIN_TOLERANCE_M
    assert head.z_min <= envelope.z_min + CONTAIN_TOLERANCE_M
    assert head.z_max >= envelope.z_max - CONTAIN_TOLERANCE_M
