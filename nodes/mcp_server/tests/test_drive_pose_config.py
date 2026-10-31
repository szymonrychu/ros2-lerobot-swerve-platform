"""The configured drive pose (client.yml mcp_server arm.drive_pose.joints) passes every drive pose check from the
parked home (scripts/gen_nav_footprint.yaml parked_arm_joints)."""

import yaml

from mcp_server.config import REPO_ROOT
from mcp_server.drive_pose_check import check_drive_pose, deployed_config

CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
FOOTPRINT_VARS = REPO_ROOT / "scripts" / "gen_nav_footprint.yaml"


def test_configured_drive_pose_passes_every_check_from_the_parked_home() -> None:
    config = deployed_config(CLIENT_VARS)
    home = yaml.safe_load(FOOTPRINT_VARS.read_text())["parked_arm_joints"]
    pose = config.arm.drive_pose.joints
    assert pose is not None
    assert check_drive_pose(config, home, pose) == []
