"""filter_node's self-collision guard checks the same arm as mcp_server (ansible/group_vars/client.yml).

The body is one packaged file (shared/arm_collision robot_body.yaml); the arm calibration stays in each node config.
This pins filter_node's `self_collision_guard.arm` to mcp_server's `arm` values of every ArmModelSettings field,
web_ui's `arm_calibration` (the 3D arm views' baseline) to mcp_server's calibrated arm values (ARM_SETTINGS_FIELDS), the
keep-away to mcp_server's body_check.hard_margin_m, filter_node's applied_dir and web_ui's arm_calibration.applied_dir
to mcp_server's arm.applied_dir, and the guard on. The node type ships the arm URDF and the C++ kernels the shared
package path-depends on.
"""

from pathlib import Path
from typing import Any

import yaml
from arm_collision.settings import ArmModelSettings
from ros2_common.head_rgbd_calibration.applied_arm_calibration import ArmCalibrationSettings
from ros2_common.head_rgbd_calibration.arm_calibration_file import ARM_SETTINGS_FIELDS

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"


def client_vars() -> dict[str, Any]:
    """client.yml group vars.

    Returns:
        dict[str, Any]: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def client_node(name: str) -> dict[str, Any]:
    """Parsed config of a client node.

    Args:
        name (str): ros2_nodes name.

    Returns:
        dict[str, Any]: The node's config YAML.
    """
    nodes = client_vars()["ros2_nodes"]
    return yaml.safe_load(next(n for n in nodes if n["name"] == name)["config"])


def test_filter_node_guard_arm_equals_mcp_server_arm() -> None:
    """Every ArmModelSettings field of the guard's arm equals mcp_server's (explicit or default alike)."""
    mcp_arm = client_node("mcp_server")["arm"]
    guard_arm = client_node("filter_node")["self_collision_guard"]["arm"]
    shared = {k: v for k, v in mcp_arm.items() if k in ArmModelSettings.model_fields}
    assert ArmModelSettings.model_validate(guard_arm) == ArmModelSettings.model_validate(shared)
    assert set(guard_arm) == set(shared)


def test_the_accepted_arm_calibration_consumers_read_mcp_servers_applied_dir() -> None:
    """filter_node's guard and the web_ui 3D arm view follow the same accepted arm calibration as mcp_server."""
    applied = client_node("mcp_server")["arm"]["applied_dir"]
    assert applied == "/var/lib/ros2/head_rgbd_calibration/applied"
    assert client_node("filter_node")["self_collision_guard"]["applied_dir"] == applied
    assert client_node("web_ui")["arm_calibration"]["applied_dir"] == applied


def test_web_ui_arm_baseline_equals_mcp_server_configured_arm_values() -> None:
    """With nothing accepted the 3D arm views draw with mcp_server's (and the guard's) configured joint offsets, mount
    height and arm base pose; web_ui's arm_calibration holds exactly those keys and its applied_dir."""
    mcp_arm = client_node("mcp_server")["arm"]
    web_arm = client_node("web_ui")["arm_calibration"]
    assert set(web_arm) == {*ARM_SETTINGS_FIELDS, "applied_dir"}
    configured = {k: mcp_arm[k] for k in ARM_SETTINGS_FIELDS}
    baseline = {k: web_arm[k] for k in ARM_SETTINGS_FIELDS}
    assert baseline == configured
    assert ArmCalibrationSettings.model_validate(baseline) == ArmCalibrationSettings.model_validate(configured)
    model = ArmModelSettings.model_validate({k: v for k, v in mcp_arm.items() if k in ArmModelSettings.model_fields})
    assert ArmCalibrationSettings.model_validate(baseline).model_dump() == {
        "joint_offsets_rad": model.joint_offsets_rad.model_dump(),
        "arm_base_height_m": model.arm_base_height_m,
        "base_in_base_link": None if model.base_in_base_link is None else model.base_in_base_link.model_dump(),
    }


def test_filter_node_guard_keep_away_equals_mcp_server_hard_margin() -> None:
    """The guard keeps the arm as far from the body as mcp_server's planner does."""
    guard = client_node("filter_node")["self_collision_guard"]
    assert guard["hard_margin_m"] == client_node("mcp_server")["body_check"]["hard_margin_m"]


def test_filter_node_guard_is_enabled() -> None:
    """The guard is configured and on."""
    assert client_node("filter_node")["self_collision_guard"]["enabled"] is True


def test_filter_node_node_type_ships_the_urdf_and_the_kernels() -> None:
    """filter_node's venv builds mcp_core_cpp (arm_collision's path dependency) and loads the arm URDF of web_ui."""
    node_type = client_vars()["ros2_node_type_defaults"]["filter_node"]
    assert node_type["src_shared"] is True
    assert set(node_type["src_extra_paths"]) == {"nodes/mcp_server/mcp_core_cpp", "nodes/web_ui/urdf"}


GRIPPER_RANGE_KEYS = ("source_min_steps", "source_max_steps", "command_min_steps", "command_max_steps")


def test_follower_gripper_command_range_is_configured_in_full() -> None:
    """The follower bridge uses a configured command range only with both ends: 1900..3377 (the servo's own angle
    limits, read by its startup register dump), so the guard can read the same values."""
    joints = client_node("lerobot_follower")["joint_names"]
    gripper = next(j for j in joints if j["name"] == "gripper")
    assert (gripper["command_min_steps"], gripper["command_max_steps"]) == (1900, 3377)


def test_filter_node_leader_gripper_equals_the_follower_bridge_mapping() -> None:
    """The guard converts a leader gripper command with exactly the follower bridge's range mapping."""
    joints = client_node("lerobot_follower")["joint_names"]
    bridge = next(j for j in joints if j["name"] == "gripper")
    guard = client_node("filter_node")["self_collision_guard"]["leader_gripper"]
    assert guard.get("joint", "gripper") == bridge["name"]
    assert {k: guard[k] for k in GRIPPER_RANGE_KEYS} == {k: bridge[k] for k in GRIPPER_RANGE_KEYS}
    assert guard.get("source_inverted", False) == bridge.get("source_inverted", False)
    assert guard.get("inverted", False) == bridge.get("inverted", False)
    direct = client_node("lerobot_follower")["direct_command_sources"]
    assert "leader" not in direct and {"web_ui", "autonomy"} <= set(direct)
