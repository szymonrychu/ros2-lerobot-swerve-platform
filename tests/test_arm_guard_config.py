"""filter_node's self-collision guard checks the same arm as mcp_server (ansible/group_vars/client.yml).

The body is one packaged file (shared/arm_collision robot_body.yaml); the arm calibration stays in each node config.
This pins filter_node's `self_collision_guard.arm` to mcp_server's `arm` values of every ArmModelSettings field, its
keep-away to mcp_server's body_check.hard_margin_m, and the guard on. The node type ships the arm URDF and the C++
kernels the shared package path-depends on.
"""

from pathlib import Path
from typing import Any

import yaml
from arm_collision.settings import ArmModelSettings

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
