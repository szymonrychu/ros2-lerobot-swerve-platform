"""Retirement pin for the Jetson coprocessor: removed paths stay gone and removed tokens stay absent.

Extend by adding entries to REMOVED_PATHS and ABSENT_TOKENS.
"""

from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
CLIENT_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
BRIDGE = "jetson_bridge_cpp"
BRIDGE_TYPE_KEYS = {"deploy_mode", "node_launch_command", "colcon_purge"}

REMOVED_PATHS: list[str] = [
    "jetson",
    "scripts/jetson_e2e_check.py",
    "scripts/jetson_e2e_check.yaml",
    "tests/test_jetson_e2e_check.py",
    "ansible/roles/jetson_base",
    "ansible/roles/jetson_depthd",
    "tests/test_jetson_base_role.py",
    "tests/test_jetson_depthd_role.py",
    "tests/test_jetson_bridge_config.py",
    "nodes/jetson_bridge_cpp",
    "shared/ros2_common_cpp/include/ros2_common_cpp/depth_wire.hpp",
    "shared/ros2_common_cpp/src/depth_wire.cpp",
    "shared/ros2_common_cpp/test/test_depth_wire.cpp",
]

ABSENT_TOKENS: list[tuple[str, str]] = [
    ("scripts/README.md", "jetson_e2e_check"),
    ("ansible/playbooks/optimize.yml", "jetson"),
    ("ansible/site.yml", "jetson"),
    ("ansible/README.md", "jetson_base"),
    ("ansible/README.md", "jetson_depthd"),
    ("ansible/README.md", "deploy-nodes.sh jetson"),
    ("ansible/README.md", "group_vars/jetson.yml"),
    (".github/workflows/ci.yml", "jetson_bridge_cpp"),
    ("nodes/README.md", "jetson_bridge_cpp"),
    ("shared/ros2_common_cpp/CMakeLists.txt", "zstd"),
    ("shared/ros2_common_cpp/CMakeLists.txt", "depth_wire"),
    ("shared/ros2_common_cpp/package.xml", "zstd"),
    ("ansible/group_vars/client.yml", "libzstd-dev"),
    ("shared/README.md", "depth_wire"),
    ("shared/README.md", "depthd"),
]


@pytest.mark.parametrize("path", REMOVED_PATHS)
def test_removed_path_is_gone(path: str) -> None:
    """Assert a retired repo-relative path does not exist.

    Args:
        path: Repo-relative path that must not exist.
    """
    assert not (REPO_ROOT / path).exists(), f"{path} must stay deleted"


@pytest.mark.parametrize(("path", "token"), ABSENT_TOKENS)
def test_removed_token_is_absent(path: str, token: str) -> None:
    """Assert a retired token does not appear in a file (case-sensitive).

    Args:
        path: Repo-relative file to scan.
        token: Text that must not occur in it.
    """
    assert token not in (REPO_ROOT / path).read_text(), f"{path} still mentions {token!r}"


def load_client() -> dict:
    """Load ansible/group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def test_jetson_bridge_entry_is_uninstalled_by_the_next_deploy() -> None:
    """The ros2_nodes entry is present: false with no enabled and no config, and a comment naming the Jetson removal."""
    node = next(n for n in load_client()["ros2_nodes"] if n["name"] == BRIDGE)
    assert node["node_type"] == BRIDGE
    assert node["present"] is False
    assert "enabled" not in node and "config" not in node
    lines = CLIENT_VARS.read_text().splitlines()
    index = lines.index(f"  - name: {BRIDGE}")
    comment = []
    while lines[index - 1].lstrip().startswith("#"):
        index -= 1
        comment.append(lines[index])
    assert "Jetson" in " ".join(comment)


def test_jetson_bridge_type_keeps_only_the_uninstall_and_purge_keys() -> None:
    """The node type has the launch command the role asserts and the colcon purge, nothing to build, run or tier."""
    defaults = load_client()["ros2_node_type_defaults"][BRIDGE]
    assert set(defaults) == BRIDGE_TYPE_KEYS
    assert defaults["deploy_mode"] == "native"
    assert defaults["node_launch_command"] == "ros2 run jetson_bridge_cpp jetson_bridge_node"
    assert defaults["colcon_purge"] == [{"package": BRIDGE, "workspace": "/opt/ros2-ws"}]


def test_client_playbook_still_deploys_jetson_bridge_so_the_uninstall_runs() -> None:
    """The playbook block stays: without it the present: false entry would never be applied."""
    assert f"_deploy_node_name: {BRIDGE}" in CLIENT_PLAYBOOK.read_text()


def test_jetson_bridge_metrics_target_is_removed() -> None:
    """No Alloy scrape of the retired bridge (127.0.0.1:19114)."""
    targets = load_client()["monitoring_agent_node_targets"]
    assert not [t for t in targets if t["node"] == BRIDGE or t["address"].endswith(":19114")]
