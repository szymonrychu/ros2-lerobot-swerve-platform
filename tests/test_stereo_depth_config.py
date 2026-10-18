"""Static invariants of the stereo_depth / stereo_depth_cpp retirement (D7).

The Pi's stereo pipeline never produced depth; it is retired (D7) and its Python sources are deleted. Both
ros2_nodes entries are present: false, so the next deploy of those names uninstalls their units. Their node types keep
only what the uninstall path of roles/ros2_node_deploy/tasks/main.yml needs: no source, build or tier settings, and the
19113 monitoring target is gone.
"""

from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
GROUP_VARS = REPO_ROOT / "ansible" / "group_vars"
CLIENT_PLAYBOOK = REPO_ROOT / "ansible" / "playbooks" / "deploy_nodes_client.yml"
ANSIBLE_README = REPO_ROOT / "ansible" / "README.md"
TESTS_README = REPO_ROOT / "tests" / "README.md"
NODES = ("stereo_depth", "stereo_depth_cpp")
# The stereo nodes' old metrics port; map_manager reuses it, so only the stereo names are checked against it.
RETIRED_PORT = 19113
# Everything else a native type would carry to build, run or tier a node: none of it applies to an uninstall.
DEPLOY_ONLY_KEYS = (
    "node_src_dir",
    "src_shared",
    "src_paths",
    "colcon_source",
    "apt_packages",
    "env",
    "config_path",
    "memory_low",
    "memory_max",
    "cpu_weight",
    "io_weight",
    "oom_score_adjust",
)


def load_client() -> dict:
    """Load ansible/group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load((GROUP_VARS / "client.yml").read_text())


def client_node(name: str) -> dict:
    """Return one client ros2_nodes entry.

    Args:
        name (str): Node entry name.

    Returns:
        dict: The entry.
    """
    return next(n for n in load_client()["ros2_nodes"] if n["name"] == name)


def test_ros2_nodes_entries_are_uninstalled_by_the_next_deploy() -> None:
    for name in NODES:
        node = client_node(name)
        assert node["node_type"] == name
        assert node["present"] is False, name
        assert "config" not in node, "the retired entry carries no node config"


def test_the_entries_name_d7_in_the_comment_above_them() -> None:
    lines = (GROUP_VARS / "client.yml").read_text().splitlines()
    for name in NODES:
        index = lines.index(f"  - name: {name}")
        comment = []
        while lines[index - 1].lstrip().startswith("#"):
            index -= 1
            comment.append(lines[index])
        assert "D7" in " ".join(comment), name


def test_node_types_keep_only_what_the_uninstall_path_needs() -> None:
    types = load_client()["ros2_node_type_defaults"]
    for name in NODES:
        defaults = types[name]
        assert defaults["deploy_mode"] == "native"
        assert defaults["node_launch_command"], "the native-mode assert in the role needs a launch command"
        for key in DEPLOY_ONLY_KEYS:
            assert key not in defaults, (name, key)
        assert set(defaults) == {"deploy_mode", "node_launch_command"}, name


def test_background_tier_has_no_stereo_type() -> None:
    types = load_client()["ros2_node_type_defaults"]
    background = [n for n, t in types.items() if (t.get("cpu_weight"), t.get("io_weight")) == (100, 100)]
    assert sorted(background) == ["joint_state_throttle_follower", "joint_state_throttle_swerve", "pi_cameras"]


def test_metrics_target_is_removed() -> None:
    targets = load_client()["monitoring_agent_node_targets"]
    assert not [t for t in targets if t["node"] in NODES]
    assert not [t for t in targets if t["address"].endswith(f":{RETIRED_PORT}") and t["node"] in NODES]


def test_client_playbook_still_deploys_both_names_so_the_uninstall_runs() -> None:
    text = CLIENT_PLAYBOOK.read_text()
    for name in NODES:
        assert f"_deploy_node_name: {name}" in text, name


def test_ansible_readme_documents_the_retirement() -> None:
    readme = ANSIBLE_README.read_text()
    section = readme[readme.index("Cut-over and rollback") :]
    assert "D7" in section and "present: false" in section
    assert not [row for row in readme.splitlines() if "stereo_depth" in row and f"| {RETIRED_PORT} |" in row]
    assert "| stereo_depth" not in readme, "no tier or memory table row for a retired node"


def test_tests_readme_metrics_wiring_rows_have_no_stereo_node() -> None:
    rows = [
        line
        for line in TESTS_README.read_text().splitlines()
        if line.startswith("| `test_config_metrics_port`")
        or line.startswith("| `test_node_types_using_ros2_metrics_declare_src_shared`")
    ]
    assert len(rows) == 2
    for row in rows:
        assert "stereo_depth" not in row and str(RETIRED_PORT) not in row, row


def test_tests_readme_lists_every_test_of_this_file() -> None:
    readme = TESTS_README.read_text()
    assert "### test_stereo_depth_config.py" in readme
    for name in [n for n in globals() if n.startswith("test_")]:
        assert f"`{name}`" in readme, name


@pytest.mark.parametrize("name", ["stereo_depth", "stereo_depth_cpp"])
def test_stereo_sources_are_deleted(name: str) -> None:
    """The retired stereo node's sources are gone (D7); only the uninstall entry stays.

    :param name: node directory name under nodes/.
    """
    assert not (REPO_ROOT / "nodes" / name).exists()
