"""Static invariants of the Pi camera retirement (D7): no cameras on the Pi.

The overview_camera ros2_nodes entry is present: false (the next deploy uninstalls its unit) and its node type keeps
only what the uninstall path needs. The boot config that replaced its overlay file is covered by
test_pi_camera_boot_config.py.
"""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE = REPO_ROOT / "ansible"
CLIENT_VARS = ANSIBLE / "group_vars" / "client.yml"
CLIENT_PLAYBOOK = ANSIBLE / "playbooks" / "deploy_nodes_client.yml"
TASKS = ANSIBLE / "playbooks" / "tasks"
REMOVED_TASKS = TASKS / "pi_camera_overlays_removed.yml"
OLD_TASKS = TASKS / "overview_camera_boot_config.yml"
ANSIBLE_README = ANSIBLE / "README.md"
TESTS_README = REPO_ROOT / "tests" / "README.md"
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
    "nice",
)


def load_client() -> dict:
    """Load ansible/group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def test_overview_camera_entry_is_uninstalled_by_the_next_deploy() -> None:
    node = next(n for n in load_client()["ros2_nodes"] if n["name"] == "overview_camera")
    assert node["node_type"] == "overview_camera"
    assert node["present"] is False
    assert "config" not in node


def test_overview_camera_entry_names_d7_in_its_comment() -> None:
    lines = CLIENT_VARS.read_text().splitlines()
    index = lines.index("  - name: overview_camera")
    comment = []
    while lines[index - 1].lstrip().startswith("#"):
        index -= 1
        comment.append(lines[index])
    assert "D7" in " ".join(comment)


def test_overview_camera_node_type_keeps_only_what_the_uninstall_path_needs() -> None:
    defaults = load_client()["ros2_node_type_defaults"]["overview_camera"]
    assert defaults["deploy_mode"] == "native"
    assert defaults["node_launch_command"]
    for key in DEPLOY_ONLY_KEYS:
        assert key not in defaults, key
    assert set(defaults) == {"deploy_mode", "node_launch_command"}


def test_only_pi_cameras_builds_libcamera_and_camera_ros() -> None:
    defaults = load_client()["ros2_node_type_defaults"]
    builders = [name for name, node in defaults.items() if "raspberrypi/libcamera" in str(node.get("colcon_source"))]
    assert builders == ["pi_cameras"]


def test_old_boot_file_and_old_tests_are_deleted() -> None:
    assert not OLD_TASKS.exists()
    assert not REMOVED_TASKS.exists()
    assert not (REPO_ROOT / "tests" / "test_overview_camera_config.py").exists()
    assert not (REPO_ROOT / "tests" / "test_overview_camera_launch_config.py").exists()


def test_client_playbook_still_deploys_overview_camera_so_the_uninstall_runs() -> None:
    assert "_deploy_node_name: overview_camera" in CLIENT_PLAYBOOK.read_text()


def test_ansible_readme_describes_the_removal() -> None:
    readme = ANSIBLE_README.read_text()
    assert "overview_camera_boot_config" not in readme
    assert "pi_camera_overlays_removed" not in readme
    assert "| overview_camera |" not in readme, "no memory table row for a retired node"


def test_tests_readme_lists_every_test_of_this_file() -> None:
    readme = TESTS_README.read_text()
    assert "### `test_pi_cameras_retired.py`" in readme
    for name in [n for n in globals() if n.startswith("test_")]:
        assert f"`{name}`" in readme, name
