# ruff: noqa: E501
"""Retirement pin for the Jetson coprocessor: removed paths stay gone and removed tokens stay absent.

Extend by adding entries to REMOVED_PATHS and ABSENT_TOKENS.
"""

import fnmatch
import re
import subprocess
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


RETIRED_PATTERN = r"jetson|\bdepthd\b|stereo|overview_camera|depth_wire|fused_depth|10\.77\.0\."
RETIRED_LINE = r"(?i)" + RETIRED_PATTERN
ARCHIVE_GLOBS = ("docs/plans/**", "docs/superpowers/**")
ALLOWED_CATEGORIES = {
    "archive",
    "retired-entry",
    "uninstall-artifact",
    "retirement-test",
    "retirement-doc",
    "pinned-build",
}

# (path glob, line regex, category). Each entry is one file plus a line regex as narrow as the hit; the only
# directory-wide globs are the two archive directories and the pinned libcamera patches.
ALLOWED_REFERENCES: tuple[tuple[str, str, str], ...] = (
    # archive: finished plans and specs record history and stay untouched.
    ("docs/plans/**", r".", "archive"),
    ("docs/superpowers/**", r".", "archive"),
    # retired-entry: the present: false entries and uninstall-only node types, so a deploy of the name uninstalls it.
    (
        "ansible/group_vars/client.yml",
        r"^  (overview_camera|stereo_depth|stereo_depth_cpp|jetson_bridge_cpp):$",
        "retired-entry",
    ),
    (
        "ansible/group_vars/client.yml",
        r'^    node_launch_command: "(ros2 launch overview_camera overview_camera\.launch\.py|python3 -m stereo_depth|ros2 run (stereo_depth_cpp stereo_depth_node|jetson_bridge_cpp jetson_bridge_node))"$',
        "retired-entry",
    ),
    ("ansible/group_vars/client.yml", r"^      - package: jetson_bridge_cpp$", "retired-entry"),
    (
        "ansible/group_vars/client.yml",
        r"^  # Retired \(D7\): the Pi's stereo pipeline never produced depth",
        "retired-entry",
    ),
    ("ansible/group_vars/client.yml", r"^  # Retired with the Jetson: present: false uninstalls", "retired-entry"),
    (
        "ansible/group_vars/client.yml",
        r"^  (- name|  node_type): (overview_camera|stereo_depth|stereo_depth_cpp|jetson_bridge_cpp)$",
        "retired-entry",
    ),
    ("ansible/group_vars/client.yml", r"^  # Retired \(D7\): the Pi never produced stereo depth", "retired-entry"),
    (
        "ansible/group_vars/client.yml",
        r"^  # Retired with the Jetson removal: present: false uninstalls",
        "retired-entry",
    ),
    (
        "ansible/playbooks/deploy_nodes_client.yml",
        r"^        - name: Deploy (overview_camera|stereo_depth|stereo_depth_cpp|jetson_bridge_cpp)$",
        "retired-entry",
    ),
    (
        "ansible/playbooks/deploy_nodes_client.yml",
        r"^ +tags: \[(overview_camera|stereo_depth|stereo_depth_cpp|jetson_bridge_cpp)(, apt, python, build, config)?\]$",
        "retired-entry",
    ),
    (
        "ansible/playbooks/deploy_nodes_client.yml",
        r"^            _deploy_node_name: (overview_camera|stereo_depth|stereo_depth_cpp|jetson_bridge_cpp)$",
        "retired-entry",
    ),
    # uninstall-artifact: the jetson_link removal block, its tag, task file and deploy target.
    ("ansible/playbooks/deploy_nodes_client.yml", r"jetson_link|Jetson", "uninstall-artifact"),
    ("ansible/playbooks/tasks/jetson_link_remove.yml", r"(?i)jetson", "uninstall-artifact"),
    ("ansible/playbooks/tasks/select_run.yml", r"jetson_link", "uninstall-artifact"),
    ("scripts/deploy-nodes.sh", r"jetson_link", "uninstall-artifact"),
    ("tests/test_ansible_deploy_speed.py", r'== \{"monitoring", "jetson_link"\}$', "uninstall-artifact"),
    # retirement-test: tests whose assertions check that a retired name is absent, or pin the uninstall.
    ("tests/test_jetson_retired.py", RETIRED_LINE, "retirement-test"),
    ("tests/test_stereo_depth_config.py", RETIRED_LINE, "retirement-test"),
    ("tests/test_pi_cameras_retired.py", RETIRED_LINE, "retirement-test"),
    ("tests/test_jetson_link_removal.py", RETIRED_LINE, "retirement-test"),
    (
        "tests/test_ansible_deploy_speed.py",
        r'def test_deploy_script_rejects_the_jetson_target|run_deploy_script\(\["jetson"',
        "retirement-test",
    ),
    ("tests/test_mcp_server_config.py", r'assert "fused_depth_topic" not in head$', "retirement-test"),
    (
        "tests/test_pi_cameras_config.py",
        r"def test_namespaces_are_not_the_overview_camera_namespace",
        "retirement-test",
    ),
    ("tests/test_pi_cameras_deploy.py", r"The retired /overview_camera/\* names must not be$", "retirement-test"),
    (
        "tests/test_pi_cameras_deploy.py",
        r"def test_web_ui_shows_the_previews_not_the_overview_camera_names|assert \"overview_camera\" not in text$",
        "retirement-test",
    ),
    (
        "tests/test_unit_cgroups.py",
        r'^        "(stereo_depth|stereo_depth_cpp|overview_camera|jetson_bridge_cpp)",$',
        "retirement-test",
    ),
    ("nodes/claude_agent/tests/test_prompt.py", r'"stereo" not in text\.lower\(\)', "retirement-test"),
    (
        "nodes/mcp_server/tests/test_tools.py",
        r"not_realsense_or_stereo|\"stereo\" not in text\.lower\(\)",
        "retirement-test",
    ),
    ("nodes/web_ui/tests/test_bridge.py", r"stereo|overview_camera", "retirement-test"),
    ("nodes/web_ui/tests/test_config.py", r"overview_camera|fused_depth|/stereo/depth/fused", "retirement-test"),
    ("nodes/web_ui/tests/test_imu_calibration.py", r"stereo", "retirement-test"),
    (
        "nodes/web_ui/tests/test_lazy_serialization.py",
        r"The stereo-only color_small path is removed",
        "retirement-test",
    ),
    (
        "nodes/web_ui/tests/test_msg_serializer.py",
        r"retired stereo colour path is gone|/stereo/left/image_rect",
        "retirement-test",
    ),
    ("nodes/web_ui/tests/test_stale_names_gone.py", r"retired stereo colour path", "retirement-test"),
    # retirement-doc: README, tests/README, SKILL and MEMORY lines that record a retirement or decision.
    ("README.md", r"(?i)jetson|stereo|overview_camera", "retirement-doc"),
    ("ansible/README.md", r"(?i)jetson|stereo|overview_camera|10\.77\.0\.", "retirement-doc"),
    (
        "tests/README.md",
        r"(?i)jetson|stereo|overview_camera|depthd|depth_wire|fused_depth|10\.77\.0\.",
        "retirement-doc",
    ),
    (".claude/skills/ansible-deploy/SKILL.md", r"(?i)jetson|stereo|overview_camera", "retirement-doc"),
    ("MEMORY.md", r"(?i)jetson|stereo|overview_camera", "retirement-doc"),
    ("nodes/pi_cameras/README.md", r"The retired `/overview_camera/\*` names are not used here", "retirement-doc"),
    # pinned-build: part of the libcamera build stamp, changing it would rebuild.
    ("nodes/pi_cameras/patches/**", r"overview_camera", "pinned-build"),
)


def find_unallowed_references() -> list[str]:
    """Run git grep for the retired tokens and drop every hit matched by ALLOWED_REFERENCES.

    Returns:
        list[str]: `path:line: text` for each hit no allowlist entry covers.
    """
    result = subprocess.run(
        ["git", "grep", "-n", "-i", "-E", RETIRED_PATTERN],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    compiled = [(glob, re.compile(regex), category) for glob, regex, category in ALLOWED_REFERENCES]
    hits = []
    for raw in result.stdout.splitlines():
        path, _, rest = raw.partition(":")
        text = rest.partition(":")[2]
        if not any(fnmatch.fnmatchcase(path, glob) and rx.search(text) for glob, rx, _ in compiled):
            hits.append(f"{path}:{rest}")
    return hits


def test_allowed_references_use_only_the_known_categories() -> None:
    """Every entry carries one of the six categories, and a directory-wide glob appears only for the archive and pinned patches."""
    assert {category for _, _, category in ALLOWED_REFERENCES} <= ALLOWED_CATEGORIES
    wide = {glob for glob, _, _ in ALLOWED_REFERENCES if "**" in glob}
    assert wide <= {*ARCHIVE_GLOBS, "nodes/pi_cameras/patches/**"}


def test_no_live_jetson_stereo_or_overview_reference() -> None:
    """No tracked file outside the allowlisted retirement records mentions the Jetson, depthd, stereo or overview_camera."""
    hits = find_unallowed_references()
    assert not hits, "unallowed retired references:\n" + "\n".join(hits)
