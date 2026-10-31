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
ARCHIVE_GLOBS = ("docs/plans/**", "docs/superpowers/**", "docs/research/**")
ALLOWED_CATEGORIES = {
    "archive",
    "retired-entry",
    "uninstall-artifact",
    "retirement-test",
    "retirement-doc",
    "pinned-build",
    "rgbd-calibration",
}

# (path glob, line regex, category). Each entry is one file plus a line regex as narrow as the hit; the only
# directory-wide globs are the three archive directories and the pinned libcamera patches.
ALLOWED_REFERENCES: tuple[tuple[str, str, str], ...] = (
    # archive: finished plans and specs record history and stay untouched.
    ("docs/plans/**", r".", "archive"),
    ("docs/superpowers/**", r".", "archive"),
    ("docs/research/**", r".", "archive"),
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
    (
        "tests/test_ansible_deploy_speed.py",
        r'def test_deploy_script_rejects_the_jetson_target|run_deploy_script\(\["jetson"',
        "retirement-test",
    ),
    ("tests/test_mcp_server_config.py", r'assert "fused_depth_topic" not in head$', "retirement-test"),
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
    ("nodes/web_ui/tests/test_config.py", r"overview_camera|fused_depth|/stereo/depth/fused", "retirement-test"),
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
    ("tests/test_jetson_retired.py", "^def\\ test_jetson_bridge_entry_is_uninstalled_b", "retirement-test"),
    ("tests/test_jetson_retired.py", "^def\\ test_jetson_bridge_type_keeps_only_the_un", "retirement-test"),
    ("tests/test_jetson_retired.py", "^def\\ test_client_playbook_still_deploys_jetson_bridge_so_t", "retirement-test"),
    ("tests/test_jetson_retired.py", "^def\\ test_jetson_bridge_metrics_target_is_remo", "retirement-test"),
    (
        "tests/test_jetson_retired.py",
        "^\\ \\ \\ \\ \\ \\ \\ \\ r'\\^\\ \\ \\ \\ node_launch_command:\\ \"\\(ros2\\ launch\\ overview_camera\\ overview_ca",
        "retirement-test",
    ),
    (
        "tests/test_jetson_retired.py",
        "^\\ \\ \\ \\ \\ \\ \\ \\ r\"\\^\\ \\ \\#\\ Retired\\ \\\\\\(D7\\\\\\):\\ the\\ Pi's\\ stereo\\ pipeline\\ ne",
        "retirement-test",
    ),
    (
        "tests/test_jetson_retired.py",
        "^\\ \\ \\ \\ \\#\\ uninstall\\-artifact:\\ the\\ jetson_link\\ remova",
        "retirement-test",
    ),
    (
        "tests/test_jetson_retired.py",
        "^\\ \\ \\ \\ \\ \\ \\ \\ r'def\\ test_deploy_script_rejects_the_jetson_target\\|run_",
        "retirement-test",
    ),
    (
        "tests/test_jetson_retired.py",
        "^\\ \\ \\ \\ \\ \\ \\ \\ r'\\^\\ \\ \\ \\ \\ \\ \\ \\ \"\\(stereo_depth\\|stereo_dept",
        "retirement-test",
    ),
    ("tests/test_jetson_retired.py", "^def\\ test_no_live_jetson_stereo_or_overview_re", "retirement-test"),
    (
        "tests/test_stereo_depth_config.py",
        '^"""Static\\ invariants\\ of\\ the\\ stereo_depth\\ /\\ ste',
        "retirement-test",
    ),
    (
        "tests/test_stereo_depth_config.py",
        "^The\\ Pi's\\ stereo\\ pipeline\\ never\\ produced\\ depth",
        "retirement-test",
    ),
    # The pin itself names every retired token it checks and lists.
    ("tests/test_jetson_retired.py", RETIRED_LINE, "retirement-test"),
    ("tests/test_stereo_depth_config.py", '^NODES\\ =\\ \\("stereo_depth",\\ "stereo_depth_cpp"\\)', "retirement-test"),
    (
        "tests/test_stereo_depth_config.py",
        "^def\\ test_background_tier_has_no_stereo_type\\(\\)\\ \\->\\ N",
        "retirement-test",
    ),
    (
        "tests/test_stereo_depth_config.py",
        '^\\ \\ \\ \\ assert\\ "\\|\\ stereo_depth"\\ not\\ in\\ readme,\\ "n',
        "retirement-test",
    ),
    (
        "tests/test_stereo_depth_config.py",
        "^def\\ test_tests_readme_metrics_wiring_rows_have_no_stereo_node\\(\\)\\ \\->\\ N",
        "retirement-test",
    ),
    (
        "tests/test_stereo_depth_config.py",
        '^\\ \\ \\ \\ \\ \\ \\ \\ assert\\ "stereo_depth"\\ not\\ in\\ row\\ and\\ ',
        "retirement-test",
    ),
    (
        "tests/test_stereo_depth_config.py",
        '^\\ \\ \\ \\ assert\\ "\\#\\#\\#\\ test_stereo_depth_config\\.py"\\ ',
        "retirement-test",
    ),
    (
        "tests/test_stereo_depth_config.py",
        '^@pytest\\.mark\\.parametrize\\("name",\\ \\["stereo_depth",\\ "st',
        "retirement-test",
    ),
    ("tests/test_stereo_depth_config.py", "^def\\ test_stereo_sources_are_deleted\\(name:\\ str", "retirement-test"),
    (
        "tests/test_stereo_depth_config.py",
        '^\\ \\ \\ \\ """The\\ retired\\ stereo\\ node\'s\\ sources\\ are\\ ',
        "retirement-test",
    ),
    ("tests/test_pi_cameras_retired.py", "^The\\ overview_camera\\ ros2_nodes\\ entry\\ is\\ prese", "retirement-test"),
    (
        "tests/test_pi_cameras_retired.py",
        '^OLD_TASKS\\ =\\ TASKS\\ /\\ "overview_camera_boot_config',
        "retirement-test",
    ),
    ("tests/test_pi_cameras_retired.py", "^def\\ test_overview_camera_entry_is_uninstalled", "retirement-test"),
    (
        "tests/test_pi_cameras_retired.py",
        '^\\ \\ \\ \\ node\\ =\\ next\\(n\\ for\\ n\\ in\\ load_client\\(\\)\\["ros2_nodes"\\]\\ if\\ n\\["name"\\]\\ ==\\ "overview_camera"\\)',
        "retirement-test",
    ),
    (
        "tests/test_pi_cameras_retired.py",
        '^\\ \\ \\ \\ assert\\ node\\["node_type"\\]\\ ==\\ "overview_camera"',
        "retirement-test",
    ),
    ("tests/test_pi_cameras_retired.py", "^def\\ test_overview_camera_entry_names_d7_in_it", "retirement-test"),
    (
        "tests/test_pi_cameras_retired.py",
        '^\\ \\ \\ \\ index\\ =\\ lines\\.index\\("\\ \\ \\-\\ name:\\ overview_camera"\\)',
        "retirement-test",
    ),
    ("tests/test_pi_cameras_retired.py", "^def\\ test_overview_camera_node_type_keeps_only", "retirement-test"),
    (
        "tests/test_pi_cameras_retired.py",
        '^\\ \\ \\ \\ defaults\\ =\\ load_client\\(\\)\\["ros2_node_type_defaults"\\]\\["overview_camera"\\]',
        "retirement-test",
    ),
    (
        "tests/test_pi_cameras_retired.py",
        '^\\ \\ \\ \\ assert\\ not\\ \\(REPO_ROOT\\ /\\ "tests"\\ /\\ "test_overview_camera_config\\.py"\\)',
        "retirement-test",
    ),
    (
        "tests/test_pi_cameras_retired.py",
        '^\\ \\ \\ \\ assert\\ not\\ \\(REPO_ROOT\\ /\\ "tests"\\ /\\ "test_overview_camera_launch_conf',
        "retirement-test",
    ),
    (
        "tests/test_pi_cameras_retired.py",
        "^def\\ test_client_playbook_still_deploys_overview_camera_so_the_unin",
        "retirement-test",
    ),
    (
        "tests/test_pi_cameras_retired.py",
        '^\\ \\ \\ \\ assert\\ "_deploy_node_name:\\ overview_camera"\\ in\\ CLIENT_',
        "retirement-test",
    ),
    (
        "tests/test_pi_cameras_retired.py",
        '^\\ \\ \\ \\ assert\\ "overview_camera_boot_config"\\ not\\ ',
        "retirement-test",
    ),
    (
        "tests/test_pi_cameras_retired.py",
        '^\\ \\ \\ \\ assert\\ "\\|\\ overview_camera\\ \\|"\\ not\\ in\\ readm',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^"""Removal\\ of\\ the\\ retired\\ Pi\\-Jetson\\ link\\ from\\ t',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^REMOVE_TASKS\\ =\\ ANSIBLE_DIR\\ /\\ "playbooks"\\ /\\ "tasks"\\ /\\ "jetson_link_remove',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^BLOCK_NAME\\ =\\ "Remove\\ the\\ retired\\ Jetson\\ link"',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        "^\\#\\ The\\ regexp\\ the\\ jetson_link\\ role's\\ hosts\\ tas",
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^HOSTS_REGEXP\\ =\\ r"\\\\sjetson\\\\\\.ros2\\\\\\.lan\\(\\\\s\\|\\$\\)"',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^SYSCTL_DROPIN\\ =\\ "/etc/sysctl\\.d/90\\-jetson\\-link\\-forwar',
        "retirement-test",
    ),
    ("tests/test_jetson_link_removal.py", '^NAT_FILE\\ =\\ "/etc/nftables\\.d/jetson\\-nat\\.nft"', "retirement-test"),
    (
        "tests/test_jetson_link_removal.py",
        '^\\ \\ \\ \\ """The\\ client\\ play\'s\\ jetson_link\\ block\\.',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^\\ \\ \\ \\ delete\\ =\\ nft_task\\(tasks,\\ "nft\\ delete\\ table\\ ip\\ jetson_link"\\)',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^\\ \\ \\ \\ assert\\ "table\\ ip\\ jetson_link"\\ in\\ when_of\\(',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        "^def\\ test_hosts_regexp_matches_only_the_jetson_line\\(\\)\\ \\->\\ N",
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^\\ \\ \\ \\ assert\\ pattern\\.search\\("10\\.77\\.0\\.2\\ jetson\\.ros',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^\\ \\ \\ \\ assert\\ not\\ pattern\\.search\\("10\\.0\\.0\\.9\\ myjetson\\.ros2\\.lan"\\)',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^\\ \\ \\ \\ assert\\ not\\ pattern\\.search\\("10\\.0\\.0\\.9\\ jetson\\.ros2\\.lan\\.ex',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^\\ \\ \\ \\ assert\\ block\\["tags"\\]\\ ==\\ \\["jetson_link"\\]',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^\\ \\ \\ \\ assert\\ inner\\[2\\]\\["ansible\\.builtin\\.include_tasks"\\]\\["file"\\]\\ ==\\ "tasks/jetson_link_remove',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^\\ \\ \\ \\ \\ \\ \\ \\ assert\\ args\\["apply"\\]\\["tags"\\]\\ ==\\ \\["jetson_link"\\]',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        "^def\\ test_no_other_client_task_carries_the_jetson_link_tag\\(\\)\\ ",
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^\\ \\ \\ \\ assert\\ text\\.count\\("jetson_link\\]"\\)\\ ==\\ 4\\ \\ \\#',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^\\ \\ \\ \\ assert\\ "Direct\\ link\\ to\\ the\\ Jetson"\\ not\\ in\\ tex',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^\\ \\ \\ \\ assert\\ "jetson_link_side"\\ not\\ in\\ text',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^\\ \\ \\ \\ assert\\ "jetson_link"\\ not\\ in\\ str\\(yaml\\.safe',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^\\ \\ \\ \\ assert\\ not\\ \\(ANSIBLE_DIR\\ /\\ "roles"\\ /\\ "jetson_link"\\)\\.exis',
        "retirement-test",
    ),
    (
        "tests/test_jetson_link_removal.py",
        '^\\ \\ \\ \\ assert\\ not\\ \\(REPO_ROOT\\ /\\ "tests"\\ /\\ "test_jetson_link_role\\.p',
        "retirement-test",
    ),
    ("tests/test_jetson_link_removal.py", '^\\ \\ \\ \\ assert\\ "jetson_link_"\\ not\\ in\\ text', "retirement-test"),
    (
        "tests/test_jetson_link_removal.py",
        '^\\ \\ \\ \\ assert\\ not\\ any\\(key\\.startswith\\("jetson_link"\\)\\ for\\ ',
        "retirement-test",
    ),
    ("nodes/web_ui/tests/test_bridge.py", "^def\\ test_stale_stereo_rect_topics_are_gone_an", "retirement-test"),
    (
        "nodes/web_ui/tests/test_bridge.py",
        '^\\ \\ \\ \\ """The\\ retired\\ /stereo/\\*_rect\\ names\\ have\\ ',
        "retirement-test",
    ),
    (
        "nodes/web_ui/tests/test_bridge.py",
        '^\\ \\ \\ \\ for\\ stale\\ in\\ \\("/stereo/left/image_rect",\\ ',
        "retirement-test",
    ),
    (
        "nodes/web_ui/tests/test_bridge.py",
        '^\\ \\ \\ \\ assert\\ subscription_spec\\("/overview_camera/image_raw/c',
        "retirement-test",
    ),
    (
        "nodes/web_ui/tests/test_bridge.py",
        "^def\\ test_head_camera_resolves_and_overview_camera_is_gone\\(\\)\\ \\-",
        "retirement-test",
    ),
    (
        "nodes/web_ui/tests/test_bridge.py",
        '^\\ \\ \\ \\ """/head_camera\\ resolves\\ to\\ CompressedImage;\\ the\\ retired\\ /overview_camera',
        "retirement-test",
    ),
    ("tests/README.md", "^\\-\\ Head\\ tab\\ \\(`head`\\):", "retirement-test"),
    (
        "nodes/web_ui/tests/test_bridge.py",
        '^\\ \\ \\ \\ """/front_camera\\ resolves\\ to\\ CompressedImage;\\ the\\ retired\\ /overview_camera\\ name\\ has\\ no',
        "retirement-test",
    ),
    (
        "nodes/web_ui/tests/test_bridge.py",
        '^\\ \\ \\ \\ assert\\ "/overview_camera/image_raw/compre',
        "retirement-test",
    ),
    (
        "nodes/web_ui/tests/test_imu_calibration.py",
        "^def\\ test_stereo_calibration_routes_are_gone\\(t",
        "retirement-test",
    ),
    (
        "nodes/web_ui/tests/test_imu_calibration.py",
        '^\\ \\ \\ \\ assert\\ client\\.get\\("/api/calibration/stereo/preview"\\)\\.s',
        "retirement-test",
    ),
    # retirement-doc: README, tests/README, SKILL and MEMORY lines that record a retirement or decision.
    ("ansible/README.md", "^\\#\\#\\ Retired\\ Jetson\\ link\\ removal", "retirement-doc"),
    ("ansible/README.md", "^The\\ Pi\\-Jetson\\ cable\\ is\\ retired\\.\\ `\\./scripts/de", "retirement-doc"),
    ("ansible/README.md", "^\\-\\ the\\ `jetson\\.ros2\\.lan`\\ line\\ from\\ `/etc/hosts", "retirement-doc"),
    ("ansible/README.md", "^\\-\\ `/etc/sysctl\\.d/90\\-jetson\\-link\\-forward\\.conf`", "retirement-doc"),
    ("ansible/README.md", "^\\-\\ the\\ live\\ nftables\\ table\\ `ip\\ jetson_link`\\ \\(only", "retirement-doc"),
    (
        "ansible/README.md",
        "^\\*\\*CSI\\ camera\\ boot\\ config\\.\\*\\*\\ `playbooks/tasks/pi_camera_boot_config\\.yml`\\ \\(included\\ from\\ `pre_tasks`\\ of\\ `deploy_nodes_client\\.yml`,\\ tags\\ `boot`,\\ `pi_cameras`,\\ `arducam_tof`\\ and\\ `head_camera_cpp`",
        "retirement-doc",
    ),
    (
        "ansible/README.md",
        "^\\*\\*Front/back\\ camera\\ previews\\ \\(`pi_cameras`\\)\\.\\*\\*\\ Native\\ node\\ type\\ in\\ `group_vars/client\\.yml`\\ \\(`ros2\\ launch\\ nodes/pi_cameras/launch/pi_cameras\\.launch\\.py`,\\ config\\ in\\ `/etc/ros2/pi_cameras/config\\.yaml`\\ via\\ `PI_CAMERAS_CONFIG`\\)\\ for\\ the\\ Camera\\ Module\\ 3\\ \\(IMX708\\)\\ on\\ the\\ Pi\\ 5\\ CSI\\ ports\\.\\ Only\\ the\\ front\\ one\\ \\(cam0\\)\\ is\\ fitted:\\ its\\ `config:\\ \\|`\\ holds\\ `roles:\\ \\[front\\]`\\ because\\ cam1\\ carries\\ the\\ Arducam\\ ToF,\\ and\\ the\\ boot\\ config\\ writes\\ `dtoverlay=imx708,cam0`\\.\\ It\\ builds\\ the\\ same\\ pinned\\ libcamera\\ and\\ camera_ros\\ sources\\ as\\ the\\ retired\\ `overview_camera`\\ \\(stamps\\ un",
        "retirement-doc",
    ),
    ("ansible/README.md", "^\\*\\*Retired\\ stereo\\ nodes\\ \\(`stereo_depth`,\\ `ster", "retirement-doc"),
    ("ansible/README.md", "^\\*\\*`jetson_bridge_cpp`\\ \\(retired\\ with\\ the\\ Jetso", "retirement-doc"),
    (
        "ansible/README.md",
        "^`map_guard_cpp`\\ is\\ `present:\\ true`,\\ `enabled:\\ true`;\\ `map_guard`\\ stays\\ `present:\\ true`,\\ `enabled:\\ false`\\ \\(installed,\\ stopped,\\ disabled\\)\\ so\\ the\\ rollback\\ is\\ one\\ line\\.\\ The\\ `_cpp`\\ config\\ block\\ equals\\ the\\ Python\\ one\\ \\(`map_guard_cpp`\\ keeps\\ `dry_run:\\ true`\\),\\ and\\ `monitoring_agent_node_targets`\\ scrapes\\ the\\ `_cpp`\\ name\\ on\\ the\\ shared\\ port\\ \\(19110\\)\\.\\ `stereo_depth`\\ and\\ ",
        "retirement-doc",
    ),
    (
        "ansible/README.md",
        "^\\*\\*Purging\\ retired\\ C\\+\\+\\ nodes\\ \\(`colcon_purge`\\)\\.\\*\\*\\ A\\ node\\ type\\ may\\ carry\\ `colcon_purge`,\\ a\\ list\\ of\\ `\\{package,\\ workspace\\}`\\ mappings\\ \\(for\\ example\\ `\\{package:\\ jetson_bridge_cpp,",
        "retirement-doc",
    ),
    (
        "tests/README.md",
        "^units,\\ weight\\ tiers,\\ `MemoryLow`,\\ `MemoryMax`\\ safety\\ net,\\ `Restart=always`,\\ host\\ slice\\ drop\\-ins\\)\\.\\ The\\ C\\+\\+\\ ports\\ sit\\ in\\ their\\ Python\\ node's\\ tier:\\ `map_guard_cpp`\\ normal;\\ the\\ retired\\ `stereo_depth`,\\ `st",
        "retirement-doc",
    ),
    (
        "tests/README.md",
        "^\\-\\ config\\ loading\\ \\(`test_config\\.py`:\\ minimal\\ config\\ defaults,\\ http_port\\ override,\\ missing\\ file\\ raises\\ `FileNotFoundError`,\\ `all_subscribed_topics`\\ for\\ camera/nav/overlay\\ tabs,\\ `load_config`\\ from\\ env\\ var\\ `WEB_UI_CONFIG`,\\ `publish_topics`\\ includes\\ goal_topic,\\ `rgbd_camera`\\ tab\\ type\\ valid\\ with\\ color/depth/camera_info\\ topics,\\ RGBD\\ topics\\ included\\ in\\ `all_subscribed_topics`,\\ `arm`\\ is\\ a\\ valid\\ tab\\ type\\ with\\ defaults\\ \\(topics,\\ mount,\\ arm\\ offset,\\ `active_source_topic`,\\ Trigger\\ services\\),\\ unset\\-field\\ overrides,\\ an\\ empty\\ topic\\ rejected,\\ other\\ tab\\ types\\ without\\ arm\\ defaults,\\ arm\\ topics\\ subscribed\\ and\\ arm\\ services\\ in\\ `trigger_services\\(\\)`,\\ the\\ shipped\\ default\\ config\\ has\\ the\\ `arm`\\ tab\\ in\\ the\\ old\\ Gripper\\ Cam\\ slot\\ \\(no\\ `camera`\\ id\\),\\ no\\ `overview_camera`\\ tab\\ or\\ `/o",
        "retirement-doc",
    ),
    (
        "tests/README.md",
        "^\\-\\ bridge\\ dirty\\-flag\\ store\\ \\(`test_bridge\\.py`:\\ `/front_camera/image_raw/compressed`\\ resolves\\ to\\ a\\ `CompressedImage`\\ subscription\\ and\\ `/overview_camera/image_raw/c",
        "retirement-doc",
    ),
    (
        "tests/README.md",
        '^\\-\\ Calibrate\\ backend\\ \\(`nodes/web_ui/tests/test_imu_calibration\\.py`,\\ built\\ with\\ `CalibrationConfig\\(\\)`\\):\\ `GET\\ /api/calibration/config`\\ returns\\ exactly\\ `\\{"imu":\\ true,\\ "arm":\\ "deferred"\\}`\\ with\\ a\\ `calibration`\\ block\\ and\\ 404\\ `calibration\\ page\\ not\\ configured`\\ without;\\ `GET\\ /api/calibration/stereo/preview`\\ is',
        "retirement-doc",
    ),
    (
        "tests/README.md",
        "^\\-\\ Head\\ tab\\ \\(`head`\\):\\ `test_config\\.py`\\ \\(`head`\\ is\\ a\\ valid\\ tab\\ type;\\ fields\\ load\\ and\\ are\\ subscribed;\\ topics\\ default\\ to\\ None\\ and\\ `depth_range_m`\\ to\\ 4\\.0\\ m;\\ a\\ non\\-positive\\ range\\ is\\ rejected;\\ `test_head_tab_depth_keys_replace_tof_keys`:\\ `depth_topic`\\ and\\ `depth_range_m`\\ are\\ fields,\\ the\\ ToF\\ keys\\ are\\ not\\ and\\ are\\ rejected\\ as\\ unknown;\\ `topic_roles`\\ gives\\ `head_rgb`\\ /\\ `head_depth`\\ and\\ `depth_ranges`\\ maps\\ the\\ depth\\ topic\\ to\\ its\\ range;\\ `test_head_tab_rejects_fused_depth_topic`:\\ `fu",
        "retirement-doc",
    ),
    ("tests/README.md", "^\\#\\#\\#\\ test_stereo_depth_config\\.py", "retirement-doc"),
    (
        "tests/README.md",
        "^Static\\ checks\\ of\\ the\\ D7\\ retirement\\ of\\ the\\ Pi\\ stereo\\ pipeline:\\ `",
        "retirement-doc",
    ),
    ("tests/README.md", "^\\|\\ `test_background_tier_has_no_stereo_type`\\ \\|\\ The", "retirement-doc"),
    (
        "tests/README.md",
        "^\\|\\ `test_metrics_target_is_removed`\\ \\|\\ `monitoring_agent_node_targets`\\ has\\ no\\ stereo\\ node\\ and\\ no",
        "retirement-doc",
    ),
    (
        "tests/README.md",
        '^\\|\\ `test_ansible_readme_documents_the_retirement`\\ \\|\\ `ansible/README\\.md`\\ "Cut\\-over\\ and\\ rollback"\\ names\\ D7\\ and\\ `present:\\ false`,\\ has\\ no\\ 19113\\ and\\ no\\ stereo\\ row\\ in\\ the\\ ',
        "retirement-doc",
    ),
    (
        "tests/README.md",
        "^\\|\\ `test_tests_readme_metrics_wiring_rows_have_no_stereo_node`\\ \\|\\ The",
        "retirement-doc",
    ),
    ("tests/README.md", "^\\|\\ `test_stereo_sources_are_deleted`\\ \\|\\ `nodes/", "retirement-doc"),
    (
        "tests/README.md",
        "^\\|\\ `test_web_ui_shows_the_previews_not_the_overview_camera_names`\\ \\|\\ Th",
        "retirement-doc",
    ),
    ("tests/README.md", "^\\|\\ `test_namespaces_are_not_the_overview_camera_namespace`\\ ", "retirement-doc"),
    ("tests/README.md", "^\\|\\ `test_overview_camera_entry_is_uninstalled_", "retirement-doc"),
    ("tests/README.md", "^\\|\\ `test_overview_camera_entry_names_d7_in_its", "retirement-doc"),
    ("tests/README.md", "^\\|\\ `test_overview_camera_node_type_keeps_only_", "retirement-doc"),
    (
        "tests/README.md",
        "^\\|\\ `test_only_pi_cameras_builds_libcamera_and_camera_ros`\\ \\|\\ Of\\ all\\ client\\ node\\ types\\ only\\ `pi_cameras`\\ builds\\ libcamera\\ \\(the\\ retired\\ `overview_camera`\\ builds\\ not",
        "retirement-doc",
    ),
    (
        "tests/README.md",
        "^\\|\\ `test_old_boot_file_and_old_tests_are_deleted`\\ \\|\\ `overview_camera_boot_config",
        "retirement-doc",
    ),
    ("tests/README.md", "^\\|\\ `test_client_playbook_still_deploys_overview_camera_so_the_unin", "retirement-doc"),
    (
        "tests/README.md",
        "^\\|\\ `test_ansible_readme_describes_the_removal`\\ \\|\\ `ansible/README\\.md`\\ names\\ neither\\ removed\\ boot\\ file\\ and\\ has\\ no\\ memory\\ row\\ for\\ `overview_camera`\\.\\ \\|",
        "retirement-doc",
    ),
    (
        "tests/README.md",
        "^\\|\\ `test_non_node_targets_are_parsed`\\ \\|\\ `non_node_targets`\\ reads\\ `monitoring`\\ and\\ `jetson_link`\\ for\\ t",
        "retirement-doc",
    ),
    ("tests/README.md", "^\\|\\ `test_deploy_script_rejects_the_jetson_target`\\ \\|\\ T", "retirement-doc"),
    ("tests/README.md", "^\\#\\#\\ test_jetson_retired\\.py", "retirement-doc"),
    ("tests/README.md", "^Retirement\\ pin\\ for\\ the\\ Jetson\\ coprocessor\\.\\ Ex", "retirement-doc"),
    ("tests/README.md", "^\\|\\ `test_jetson_bridge_entry_is_uninstalled_by", "retirement-doc"),
    ("tests/README.md", "^\\|\\ `test_jetson_bridge_type_keeps_only_the_uni", "retirement-doc"),
    ("tests/README.md", "^\\|\\ `test_client_playbook_still_deploys_jetson_bridge_so_t", "retirement-doc"),
    ("tests/README.md", "^\\|\\ `test_jetson_bridge_metrics_target_is_remov", "retirement-doc"),
    ("tests/README.md", "^\\|\\ `test_no_live_jetson_stereo_or_overview_ref", "retirement-doc"),
    ("tests/README.md", "^\\#\\#\\ test_jetson_link_removal\\.py", "retirement-doc"),
    ("tests/README.md", "^The\\ removal\\ of\\ the\\ retired\\ Pi\\-Jetson\\ link\\ from\\ t", "retirement-doc"),
    (
        "tests/README.md",
        "^\\|\\ `test_removal_tasks_are_state_based_and_guarded`\\ \\|\\ `state:\\ absent`\\ for\\ the\\ `jetson\\.ros2\\.lan`\\ h",
        "retirement-doc",
    ),
    ("tests/README.md", "^\\|\\ `test_hosts_regexp_matches_only_the_jetson_line`\\ \\|\\ The", "retirement-doc"),
    (
        "tests/README.md",
        '^\\|\\ `test_client_block_runs_network_chrony_and_removal`\\ \\|\\ The\\ "Remove\\ the\\ retired\\ Jetson\\ link"\\ block',
        "retirement-doc",
    ),
    ("tests/README.md", "^\\|\\ `test_no_other_client_task_carries_the_jetson_link_tag`\\ \\|", "retirement-doc"),
    (
        "tests/README.md",
        "^\\|\\ `test_role_and_link_vars_are_gone`\\ \\|\\ No\\ `roles/jetson_link`,\\ no\\ `",
        "retirement-doc",
    ),
    (
        "tests/README.md",
        "^The\\ network\\ role's\\ netplan\\ rendering\\ after\\ the\\ Jetson\\ link\\ suppor",
        "retirement-doc",
    ),
    (
        ".claude/skills/ansible-deploy/SKILL.md",
        "^\\|\\ `realsense_d435i`\\ \\|\\ Removed\\ \\(present:\\ false\\):\\ replaced\\ by\\ `overview_camera`,\\ itself\\ retired;",
        "retirement-doc",
    ),
    (
        ".claude/skills/ansible-deploy/SKILL.md",
        "^\\|\\ `overview_camera`\\ \\|\\ Retired\\ \\(D7\\):\\ `present:",
        "retirement-doc",
    ),
    (
        ".claude/skills/ansible-deploy/SKILL.md",
        "^\\|\\ `stereo_depth`\\ \\|\\ Retired\\ \\(D7,\\ sources\\ delet",
        "retirement-doc",
    ),
    (
        ".claude/skills/ansible-deploy/SKILL.md",
        "^\\|\\ `stereo_depth_cpp`\\ \\|\\ Retired\\ \\(D7,\\ sources\\ d",
        "retirement-doc",
    ),
    (
        ".claude/skills/ansible-deploy/SKILL.md",
        "^\\|\\ `jetson_bridge_cpp`\\ \\|\\ Retired:\\ `present:\\ fa",
        "retirement-doc",
    ),
    ("MEMORY.md", "^\\#\\#\\ Stereo\\ depth\\ \\(IMX219\\ pair,\\ client\\)\\ \\-\\ SUPER", "retirement-doc"),
    (
        "MEMORY.md",
        "^\\ \\ \\-\\ The\\ IMX219\\ lens\\ is\\ fixed,\\ while\\ the\\ IMX708's\\ VCM\\ lens\\ drifts\\ even\\ in\\ manual\\ focus\\ \\(gravity\\ on\\ the\\ pan/tilt\\ head,\\ temperature,\\ vibration\\),\\ and\\ any\\ lens\\ shift\\ breaks\\ the\\ stereo\\ calibration",
        "retirement-doc",
    ),
    ("MEMORY.md", "^\\*\\ `nodes/stereo_depth`\\ publishes\\ depth\\ on\\ the", "retirement-doc"),
    (
        "MEMORY.md",
        "^\\*\\ No\\ calibration,\\ a\\ pair\\ skew\\ above\\ `sync_slop_s`\\ \\(2\\ ms;\\ libcamera\\ SyncMode\\ keeps\\ the\\ sensors\\ in\\ step\\),\\ a\\ size\\ mismatch\\ or\\ an\\ all\\-invalid\\ pair\\ means\\ nothing\\ is\\ published\\ \\(logged\\ and\\ counted\\)\\.\\ Calibration\\ lives\\ in\\ `/var/lib/ros2/camera_calibration/\\{stereo_left,stereo",
        "retirement-doc",
    ),
    ("MEMORY.md", "^\\#\\#\\ Jetson\\ removal\\ \\(2026\\-10\\-18\\)", "retirement-doc"),
    ("MEMORY.md", "^\\*\\ The\\ Jetson\\ coprocessor\\ is\\ removed\\ from\\ the\\ ", "retirement-doc"),
    ("MEMORY.md", "^\\*\\ `stereo_depth`\\ /\\ `stereo_depth_cpp`\\ sources", "retirement-doc"),
    (
        "MEMORY.md",
        "^\\*\\ The\\ web_ui\\ Head\\ tab\\ shows\\ the\\ Arducam\\ ToF\\ depth\\ \\+\\ amplitude,\\ with\\ no\\ RGB\\.\\ `/front_camera`\\ feeds\\ the\\ Arm\\ tab\\ and\\ the\\ mcp_server\\ `front`\\ camera\\.\\ The\\ Overview\\ tab\\ and\\ stereo\\ calibration",
        "retirement-doc",
    ),
    ("nodes/pi_cameras/README.md", r"The retired `/overview_camera/\*` names are not used here", "retirement-doc"),
    # rgbd-calibration (cont.): the D435 is a stereo camera and its depth error model says so; unrelated to the Jetson.
    (
        "nodes/bridges/realsense_d435/README.md",
        r"a stereo camera puts between the edge",
        "rgbd-calibration",
    ),
    (
        "nodes/head_rgbd_cpp/README.md",
        r"flying pixels a stereo camera puts between the",
        "rgbd-calibration",
    ),
    (
        "shared/README.md",
        r"flying pixels a stereo camera puts between the edge",
        "rgbd-calibration",
    ),
    (
        "nodes/claude_agent/tests/test_prompt.py",
        r'assert "overhead" not in lower and "stereo" not in lower',
        "rgbd-calibration",
    ),
    (
        "nodes/mcp_server/mcp_server/config.py",
        r"# D435 stereo baseline of the z\^2 error model",
        "rgbd-calibration",
    ),
    (
        "nodes/mcp_server/mcp_server/depth_point.py",
        r"D435 stereo (model|depth error estimate)",
        "rgbd-calibration",
    ),
    (
        "tests/README.md",
        r"test_depth_error_model_is_the_d435_stereo_model",
        "rgbd-calibration",
    ),
    (
        "tests/test_mcp_camera_arm_config.py",
        r"def test_depth_error_model_is_the_d435_stereo_model",
        "rgbd-calibration",
    ),
    # rgbd-calibration: the head ToF-to-RGB extrinsic is an OpenCV stereo calibration (cv2.stereoCalibrate and its
    # RMS), a new pair unrelated to the retired Jetson stereo depth.
    (
        "tests/README.md",
        r"(?i)stereo(_?calibrat\w*| calibration| rms| pairs|_rms\w*|_max_rms_px|_recovers\w*)",
        "rgbd-calibration",
    ),
    (
        "scripts/README.md",
        r"(?i)stereo(_?calibrat\w*| calibration| rms| pairs|_rms\w*|_max_rms_px|_recovers\w*)",
        "rgbd-calibration",
    ),
    (
        "docs/head-rgbd-fusion-methodology.md",
        r"(?i)stereo(_?calibrat\w*| calibration| extrinsic| rms| pairs|_rms\w*|_max_rms_px|_recovers\w*)",
        "rgbd-calibration",
    ),
    (
        "ansible/group_vars/client.yml",
        r"(?i)stereo(_?calibrat\w*| calibration| rms| pairs|_rms\w*|_max_rms_px|_recovers\w*)",
        "rgbd-calibration",
    ),
    (
        "nodes/head_rgbd_calibration/README.md",
        r"(?i)stereo(_?calibrat\w*| calibration| rms| pairs|_rms\w*|_max_rms_px|_recovers\w*|_error_px| fit|`|, flag)",
        "rgbd-calibration",
    ),
    (
        "nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml",
        r"(?i)stereo(_?calibrat\w*| calibration| rms| pairs|_rms\w*|_max_rms_px|_recovers\w*)",
        "rgbd-calibration",
    ),
    (
        "nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py",
        r"(?i)stereo(_?calibrat\w*| calibration|[ _]extrinsic\w*| rms| pairs|_pairs|_rms\w*|_max_rms_px|_recovers\w*|_error_px| fit|`|, flag)",
        "rgbd-calibration",
    ),
    (
        "nodes/head_rgbd_calibration/tests/test_compute.py",
        r"(?i)stereo(_?calibrat\w*| calibration|[ _]extrinsic\w*| rms| pairs|_pairs|_rms\w*|_max_rms_px|_recovers\w*)",
        "rgbd-calibration",
    ),
    (
        "nodes/web_ui/README.md",
        r"(?i)stereo(_?calibrat\w*| calibration| rms| pairs|_rms\w*|_max_rms_px|_recovers\w*|_error_px| \$\{f| - px| z px)",
        "rgbd-calibration",
    ),
    (
        "nodes/web_ui/frontend/src/calibrate/RgbdResults.tsx",
        r"(?i)stereo(_?calibrat\w*| calibration| rms| pairs|_rms\w*|_max_rms_px|_recovers\w*)",
        "rgbd-calibration",
    ),
    (
        "nodes/head_rgbd_calibration/tests/fixtures/api/result.json",
        r"(?i)stereo(_?calibrat\w*| calibration| rms| pairs|_pairs|_rms\w*|_max_rms_px|_recovers\w*|_error_px| \$\{f| - px| z px)",
        "rgbd-calibration",
    ),
    (
        "nodes/web_ui/frontend/src/calibrate/RgbdResults.test.tsx",
        r"(?i)stereo(_?calibrat\w*| calibration| rms| pairs|_rms\w*|_max_rms_px|_recovers\w*)",
        "rgbd-calibration",
    ),
    (
        "nodes/web_ui/frontend/src/calibrate/RgbdCapture.test.tsx",
        r"(?i)stereo(_error_px| \$\{f| - px| z px)",
        "rgbd-calibration",
    ),
    (
        "nodes/web_ui/frontend/src/calibrate/RgbdCapture.tsx",
        r"(?i)stereo(_error_px| \$\{f| - px| z px)",
        "rgbd-calibration",
    ),
    (
        "nodes/web_ui/frontend/src/calibrate/rgbdApi.test.ts",
        r"(?i)stereo(_error_px| \$\{f| - px| z px)",
        "rgbd-calibration",
    ),
    (
        "nodes/web_ui/frontend/src/calibrate/rgbdApi.ts",
        r"(?i)stereo",  # RGB-D calibration: the ToF-to-RGB stereo fit and its per-pose errors
        "rgbd-calibration",
    ),
    (
        "shared/ros2_common/head_rgbd_calibration/calibrate.py",
        r"(?i)stereo",  # RGB-D calibration: the ToF-to-RGB stereo fit and its per-pose errors
        "rgbd-calibration",
    ),
    (
        "shared/ros2_common/head_rgbd_calibration/hints.py",
        r"(?i)stereo",  # RGB-D calibration: the ToF-to-RGB stereo fit and its per-pose errors
        "rgbd-calibration",
    ),
    (
        "tests/test_head_rgbd_calib_hints.py",
        r"(?i)stereo",  # RGB-D calibration: the ToF-to-RGB stereo fit and its per-pose errors
        "rgbd-calibration",
    ),
    (
        "shared/README.md",
        r"(?i)stereo(_?calibrat\w*| calibration| rms| pairs|_rms\w*|_max_rms_px|_px|_recovers\w*| error per pair)",
        "rgbd-calibration",
    ),
    (
        "shared/ros2_common/head_rgbd_calibration/calibrate.py",
        r"(?i)stereo(_?calibrat\w*| calibration|[ _]extrinsic\w*| rms| pairs|_pairs|_rms\w*|_max_rms_px|_recovers\w*)",
        "rgbd-calibration",
    ),
    (
        "shared/ros2_common/head_rgbd_calibration/config.py",
        r"(?i)stereo(_?calibrat\w*| calibration| rms| pairs|_rms\w*|_max_rms_px|_recovers\w*)",
        "rgbd-calibration",
    ),
    (
        "shared/ros2_common/head_rgbd_calibration/urdf_text.py",
        r"(?i)stereo(_?calibrat\w*| calibration| rms| pairs|_rms\w*|_max_rms_px|_recovers\w*)",
        "rgbd-calibration",
    ),
    (
        "tests/test_head_rgbd_calib_core.py",
        r"(?i)stereo(_?calibrat\w*| calibration|[ _]extrinsic\w*| rms| pairs|_pairs|_rms\w*|_max_rms_px|_recovers\w*)",
        "rgbd-calibration",
    ),
    # retirement-test: the pins of the retired ToF-to-RGB extrinsics and its selection and limit keys.
    (
        "tests/test_head_rgbd_calib_retired.py",
        r"(?i)stereo(_max_rms_px|_modules|_module_import| extrinsics)",
        "retirement-test",
    ),
    ("nodes/head_rgbd_calibration/tests/test_config.py", r'"stereo_max_rms_px": 1\.0,$', "retirement-test"),
    # pinned-build: part of the libcamera build stamp, changing it would rebuild.
    ("nodes/head_camera_cpp/patches/**", r"overview_camera", "pinned-build"),
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
    """Every entry carries one of the seven categories, and a directory-wide glob appears only for the archive and pinned patches."""
    assert {category for _, _, category in ALLOWED_REFERENCES} <= ALLOWED_CATEGORIES
    wide = {glob for glob, _, _ in ALLOWED_REFERENCES if "**" in glob}
    assert wide <= {*ARCHIVE_GLOBS, "nodes/head_camera_cpp/patches/**"}


def test_no_live_jetson_stereo_or_overview_reference() -> None:
    """No tracked file outside the allowlisted retirement records mentions the Jetson, depthd, stereo or overview_camera."""
    hits = find_unallowed_references()
    assert not hits, "unallowed retired references:\n" + "\n".join(hits)
