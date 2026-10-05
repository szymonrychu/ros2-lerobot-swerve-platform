"""Static invariants of the robot MCP server wiring (Ansible, unit template, filter_node lease, web_ui tabs, setup).

The mcp_server node (nodes/mcp_server) serves MCP over Streamable HTTP on the client; its bearer token lives only on the
robot in a 0600 EnvironmentFile created by Ansible, never in git. Claude Code reads it via scripts/robot_mcp_token.sh
and the repo-root .mcp.json expands ${ROBOT_MCP_TOKEN}.
"""

import ast
import json
import os
import re
import subprocess
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
CLIENT_VARS = ANSIBLE_DIR / "group_vars" / "client.yml"
PLAYBOOKS_DIR = ANSIBLE_DIR / "playbooks"
ROLE_DIR = ANSIBLE_DIR / "roles" / "ros2_node_deploy"
SERVICE_TEMPLATE = ROLE_DIR / "templates" / "ros2-node-native.service.j2"
RESOLVE_TASKS = PLAYBOOKS_DIR / "tasks" / "resolve_and_deploy.yml"
SETUP_TASKS = PLAYBOOKS_DIR / "tasks" / "mcp_server_setup.yml"
NODE_PLAYBOOK = PLAYBOOKS_DIR / "nodes" / "client" / "mcp_server.yml"
MCP_JSON = REPO_ROOT / ".mcp.json"
TOKEN_SCRIPT = REPO_ROOT / "scripts" / "robot_mcp_token.sh"
TESTS_README = REPO_ROOT / "tests" / "README.md"
NODE_DIR = REPO_ROOT / "nodes" / "mcp_server"
NODES_README = REPO_ROOT / "nodes" / "README.md"

CONFIG_DIR = "/etc/ros2/mcp_server"
TOKEN_FILE = "/etc/ros2/mcp_server/token"
ARM_DIR = "/var/lib/ros2/arm"
PORT = 18200
MCP_URL = "http://client.ros2.lan:18200/mcp"
AUTONOMY = {
    "autonomy_input_topic": "/filter/autonomy_joint_commands",
    "autonomy_release_topic": "/filter/autonomy_release",
    "active_source_topic": "/filter/active_source",
}
TILE_CACHE_DIR = "/var/cache/web_ui/tiles"
TILE_CACHE_TASKS = PLAYBOOKS_DIR / "tasks" / "web_ui_tile_cache_dir.yml"
WEB_UI_PLAYBOOK = PLAYBOOKS_DIR / "nodes" / "client" / "web_ui.yml"
LEGACY_MAP_KEYS = {"urdf_file", "topic", "arm_urdf_file", "arm_joint_topic", "scan_topic", "costmap_topic"}
WEB_UI_TABS = ["map", "agent", "camera", "rgbd_camera", "imu_graphs"]
REMOVED_WEB_UI_TABS = {"arm_servos", "local_nav", "gps_nav", "scene3d", "robot_status"}


def client_vars() -> dict:
    """Load group_vars/client.yml.

    Returns:
        dict: Parsed YAML.
    """
    return yaml.safe_load(CLIENT_VARS.read_text())


def node_entry(name: str) -> dict:
    """Return one client ros2_nodes entry.

    Args:
        name: Node name.

    Returns:
        dict: The entry.
    """
    return next(n for n in client_vars()["ros2_nodes"] if n["name"] == name)


def node_config(name: str) -> dict:
    """Parse the `config: |` block of one client node.

    Args:
        name: Node name.

    Returns:
        dict: Parsed config.
    """
    return yaml.safe_load(node_entry(name)["config"])


def play_tasks(path: Path) -> tuple[list[dict], list[dict], list[dict]]:
    """Pre-tasks, tasks and post-tasks of the first play in a playbook.

    Args:
        path: Playbook path.

    Returns:
        tuple[list[dict], list[dict], list[dict]]: (pre_tasks, tasks, post_tasks).
    """
    play = yaml.safe_load(path.read_text())[0]
    return play.get("pre_tasks", []), play.get("tasks", []), play.get("post_tasks", [])


def deploy_index(tasks: list[dict], name: str) -> int:
    """Index of the task deploying a node through resolve_and_deploy.yml.

    Args:
        tasks: Task list.
        name: Node name.

    Returns:
        int: Task index (-1 when absent).
    """
    for i, t in enumerate(tasks):
        if "resolve_and_deploy.yml" in str(t.get("ansible.builtin.include_tasks", "")) and (
            t.get("vars", {}).get("_deploy_node_name") == name
        ):
            return i
    return -1


def include_index(tasks: list[dict], filename: str) -> int:
    """Index of the task including a task file.

    Args:
        tasks: Task list.
        filename: Included file name.

    Returns:
        int: Task index (-1 when absent).
    """
    return next((i for i, t in enumerate(tasks) if filename in str(t.get("ansible.builtin.include_tasks", ""))), -1)


def test_mcp_server_node_type_defaults() -> None:
    d = client_vars()["ros2_node_type_defaults"]["mcp_server"]
    assert d["deploy_mode"] == "native"
    assert d["node_src_dir"] == "nodes/mcp_server"
    assert d["node_launch_command"] == "python3 -m mcp_server"
    assert (d["cpu_quota"], d["memory_max"]) == ("25%", "256M")
    assert d["config_path"] == CONFIG_DIR
    assert f"MCP_SERVER_CONFIG={CONFIG_DIR}/config.yaml" in d["env"]
    assert d["environment_file"] == TOKEN_FILE
    # The token is never inlined into the unit's Environment= lines.
    assert not any("MCP_SERVER_TOKEN" in e for e in d["env"])


def test_mcp_server_node_entry_and_config() -> None:
    entry = node_entry("mcp_server")
    assert entry["node_type"] == "mcp_server"
    assert entry["present"] is True and entry["enabled"] is True
    cfg = node_config("mcp_server")
    assert set(cfg) <= {"server", "topics", "limits", "timeouts", "arm", "battery"}
    assert cfg["server"] == {"host": "0.0.0.0", "port": PORT, "path": "/mcp"}
    assert cfg["arm"]["home_file"] == f"{ARM_DIR}/home.yaml"
    assert cfg["arm"]["urdf_path"] == "nodes/web_ui/urdf/so101_arm.urdf"
    assert (REPO_ROOT / cfg["arm"]["urdf_path"]).is_file()


def test_mcp_server_topics_match_filter_node_lease() -> None:
    topics = node_config("mcp_server").get("topics", {})
    flt = node_config("filter_node")
    assert topics.get("autonomy_command", AUTONOMY["autonomy_input_topic"]) == flt["autonomy_input_topic"]
    assert topics.get("autonomy_release", AUTONOMY["autonomy_release_topic"]) == flt["autonomy_release_topic"]
    assert topics.get("active_source", AUTONOMY["active_source_topic"]) == flt["active_source_topic"]
    assert topics.get("follower_joint_states", "/follower/joint_states") == flt["follower_feedback_topic"]


def test_filter_node_autonomy_params() -> None:
    cfg = node_config("filter_node")
    for key, topic in AUTONOMY.items():
        assert cfg[key] == topic, key


def test_gripper_camera_enabled() -> None:
    entry = node_entry("gripper_uvc_camera")
    assert entry["present"] is True and entry["enabled"] is True


def test_gripper_camera_rotated_180_at_source() -> None:
    env = node_entry("gripper_uvc_camera")["env"]
    assert "UVC_ROTATE_DEG=180" in env
    assert sum(e.startswith("UVC_ROTATE_DEG=") for e in env) == 1


def test_mcp_server_arm_base_height_is_16_5_cm() -> None:
    assert node_config("mcp_server")["arm"]["arm_base_height_m"] == 0.165


def test_web_ui_tab_set() -> None:
    tabs = node_config("web_ui")["tabs"]
    assert [t["id"] for t in tabs] == WEB_UI_TABS
    assert not REMOVED_WEB_UI_TABS & {t["id"] for t in tabs}
    assert tabs[0]["type"] == "map_nav"
    assert tabs[1]["type"] == "agent_chat"
    camera = tabs[2]
    assert camera["type"] == "camera"
    assert camera["topic"] == "/camera_0/image_raw/compressed"
    assert tabs[3]["type"] == "rgbd_camera"
    assert tabs[4]["type"] == "imu_orientation"


def test_web_ui_map_tab_uses_contract_fields() -> None:
    """The map tab names the frontend contract fields and the arm Trigger services mcp_server actually serves."""
    tab = node_config("web_ui")["tabs"][0]
    assert not LEGACY_MAP_KEYS & tab.keys(), f"legacy map tab keys: {sorted(LEGACY_MAP_KEYS & tab.keys())}"
    assert tab["base_urdf"] == "robot.urdf"
    assert tab["arm_urdf"] == "so101_arm.urdf"
    assert tab["base_joint_states_topic"] == "/swerve_drive/joint_states"
    assert tab["arm_joint_states_topic"] == "/follower/joint_states"
    assert tab["local_costmap_topic"] == "/local_costmap/costmap"
    topics = node_config("mcp_server").get("topics", {})
    assert tab["arm_home_service"] == topics.get("home_service", "/arm/home") == "/arm/home"
    assert tab["arm_set_home_service"] == topics.get("set_home_service", "/arm/set_home") == "/arm/set_home"
    assert tab.get("tile_cache_dir", TILE_CACHE_DIR) == TILE_CACHE_DIR


def test_web_ui_tile_cache_dir_task_owned_by_node_user() -> None:
    tasks = yaml.safe_load(TILE_CACHE_TASKS.read_text())
    file_task = next(t for t in tasks if "ansible.builtin.file" in t)
    args = file_task["ansible.builtin.file"]
    paths = file_task.get("loop", [args["path"]])
    assert TILE_CACHE_DIR in paths and "/var/cache/web_ui" in paths
    assert args["state"] == "directory"
    assert args["owner"] == "{{ ansible_user }}" and args["group"] == "{{ ansible_user }}"


@pytest.mark.parametrize("playbook", ["deploy_nodes_client.yml", "nodes/client/web_ui.yml"])
def test_playbooks_create_tile_cache_before_deploying_web_ui(playbook: str) -> None:
    _, tasks, _ = play_tasks(PLAYBOOKS_DIR / playbook)
    deploy = deploy_index(tasks, "web_ui")
    setup = include_index(tasks, "web_ui_tile_cache_dir.yml")
    assert deploy >= 0, f"{playbook} does not deploy web_ui"
    assert 0 <= setup < deploy, f"{playbook}: the tile cache dir must exist before web_ui starts"


def test_mcp_server_setup_tasks_create_token_and_arm_dir() -> None:
    tasks = yaml.safe_load(SETUP_TASKS.read_text())
    dirs = {t["ansible.builtin.file"]["path"]: t["ansible.builtin.file"] for t in tasks if "ansible.builtin.file" in t}
    assert dirs[CONFIG_DIR]["state"] == "directory"
    arm = dirs[ARM_DIR]
    assert arm["state"] == "directory"
    assert arm["owner"] == "{{ ansible_user }}" and arm["group"] == "{{ ansible_user }}"
    token_task = next(t for t in tasks if t.get("ansible.builtin.copy", {}).get("dest") == TOKEN_FILE)
    copy = token_task["ansible.builtin.copy"]
    assert copy["force"] is False, "an existing token must never be overwritten"
    assert copy["mode"] == "0640", "group-readable so claude_agent (group mcp-token) can use it; never world-readable"
    assert copy["owner"] == "{{ ansible_user }}" and copy["group"] == "mcp-token"
    assert token_task["no_log"] is True
    content = copy["content"]
    assert content.startswith("MCP_SERVER_TOKEN=")
    assert "lookup('ansible.builtin.password', '/dev/null'" in content
    assert "length=48" in content


@pytest.mark.parametrize("playbook", ["deploy_nodes_client.yml", "nodes/client/mcp_server.yml"])
def test_playbooks_run_setup_before_deploying_mcp_server(playbook: str) -> None:
    pre, tasks, _ = play_tasks(PLAYBOOKS_DIR / playbook)
    deploy = deploy_index(tasks, "mcp_server")
    setup = include_index(tasks, "mcp_server_setup.yml")
    assert deploy >= 0, f"{playbook} does not deploy mcp_server"
    assert 0 <= setup < deploy, f"{playbook}: token/arm dir setup must run before the deploy"


def test_node_playbook_stops_first_and_starts_last() -> None:
    pre, tasks, post = play_tasks(NODE_PLAYBOOK)
    assert "stop_ros_nodes.yml" in pre[0]["ansible.builtin.include_tasks"]
    assert include_index(pre, "repo_sync.yml") > 0
    assert "start_ros_nodes.yml" in post[-1]["ansible.builtin.include_tasks"]
    assert yaml.safe_load(NODE_PLAYBOOK.read_text())[0]["hosts"] == "client"


def test_unit_template_renders_environment_file_only_when_set() -> None:
    jinja2 = pytest.importorskip("jinja2")
    template = jinja2.Template(SERVICE_TEMPLATE.read_text())
    base = {"node_name": "mcp_server", "ansible_user": "ubuntu", "node_env": ["MCP_SERVER_CONFIG=x"]}
    with_file = template.render(**base, node_environment_file=TOKEN_FILE)
    assert f"EnvironmentFile={TOKEN_FILE}\n" in with_file
    assert with_file.index("EnvironmentFile=") < with_file.index("ExecStart=")
    assert "EnvironmentFile" not in template.render(**base)
    assert "EnvironmentFile" not in template.render(**base, node_environment_file="")


def test_resolve_and_deploy_passes_environment_file() -> None:
    tasks = yaml.safe_load(RESOLVE_TASKS.read_text())
    role_vars = next(t for t in tasks if "ansible.builtin.include_role" in t)["vars"]
    assert "environment_file" in role_vars["node_environment_file"]
    assert "default('')" in role_vars["node_environment_file"]


def test_token_file_never_in_repo() -> None:
    tracked = subprocess.run(
        ["git", "ls-files"], cwd=REPO_ROOT, capture_output=True, text=True, check=True
    ).stdout.splitlines()
    assert not [p for p in tracked if Path(p).name == "token" or p.endswith(".token")], "token files tracked in git"
    literal = re.compile(r"(MCP_SERVER_TOKEN|ROBOT_MCP_TOKEN)=['\"]?[A-Za-z0-9]{16,}")
    for rel in tracked:
        path = REPO_ROOT / rel
        if rel.startswith(("ansible/", "scripts/", "nodes/mcp_server/", "tests/")) or rel == ".mcp.json":
            if path.is_file() and path.suffix in {".yml", ".yaml", ".sh", ".json", ".py", ".md", ".j2", ""}:
                assert not literal.search(path.read_text(errors="ignore")), f"literal token in {rel}"
    assert "${ROBOT_MCP_TOKEN}" in MCP_JSON.read_text()


def test_mcp_json_registers_robot_server() -> None:
    doc = json.loads(MCP_JSON.read_text())
    robot = doc["mcpServers"]["robot"]
    assert robot["type"] == "http"
    assert robot["url"] == MCP_URL
    assert robot["headers"] == {"Authorization": "Bearer ${ROBOT_MCP_TOKEN}"}
    assert str(PORT) in robot["url"] and node_config("mcp_server")["server"]["path"] in robot["url"]


def test_robot_mcp_token_script() -> None:
    assert TOKEN_SCRIPT.is_file() and os.access(TOKEN_SCRIPT, os.X_OK)
    text = TOKEN_SCRIPT.read_text()
    assert text.startswith("#!/usr/bin/env bash")
    assert "set -euo pipefail" in text
    assert "ssh" in text and TOKEN_FILE in text
    assert "export ROBOT_MCP_TOKEN=" in text
    result = subprocess.run(["bash", "-n", str(TOKEN_SCRIPT)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_robot_mcp_token_script_prints_export_line(tmp_path: Path) -> None:
    fake_ssh = tmp_path / "ssh"
    fake_ssh.write_text("#!/usr/bin/env bash\necho 'MCP_SERVER_TOKEN=abc123XYZ'\n")
    fake_ssh.chmod(0o755)
    env = os.environ | {"PATH": f"{tmp_path}:{os.environ['PATH']}"}
    result = subprocess.run(["bash", str(TOKEN_SCRIPT)], capture_output=True, text=True, env=env)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "export ROBOT_MCP_TOKEN='abc123XYZ'"


def test_mcp_server_node_package_layout() -> None:
    pyproject = (NODE_DIR / "pyproject.toml").read_text()
    assert 'mcp = "' in pyproject and 'packages = [{ include = "mcp_server"' in pyproject
    assert (NODE_DIR / "poetry.lock").is_file()
    assert (NODE_DIR / "mcp_server" / "__main__.py").is_file()
    assert (NODE_DIR / "README.md").read_text().count("claude mcp add") >= 1


def test_mcp_server_listed_in_nodes_readme_index() -> None:
    """nodes/README.md Layout index must carry an entry for the mcp_server node."""
    layout = NODES_README.read_text().split("## Layout", 1)[1]
    assert re.search(r"^- \*\*mcp_server/\*\* .*MCP", layout, re.MULTILINE)


def test_every_test_is_documented_in_tests_readme() -> None:
    """tests/README.md must list every test in this file."""
    section = TESTS_README.read_text().split("### test_mcp_server_config.py", 1)[1].split("\n### ", 1)[0]
    tree = ast.parse(Path(__file__).read_text())
    names = [n.name for n in tree.body if isinstance(n, ast.FunctionDef) and n.name.startswith("test_")]
    missing = [name for name in names if f"`{name}`" not in section]
    assert not missing, f"undocumented in tests/README.md: {missing}"
