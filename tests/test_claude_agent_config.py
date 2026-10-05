"""Static invariants of the claude_agent wiring (Ansible, unit template, token handling, docs, package layout).

The claude_agent node (nodes/claude_agent) runs Claude through the Agent SDK against the robot MCP server. It runs as the
dedicated non-root user `claude_agent`; its OAuth token travels from the deploying machine's CLAUDE_CODE_OAUTH_TOKEN
into a 0600 EnvironmentFile and never enters git, logs or the unit.
"""

import ast
import importlib.util
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
SETUP_TASKS = PLAYBOOKS_DIR / "tasks" / "claude_agent_setup.yml"
MCP_SETUP_TASKS = PLAYBOOKS_DIR / "tasks" / "mcp_server_setup.yml"
NODE_PLAYBOOK = PLAYBOOKS_DIR / "nodes" / "client" / "claude_agent.yml"
NODE_DIR = REPO_ROOT / "nodes" / "claude_agent"
MCP_TOOLS = REPO_ROOT / "nodes" / "mcp_server" / "mcp_server" / "tools.py"
TESTS_README = REPO_ROOT / "tests" / "README.md"
NODES_README = REPO_ROOT / "nodes" / "README.md"
ANSIBLE_README = ANSIBLE_DIR / "README.md"
DEPLOY_SKILL = REPO_ROOT / ".claude" / "skills" / "ansible-deploy" / "SKILL.md"
LINT_SCRIPT = REPO_ROOT / "scripts" / "lint-all-nodes.sh"
ROOT_PYPROJECT = REPO_ROOT / "pyproject.toml"

ENV_FILE = "/etc/ros2/claude_agent/env"
TOKEN_VAR = "CLAUDE_CODE_OAUTH_TOKEN"
MCP_TOKEN_FILE = "/etc/ros2/mcp_server/token"
MCP_TOKEN_GROUP = "mcp-token"
SERVICE_USER = "claude_agent"
API_PORT = 18300
MCP_PORT = 18200


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


def play_tasks(path: Path) -> tuple[list[dict], list[dict], list[dict]]:
    """Pre-tasks, tasks and post-tasks of the first play in a playbook.

    Args:
        path: Playbook path.

    Returns:
        tuple[list[dict], list[dict], list[dict]]: (pre_tasks, tasks, post_tasks).
    """
    play = yaml.safe_load(path.read_text())[0]
    return play.get("pre_tasks", []), play.get("tasks", []), play.get("post_tasks", [])


def index_of(tasks: list[dict], filename: str, node: str | None = None) -> int:
    """Index of the include_tasks entry for a task file (optionally the deploy of one node).

    Args:
        tasks: Task list.
        filename: Included file name.
        node: When set, the include must deploy this `_deploy_node_name`.

    Returns:
        int: Task index (-1 when absent).
    """
    for i, task in enumerate(tasks):
        if filename in str(task.get("ansible.builtin.include_tasks", "")) and (
            node is None or task.get("vars", {}).get("_deploy_node_name") == node
        ):
            return i
    return -1


def setup_tasks() -> list[dict]:
    """Parse claude_agent_setup.yml.

    Returns:
        list[dict]: Its tasks.
    """
    return yaml.safe_load(SETUP_TASKS.read_text())


def mcp_tool_names() -> set[str]:
    """Names of the tools registered in mcp_server/tools.py (functions decorated with @tool).

    Returns:
        set[str]: Tool function names.
    """
    tree = ast.parse(MCP_TOOLS.read_text())
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            for deco in node.decorator_list:
                target = deco.func if isinstance(deco, ast.Call) else deco
                if isinstance(target, ast.Name) and target.id == "tool":
                    names.add(node.name)
    return names


def test_claude_agent_node_type_defaults() -> None:
    d = client_vars()["ros2_node_type_defaults"]["claude_agent"]
    assert d["deploy_mode"] == "native"
    assert d["node_src_dir"] == "nodes/claude_agent"
    assert d["node_launch_command"] == "python3 -m claude_agent"
    assert (d["cpu_quota"], d["memory_max"]) == ("50%", "1G")
    assert d["nice"] >= 5
    assert d["config_path"] == "/etc/ros2/claude_agent"
    assert d["user"] == SERVICE_USER
    assert MCP_TOKEN_GROUP in d["supplementary_groups"]
    assert d["environment_file"] == ENV_FILE
    assert "CLAUDE_AGENT_CONFIG=/etc/ros2/claude_agent/config.yaml" in d["env"]
    assert "DISABLE_AUTOUPDATER=1" in d["env"]
    assert not [e for e in d["env"] if TOKEN_VAR in e or "ANTHROPIC_API_KEY" in e], "secrets never go in Environment="


def test_claude_agent_entry_after_mcp_server_and_enabled() -> None:
    names = [n["name"] for n in client_vars()["ros2_nodes"]]
    assert names.index("claude_agent") == names.index("mcp_server") + 1
    entry = node_entry("claude_agent")
    assert entry["node_type"] == "claude_agent" and entry["present"] is True and entry["enabled"] is True


def test_claude_agent_config_valid_and_consistent_with_mcp_server() -> None:
    pytest.importorskip("pydantic")
    spec = importlib.util.spec_from_file_location("claude_agent_config", NODE_DIR / "claude_agent" / "config.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    raw = yaml.safe_load(node_entry("claude_agent")["config"])
    cfg = module.ClaudeAgentConfig.model_validate(raw)
    assert cfg.http_host == "127.0.0.1", "the agent API must stay on loopback"
    assert cfg.http_port == API_PORT
    assert cfg.mcp_url == f"http://127.0.0.1:{MCP_PORT}/mcp"
    assert cfg.mcp_token_file == MCP_TOKEN_FILE
    assert cfg.model == "opus"
    tools = mcp_tool_names()
    classified = set(cfg.effector_tools) | set(cfg.uncapped_tools) | set(cfg.sensor_tools)
    assert classified <= tools, f"tools unknown to mcp_server: {sorted(classified - tools)}"
    assert "stop" in cfg.uncapped_tools and "stop" not in cfg.effector_tools


def test_effector_tools_match_mcp_server_motion_tools_when_defined() -> None:
    pytest.importorskip("pydantic")
    source = "\n".join(p.read_text() for p in (REPO_ROOT / "nodes" / "mcp_server" / "mcp_server").glob("*.py"))
    match = re.search(r"^MOTION_TOOLS\b[^=]*=\s*(.+?)$", source, re.M)
    if not match:
        pytest.skip("mcp_server defines no MOTION_TOOLS yet")
    try:
        motion = set(ast.literal_eval(match.group(1).strip()))
    except (ValueError, SyntaxError):
        pytest.skip("MOTION_TOOLS is not a plain literal")
    cfg = yaml.safe_load(node_entry("claude_agent")["config"])
    assert set(cfg["effector_tools"]) == motion


def test_service_template_user_groups_and_nice_are_optional() -> None:
    jinja2 = pytest.importorskip("jinja2")
    template = jinja2.Template(SERVICE_TEMPLATE.read_text())
    base = {"node_name": "x", "ansible_user": "ubuntu"}
    default = template.render(**base)
    assert "User=ubuntu\n" in default
    assert "SupplementaryGroups" not in default and "Nice=" not in default
    custom = template.render(**base, node_user=SERVICE_USER, node_supplementary_groups=[MCP_TOKEN_GROUP], node_nice=10)
    assert f"User={SERVICE_USER}\n" in custom
    assert f"SupplementaryGroups={MCP_TOKEN_GROUP}\n" in custom
    assert "Nice=10\n" in custom
    empty = template.render(**base, node_user="", node_supplementary_groups=[], node_nice="")
    assert "User=ubuntu\n" in empty and "SupplementaryGroups" not in empty and "Nice=" not in empty


def test_resolve_and_deploy_passes_user_groups_and_nice() -> None:
    tasks = yaml.safe_load(RESOLVE_TASKS.read_text())
    role_vars = next(t for t in tasks if "ansible.builtin.include_role" in t)["vars"]
    assert ".user" in role_vars["node_user"] and "default('')" in role_vars["node_user"]
    assert ".supplementary_groups" in role_vars["node_supplementary_groups"]
    assert ".nice" in role_vars["node_nice"]


def test_setup_tasks_read_token_from_controller_env_and_fail_clearly() -> None:
    tasks = setup_tasks()
    text = SETUP_TASKS.read_text()
    assert f"lookup('env', '{TOKEN_VAR}')" in text
    fail = next(t for t in tasks if "ansible.builtin.fail" in t)
    assert f"export {TOKEN_VAR}=" in fail["ansible.builtin.fail"]["msg"]
    assert "deploy-nodes.sh client claude_agent" in fail["ansible.builtin.fail"]["msg"]
    assert "no_log" not in fail, "no_log would censor the failure message"
    assert "lookup(" not in str(fail), "the fail task must not touch the token"
    assert tasks.index(fail) < next(i for i, t in enumerate(tasks) if "ansible.builtin.copy" in t)


def test_setup_tasks_write_env_file_0600_no_log_and_restart() -> None:
    tasks = setup_tasks()
    write = next(t for t in tasks if t.get("ansible.builtin.copy", {}).get("dest") == ENV_FILE)
    copy = write["ansible.builtin.copy"]
    assert copy["mode"] == "0600" and copy["owner"] == SERVICE_USER
    assert copy["content"].startswith(f"{TOKEN_VAR}=")
    assert write["no_log"] is True
    assert "Restart ROS2 node" in str(write["notify"])
    assert "length" in str(write["when"]), "an empty env var must keep the existing file"


def test_setup_tasks_no_log_on_every_task_touching_the_token() -> None:
    touching = [t for t in setup_tasks() if "lookup(" in str(t) or TOKEN_VAR in str(t.get("ansible.builtin.copy", ""))]
    assert len(touching) >= 2
    for task in touching:
        assert task.get("no_log") is True, f"token task without no_log: {task.get('name')}"


def test_setup_tasks_create_user_and_dirs_and_keep_existing_file() -> None:
    tasks = setup_tasks()
    user = next(t["ansible.builtin.user"] for t in tasks if "ansible.builtin.user" in t)
    assert user["name"] == SERVICE_USER and user["system"] is True
    assert user["shell"] == "/usr/sbin/nologin" and user["home"] == "/var/lib/claude_agent"
    dirs = {t["ansible.builtin.file"]["path"]: t["ansible.builtin.file"] for t in tasks if "ansible.builtin.file" in t}
    assert dirs["/etc/ros2/claude_agent"]["state"] == "directory"
    assert dirs["/var/lib/claude_agent"]["owner"] == SERVICE_USER
    assert any(t.get("ansible.builtin.stat", {}).get("path") == ENV_FILE for t in tasks)


def test_mcp_token_readable_by_group_not_world() -> None:
    tasks = yaml.safe_load(MCP_SETUP_TASKS.read_text())
    group = next(t["ansible.builtin.group"] for t in tasks if "ansible.builtin.group" in t)
    assert group["name"] == MCP_TOKEN_GROUP and group["system"] is True
    assert tasks.index(next(t for t in tasks if "ansible.builtin.group" in t)) < next(
        i for i, t in enumerate(tasks) if t.get("ansible.builtin.copy", {}).get("dest") == MCP_TOKEN_FILE
    )
    copy = next(t["ansible.builtin.copy"] for t in tasks if t.get("ansible.builtin.copy", {}).get("dest") == MCP_TOKEN_FILE)
    assert copy["group"] == MCP_TOKEN_GROUP and copy["mode"] == "0640"


@pytest.mark.parametrize("playbook", ["deploy_nodes_client.yml", "nodes/client/claude_agent.yml"])
def test_playbooks_run_setup_before_deploying_claude_agent_after_mcp_server(playbook: str) -> None:
    _, tasks, _ = play_tasks(PLAYBOOKS_DIR / playbook)
    deploy = index_of(tasks, "resolve_and_deploy.yml", "claude_agent")
    setup = index_of(tasks, "claude_agent_setup.yml")
    mcp_setup = index_of(tasks, "mcp_server_setup.yml")
    assert deploy >= 0, f"{playbook} does not deploy claude_agent"
    assert 0 <= mcp_setup < setup < deploy, f"{playbook}: mcp token group and agent token must be set up first"
    if playbook == "deploy_nodes_client.yml":
        assert index_of(tasks, "resolve_and_deploy.yml", "mcp_server") < deploy


def test_node_playbook_stops_first_and_starts_last() -> None:
    pre, _, post = play_tasks(NODE_PLAYBOOK)
    assert "stop_ros_nodes.yml" in pre[0]["ansible.builtin.include_tasks"]
    assert "start_ros_nodes.yml" in post[-1]["ansible.builtin.include_tasks"]
    assert yaml.safe_load(NODE_PLAYBOOK.read_text())[0]["hosts"] == "client"


def test_oauth_token_never_in_repo() -> None:
    tracked = subprocess.run(
        ["git", "ls-files"], cwd=REPO_ROOT, capture_output=True, text=True, check=True
    ).stdout.splitlines()
    literal = re.compile(rf"{TOKEN_VAR}=['\"]?(sk-ant-|[A-Za-z0-9_-]{{30,}})")
    for rel in tracked:
        path = REPO_ROOT / rel
        if path.is_file() and path.suffix in {".yml", ".yaml", ".sh", ".py", ".md", ".j2", ".toml"}:
            assert not literal.search(path.read_text(errors="ignore")), f"literal OAuth token in {rel}"


def test_claude_agent_package_layout_and_pinned_sdk() -> None:
    pyproject = (NODE_DIR / "pyproject.toml").read_text()
    assert re.search(r'^claude-agent-sdk = "\d+\.\d+\.\d+"$', pyproject, re.M), "SDK (and bundled CLI) must be pinned"
    for dep in ("fastapi", "uvicorn", "pillow", "pydantic"):
        assert re.search(rf"^{dep} = ", pyproject, re.M), dep
    assert (NODE_DIR / "poetry.lock").is_file() and (NODE_DIR / "README.md").is_file()
    assert (NODE_DIR / "claude_agent" / "__main__.py").is_file() and (NODE_DIR / "tests").is_dir()


def test_claude_agent_readme_documents_api_and_token_deploy() -> None:
    text = (NODE_DIR / "README.md").read_text()
    for needle in ("/api/state", "/api/message", "/api/stop", "/api/reset", "/ws/events", "effector_call_cap"):
        assert needle in text, needle
    assert f"export {TOKEN_VAR}=" in text and "./scripts/deploy-nodes.sh client claude_agent" in text


def test_docs_and_lint_scripts_list_claude_agent() -> None:
    assert re.search(r"^- \*\*claude_agent/\*\*", NODES_README.read_text(), re.M)
    assert "claude_agent" in ANSIBLE_README.read_text() and TOKEN_VAR in ANSIBLE_README.read_text()
    assert "`claude_agent`" in DEPLOY_SKILL.read_text()
    assert "nodes/claude_agent" in LINT_SCRIPT.read_text()
    assert "nodes/claude_agent" in ROOT_PYPROJECT.read_text()


def test_deploy_script_discovers_node_playbook() -> None:
    assert NODE_PLAYBOOK.is_file()
    assert NODE_PLAYBOOK.parent == PLAYBOOKS_DIR / "nodes" / "client"


def test_every_test_is_documented_in_tests_readme() -> None:
    """tests/README.md must list every test in this file."""
    section = TESTS_README.read_text().split("### test_claude_agent_config.py", 1)[1].split("\n### ", 1)[0]
    tree = ast.parse(Path(__file__).read_text())
    names = [n.name for n in tree.body if isinstance(n, ast.FunctionDef) and n.name.startswith("test_")]
    missing = [name for name in names if f"`{name}`" not in section]
    assert not missing, f"undocumented in tests/README.md: {missing}"
