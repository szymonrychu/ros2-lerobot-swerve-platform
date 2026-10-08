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
ROUND2_SENSOR_TOOLS = {
    "get_body_state", "pixel_to_ground", "get_annotated_camera_image", "mark_candidate_points", "resolve_candidate",
    "capture_calibration_sample", "solve_camera_calibration", "clear_calibration_samples", "get_topdown_view",
    "remember_object", "list_objects", "forget_object", "list_pois", "add_poi", "update_poi", "delete_poi",
}


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


def flat(tasks: list[dict]) -> list[dict]:
    """Expand the deploy playbook's single block into its tasks.

    Args:
        tasks: A play's task list.

    Returns:
        list[dict]: The tasks, with block children in place of the block.
    """
    return [c for t in tasks for c in (t["block"] if "block" in t else [t])]


def play_tasks(path: Path) -> tuple[list[dict], list[dict], list[dict]]:
    """Pre-tasks, tasks and post-tasks of the first play in a playbook.

    Args:
        path: Playbook path.

    Returns:
        tuple[list[dict], list[dict], list[dict]]: (pre_tasks, tasks, post_tasks).
    """
    play = yaml.safe_load(path.read_text())[0]
    return play.get("pre_tasks", []), flat(play.get("tasks", [])), play.get("post_tasks", [])


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
    """Names of the tools registered by the mcp_server modules (functions decorated with @tool / @server.tool).

    Returns:
        set[str]: Tool function names.
    """
    names = set()
    for module in MCP_TOOLS.parent.glob("*.py"):
        for node in ast.walk(ast.parse(module.read_text())):
            if isinstance(node, ast.FunctionDef):
                for deco in node.decorator_list:
                    target = deco.func if isinstance(deco, ast.Call) else deco
                    if (isinstance(target, ast.Name) and target.id == "tool") or (
                        isinstance(target, ast.Attribute) and target.attr == "tool"
                    ):
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
    assert names.index("claude_agent") == names.index("poi_store") + 1 == names.index("mcp_server") + 2
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
    unknown = classified - tools
    assert not unknown, f"tools unknown to mcp_server: {sorted(unknown)}"
    assert "stop" in cfg.uncapped_tools and "stop" not in cfg.effector_tools


def mcp_server_motion_tools() -> set[str]:
    """Read MOTION_TOOLS from nodes/mcp_server/mcp_server/tools.py without importing the node.

    Returns:
        set[str]: Tool names in the MOTION_TOOLS set literal (optionally wrapped in frozenset(...)).
    """
    tree = ast.parse((REPO_ROOT / "nodes" / "mcp_server" / "mcp_server" / "tools.py").read_text())
    for node in ast.walk(tree):
        target = node.targets[0] if isinstance(node, ast.Assign) else getattr(node, "target", None)
        if isinstance(target, ast.Name) and target.id == "MOTION_TOOLS" and node.value is not None:
            value = node.value
            if isinstance(value, ast.Call) and value.args:
                value = value.args[0]
            return set(ast.literal_eval(value))
    raise AssertionError("mcp_server tools.py defines no MOTION_TOOLS")


def test_effector_tools_match_mcp_server_motion_tools() -> None:
    """claude_agent effector_tools equal mcp_server MOTION_TOOLS exactly (look_around included)."""
    cfg = yaml.safe_load(node_entry("claude_agent")["config"])
    motion = mcp_server_motion_tools()
    assert "look_around" in motion
    assert set(cfg["effector_tools"]) == motion


def test_round2_tools_classified_in_group_vars_and_defaults() -> None:
    """look_around is an effector, every other round-1/2 tool a sensor, in the client.yml config and the code defaults."""
    spec = importlib.util.spec_from_file_location("claude_agent_config", NODE_DIR / "claude_agent" / "config.py")
    pytest.importorskip("pydantic")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    raw = yaml.safe_load(node_entry("claude_agent")["config"])
    for effectors, sensors in (
        (raw["effector_tools"], raw["sensor_tools"]),
        (module.DEFAULT_EFFECTOR_TOOLS, module.DEFAULT_SENSOR_TOOLS),
    ):
        assert "look_around" in effectors and "look_around" not in sensors
        assert ROUND2_SENSOR_TOOLS <= set(sensors)
        assert not ROUND2_SENSOR_TOOLS & set(effectors)


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


def test_playbook_level_task_files_notify_no_role_handlers() -> None:
    """Task files under playbooks/tasks/ are included at playbook level, where the ros2_node_deploy role's handlers
    are not visible ("The requested handler 'Restart ROS2 node' was not found"). Deploys stop every node first and
    start them last, so these files never need to restart a service."""
    offenders = [
        f"{path.name}: {task.get('name')}"
        for path in sorted((PLAYBOOKS_DIR / "tasks").glob("*.yml"))
        for task in yaml.safe_load(path.read_text()) or []
        if isinstance(task, dict) and "notify" in task
    ]
    assert offenders == []


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


def test_setup_tasks_write_env_file_0600_no_log() -> None:
    tasks = setup_tasks()
    write = next(t for t in tasks if t.get("ansible.builtin.copy", {}).get("dest") == ENV_FILE)
    copy = write["ansible.builtin.copy"]
    assert copy["mode"] == "0600" and copy["owner"] == SERVICE_USER
    assert copy["content"].startswith(f"{TOKEN_VAR}=")
    assert write["no_log"] is True
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


def test_existing_mcp_token_gets_group_read_permissions() -> None:
    """The create task uses force: false, which leaves an existing token untouched (owner-only 0600 from before the
    mcp-token group existed); a file task after it must enforce group mcp-token and 0640 on that file."""
    tasks = yaml.safe_load(MCP_SETUP_TASKS.read_text())
    create = next(i for i, t in enumerate(tasks) if t.get("ansible.builtin.copy", {}).get("dest") == MCP_TOKEN_FILE)
    enforce = [
        (i, t["ansible.builtin.file"])
        for i, t in enumerate(tasks)
        if t.get("ansible.builtin.file", {}).get("path") == MCP_TOKEN_FILE
    ]
    assert enforce, "no task enforces permissions on an existing token"
    index, spec = enforce[0]
    assert index > create
    assert spec["group"] == MCP_TOKEN_GROUP and spec["mode"] == "0640"


@pytest.mark.parametrize("playbook", ["deploy_nodes_client.yml"])
def test_playbooks_run_setup_before_deploying_claude_agent_after_mcp_server(playbook: str) -> None:
    _, tasks, _ = play_tasks(PLAYBOOKS_DIR / playbook)
    deploy = index_of(tasks, "resolve_and_deploy.yml", "claude_agent")
    setup = index_of(tasks, "claude_agent_setup.yml")
    mcp_setup = index_of(tasks, "mcp_server_setup.yml")
    assert deploy >= 0, f"{playbook} does not deploy claude_agent"
    assert 0 <= mcp_setup < setup < deploy, f"{playbook}: mcp token group and agent token must be set up first"
    if playbook == "deploy_nodes_client.yml":
        assert index_of(tasks, "resolve_and_deploy.yml", "mcp_server") < deploy


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
    for needle in ("/api/state", "/api/message", "/api/stop", "/api/reset", "/ws/events", "set_task_plan", "complete_phase", "revise_plan", "raise_phase_budget", "max_rw_cap", "max_phase_rw_cap", "/robot_events"):
        assert needle in text, needle
    assert f"export {TOKEN_VAR}=" in text and "./scripts/deploy-nodes.sh client claude_agent" in text


def test_docs_and_lint_scripts_list_claude_agent() -> None:
    assert re.search(r"^- \*\*claude_agent/\*\*", NODES_README.read_text(), re.M)
    assert "claude_agent" in ANSIBLE_README.read_text() and TOKEN_VAR in ANSIBLE_README.read_text()
    assert "`claude_agent`" in DEPLOY_SKILL.read_text()
    assert "nodes/claude_agent" in LINT_SCRIPT.read_text()
    assert "nodes/claude_agent" in ROOT_PYPROJECT.read_text()


def test_every_test_is_documented_in_tests_readme() -> None:
    """tests/README.md must list every test in this file."""
    section = TESTS_README.read_text().split("### test_claude_agent_config.py", 1)[1].split("\n### ", 1)[0]
    tree = ast.parse(Path(__file__).read_text())
    names = [n.name for n in tree.body if isinstance(n, ast.FunctionDef) and n.name.startswith("test_")]
    missing = [name for name in names if f"`{name}`" not in section]
    assert not missing, f"undocumented in tests/README.md: {missing}"


HARDENING_LINES = ("ProtectProc=invisible", "ProcSubset=pid", "NoNewPrivileges=yes", "PrivateTmp=yes")
HARDENING_VARS = ("node_protect_proc", "node_proc_subset", "node_no_new_privileges", "node_private_tmp")


def test_claude_agent_hardening_in_group_vars_without_filesystem_protection() -> None:
    d = client_vars()["ros2_node_type_defaults"]["claude_agent"]
    assert d["protect_proc"] == "invisible" and d["proc_subset"] == "pid"
    assert d["no_new_privileges"] is True and d["private_tmp"] is True
    assert "protect_system" not in d and "protect_home" not in d


def test_only_claude_agent_sets_hardening_in_group_vars() -> None:
    keys = {"protect_proc", "proc_subset", "no_new_privileges", "private_tmp"}
    for group_vars in (CLIENT_VARS, CLIENT_VARS.with_name("server.yml")):
        for name, defaults in yaml.safe_load(group_vars.read_text()).get("ros2_node_type_defaults", {}).items():
            if name != "claude_agent":
                assert not keys & set(defaults), f"{name} must keep the default unit"


def test_resolve_and_deploy_passes_hardening_with_empty_defaults() -> None:
    tasks = yaml.safe_load(RESOLVE_TASKS.read_text())
    role_vars = next(t for t in tasks if "ansible.builtin.include_role" in t)["vars"]
    for var, key in zip(HARDENING_VARS, ("protect_proc", "proc_subset", "no_new_privileges", "private_tmp")):
        assert f".{key}" in role_vars[var] and "default(" in role_vars[var]


def test_role_defaults_keep_hardening_off() -> None:
    defaults = yaml.safe_load((ROLE_DIR / "defaults" / "main.yml").read_text())
    for var in HARDENING_VARS:
        assert var in defaults and defaults[var] in ("", False)


def test_service_template_hardening_is_optional() -> None:
    jinja2 = pytest.importorskip("jinja2")
    template = jinja2.Template(SERVICE_TEMPLATE.read_text())
    base = {"node_name": "x", "ansible_user": "ubuntu"}
    default = template.render(**base)
    for needle in ("ProtectProc", "ProcSubset", "NoNewPrivileges", "PrivateTmp", "ProtectSystem", "ProtectHome"):
        assert needle not in default
    hardened = template.render(
        **base, node_protect_proc="invisible", node_proc_subset="pid", node_no_new_privileges=True, node_private_tmp=True
    )
    for line in HARDENING_LINES:
        assert f"{line}\n" in hardened
    assert "ProtectSystem" not in hardened and "ProtectHome" not in hardened
    off = template.render(
        **base, node_protect_proc="", node_proc_subset="", node_no_new_privileges=False, node_private_tmp=False
    )
    assert off == default


def test_claude_agent_config_has_watchdog_and_sdk_initialize_timeout() -> None:
    cfg = yaml.safe_load(node_entry("claude_agent")["config"])
    assert cfg["instruction_timeout_s"] == 900
    env = client_vars()["ros2_node_type_defaults"]["claude_agent"]["env"]
    assert "CLAUDE_CODE_STREAM_CLOSE_TIMEOUT=180000" in env


WORKDIR = "/var/lib/claude_agent/workspace"
STATE_DIR = "/var/lib/claude_agent"


def test_setup_tasks_create_persistent_workdir_owned_0750() -> None:
    tasks = setup_tasks()
    dirs = {t["ansible.builtin.file"]["path"]: t["ansible.builtin.file"] for t in tasks if "ansible.builtin.file" in t}
    spec = dirs[WORKDIR]
    assert spec["state"] == "directory"
    assert spec["owner"] == SERVICE_USER and spec["group"] == SERVICE_USER and spec["mode"] == "0750"
    names = [t["ansible.builtin.file"]["path"] for t in tasks if "ansible.builtin.file" in t]
    assert names.index(STATE_DIR) < names.index(WORKDIR), "HOME must exist before the workdir inside it"
    assert "persistent" in SETUP_TASKS.read_text().lower()


def test_ansible_never_removes_the_workdir_or_state_dir() -> None:
    """The workdir is the agent's persistent volume (NOTES.md): no task may delete it, recurse-reset it or purge it."""
    offenders = []
    for path in sorted(ANSIBLE_DIR.rglob("*.yml")):
        text = path.read_text()
        if "claude_agent" not in text and "/var/lib" not in text:
            continue
        for task in yaml.safe_load(text) or []:
            if not isinstance(task, dict):
                continue
            for module in ("ansible.builtin.file", "ansible.builtin.command", "ansible.builtin.shell"):
                body = task.get(module)
                rendered = str(body)
                if body and "/var/lib/claude_agent" in rendered and ("absent" in rendered or "rm " in rendered):
                    offenders.append(f"{path.name}: {task.get('name')}")
    assert offenders == []


def test_claude_agent_config_workdir_state_dir_and_hardware_facts() -> None:
    raw = yaml.safe_load(node_entry("claude_agent")["config"])
    assert raw["workdir"] == WORKDIR and "work_dir" not in raw
    assert raw["state_dir"] == STATE_DIR
    assert raw["arm_base_height_m"] == 0.165
    assert "arm_reach_cm" in raw
    home = [e for e in client_vars()["ros2_node_type_defaults"]["claude_agent"]["env"] if e.startswith("HOME=")]
    assert home == [f"HOME={STATE_DIR}"], "HOME stays separate from (and above) the workdir"
    assert raw["workdir"] != home[0].split("=", 1)[1]


def test_claude_agent_config_has_budget_maxima_and_robot_events_topic() -> None:
    """The agent picks its own rw/turn budget under hard maxima (sensor calls are uncapped); the static caps are gone; events come from /robot_events."""
    raw = yaml.safe_load(node_entry("claude_agent")["config"])
    assert (raw["max_rw_cap"], raw["max_turn_cap"]) == (150, 200)
    assert (raw["max_phase_rw_cap"], raw["max_phase_turn_cap"]) == (40, 40)
    assert "max_ro_cap" not in raw and "max_phase_ro_cap" not in raw
    assert "effector_call_cap" not in raw and "max_turns" not in raw
    assert raw["robot_events_topic"] == "/robot_events"
