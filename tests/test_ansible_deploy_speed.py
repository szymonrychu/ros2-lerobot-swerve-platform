"""Static and script-level checks of the deploy speed work: stamps instead of always-run builds, queued restarts,
batched apt, tags on every task, a single tag-filtered playbook run per deploy."""

import json
import os
import re
import subprocess
from pathlib import Path
from typing import Any

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ANSIBLE_DIR = REPO_ROOT / "ansible"
ROLE_DIR = ANSIBLE_DIR / "roles" / "ros2_node_deploy"
VERIFY_DIR = ANSIBLE_DIR / "roles" / "ros2_node_verify"
PLAYBOOKS_DIR = ANSIBLE_DIR / "playbooks"
TASKS_DIR = PLAYBOOKS_DIR / "tasks"
DEPLOY_SCRIPT = REPO_ROOT / "scripts" / "deploy-nodes.sh"
ANSIBLE_README = ANSIBLE_DIR / "README.md"
DEPLOY_SKILL = REPO_ROOT / ".claude" / "skills" / "ansible-deploy" / "SKILL.md"
TARGETS = ("client", "server")
PHASE_TAGS = {"apt", "python", "build", "config", "boot", "setup", "sync", "restart", "verify", "always"}
NODE_PHASES = {"apt", "python", "build", "config"}
SETUP_NODE = {
    "web_ui_tile_cache_dir.yml": "web_ui",
    "slam_maps_dir.yml": "slam_toolbox",
    "mcp_server_setup.yml": "mcp_server",
    "poi_store_dir.yml": "poi_store",
    "claude_agent_setup.yml": "claude_agent",
    "overview_camera_boot_config.yml": "overview_camera",
}


def load(path: Path) -> Any:
    """Parse a YAML file.

    Args:
        path: YAML file.

    Returns:
        Any: Parsed document.
    """
    return yaml.safe_load(path.read_text())


def node_names(target: str) -> list[str]:
    """Names of the ros2_nodes of a target.

    Args:
        target: client or server.

    Returns:
        list[str]: Node names in deploy order.
    """
    return [n["name"] for n in load(ANSIBLE_DIR / "group_vars" / f"{target}.yml")["ros2_nodes"]]


def effective_tasks(tasks: list[dict], inherited: tuple[str, ...] = ()) -> list[tuple[dict, set[str]]]:
    """Flatten tasks (descending into blocks) with the tags each one effectively carries.

    Args:
        tasks: Parsed task list.
        inherited: Tags of the enclosing blocks.

    Returns:
        list[tuple[dict, set[str]]]: (task, own plus inherited tags).
    """
    out: list[tuple[dict, set[str]]] = []
    for task in tasks:
        tags = set(inherited) | set(task.get("tags", []))
        for key in ("block", "rescue", "always"):
            if key in task:
                out += effective_tasks(task[key], tuple(tags))
        if "block" not in task:
            out.append((task, tags))
    return out


def documented_tags() -> set[str]:
    """Tags listed in the "Deploy tags" table of ansible/README.md.

    Returns:
        set[str]: The documented phase tags.
    """
    section = ANSIBLE_README.read_text().split("## Deploy tags", 1)[1].split("\n## ", 1)[0]
    return set(re.findall(r"^\| `([a-z_]+)` \|", section, re.M))


def all_task_files() -> list[Path]:
    """Every task file whose tasks must be tagged.

    Returns:
        list[Path]: Role task files, the verify role and playbooks/tasks/*.yml.
    """
    return sorted((ROLE_DIR / "tasks").glob("*.yml")) + sorted(VERIFY_DIR.glob("tasks/*.yml")) + sorted(TASKS_DIR.glob("*.yml"))


def test_ansible_cfg_enables_timing_pipelining_and_connection_reuse() -> None:
    cfg = (ANSIBLE_DIR / "ansible.cfg").read_text()
    assert "ansible.posix.profile_tasks" in cfg and "ansible.posix.timer" in cfg
    assert re.search(r"^pipelining = True", cfg, re.M)
    assert "ControlMaster=auto" in cfg and "ControlPersist=" in cfg
    assert "ControlPath=none" not in cfg and "ControlMaster=no" not in cfg
    assert "ansible.posix" in (ANSIBLE_DIR / "requirements.yml").read_text()


def test_ansible_cfg_gathers_minimal_facts_and_caches_them() -> None:
    cfg = (ANSIBLE_DIR / "ansible.cfg").read_text()
    assert re.search(r"^gather_subset = min", cfg, re.M)
    assert re.search(r"^gathering = smart", cfg, re.M)
    assert re.search(r"^fact_caching = jsonfile", cfg, re.M)
    assert ".ansible_facts_cache" in (REPO_ROOT / ".gitignore").read_text()


def test_documented_tag_set_lists_every_phase_tag() -> None:
    assert documented_tags() == PHASE_TAGS


@pytest.mark.parametrize("path", all_task_files(), ids=lambda p: f"{p.parent.parent.name}/{p.name}")
def test_every_task_in_role_and_task_files_carries_a_documented_tag(path: Path) -> None:
    """Without a tag a task is skipped by every --tags filter, so a partial run would silently miss it."""
    allowed = documented_tags()
    for task, tags in effective_tasks(load(path)):
        assert tags & allowed, f"{path.name}: task {task.get('name')!r} has no documented tag ({sorted(tags)})"


@pytest.mark.parametrize("target", TARGETS)
def test_every_task_in_the_deploy_playbooks_carries_a_documented_or_node_tag(target: str) -> None:
    play = load(PLAYBOOKS_DIR / f"deploy_nodes_{target}.yml")[0]
    allowed = documented_tags() | set(node_names(target))
    for section in ("pre_tasks", "tasks", "post_tasks"):
        for task, tags in effective_tasks(play.get(section, [])):
            assert tags & allowed, f"{target} {section}: {task.get('name')!r} has no tag"


@pytest.mark.parametrize("target", TARGETS)
def test_every_node_has_a_tagged_deploy_step(target: str) -> None:
    """--tags <node> must reach the node: its deploy include carries the node name and every phase tag, and applies the
    node tag to the included tasks."""
    tasks = load(PLAYBOOKS_DIR / f"deploy_nodes_{target}.yml")[0]["tasks"]
    deploys = {
        t["vars"]["_deploy_node_name"]: t
        for t in tasks
        if "resolve_and_deploy.yml" in str(t.get("ansible.builtin.include_tasks", ""))
    }
    assert set(deploys) == set(node_names(target))
    for name, task in deploys.items():
        assert {name} | NODE_PHASES <= set(task["tags"]), name
        assert task["ansible.builtin.include_tasks"]["apply"]["tags"] == [name], name
    assert list(deploys) != [] and len(deploys) == len(node_names(target))


def test_node_specific_setup_files_carry_their_node_tag() -> None:
    client = load(PLAYBOOKS_DIR / "deploy_nodes_client.yml")[0]
    includes = [t for sec in ("pre_tasks", "tasks") for t in client[sec]]
    for filename, node in SETUP_NODE.items():
        task = next(t for t in includes if filename in str(t.get("ansible.builtin.include_tasks", "")))
        assert node in task["tags"], f"{filename} must carry the {node} tag"
        if filename != "overview_camera_boot_config.yml":
            assert task["ansible.builtin.include_tasks"]["apply"]["tags"] == [node]


@pytest.mark.parametrize("target", TARGETS)
def test_shared_steps_run_under_any_node_filter(target: str) -> None:
    """Repo sync, apt, restart and verify are tagged always so a --tags <node> run still syncs, installs, restarts
    the queued nodes and verifies; the apt steps decide by ansible_run_tags whether they take part."""
    play = load(PLAYBOOKS_DIR / f"deploy_nodes_{target}.yml")[0]
    by_file = {
        Path(str(next(iter(v for k, v in t.items() if k.startswith("ansible.builtin.") and isinstance(v, str)), ""))).name: t
        for t in play["pre_tasks"] + play["post_tasks"]
    }
    for filename in ("select_run.yml", "repo_sync.yml", "ros_packages_sync.yml", "apt_nodes.yml", "start_ros_nodes.yml"):
        assert "always" in by_file[filename]["tags"], filename
    assert "always" in next(t for t in play["tasks"] if t.get("ansible.builtin.include_role"))["tags"]
    assert "when" in by_file["apt_nodes.yml"] and "ros2_run_apt" in by_file["apt_nodes.yml"]["when"]


def run_deploy_script(args: list[str], tmp_path: Path) -> subprocess.CompletedProcess[str]:
    """Run scripts/deploy-nodes.sh with a fake ansible-playbook that prints its arguments.

    Args:
        args: Script arguments.
        tmp_path: Directory for the fake binary.

    Returns:
        subprocess.CompletedProcess[str]: The finished process.
    """
    fake = tmp_path / "ansible-playbook"
    fake.write_text('#!/bin/sh\necho "ansible-playbook $*"\n')
    fake.chmod(0o755)
    env = {**os.environ, "PATH": f"{tmp_path}:{os.environ['PATH']}"}
    return subprocess.run([str(DEPLOY_SCRIPT), *args], capture_output=True, text=True, env=env, check=False)


def test_deploy_script_runs_one_playbook_with_the_joined_node_tags(tmp_path: Path) -> None:
    result = run_deploy_script(["client", "web_ui", "mcp_server"], tmp_path)
    assert result.returncode == 0, result.stderr
    calls = [line for line in result.stdout.splitlines() if line.startswith("ansible-playbook")]
    assert calls == ["ansible-playbook -i inventory playbooks/deploy_nodes_client.yml -l client --tags web_ui,mcp_server"]


def test_deploy_script_all_passes_tags_and_other_options_through(tmp_path: Path) -> None:
    result = run_deploy_script(["server", "--all", "--tags", "config,restart", "--skip-tags", "verify"], tmp_path)
    assert result.returncode == 0, result.stderr
    assert "playbooks/deploy_nodes_server.yml -l server --tags config,restart --skip-tags verify" in result.stdout
    bare = run_deploy_script(["client", "--all"], tmp_path)
    assert "--tags" not in bare.stdout.split("ansible-playbook", 1)[1]


def test_deploy_script_rejects_unknown_nodes_and_node_list_with_tags(tmp_path: Path) -> None:
    unknown = run_deploy_script(["client", "no_such_node"], tmp_path)
    assert unknown.returncode != 0 and "no_such_node" in unknown.stderr and "web_ui" in unknown.stderr
    mixed = run_deploy_script(["client", "web_ui", "--tags", "config"], tmp_path)
    assert mixed.returncode != 0 and "--tags cannot be combined" in mixed.stderr
    assert not (PLAYBOOKS_DIR / "nodes").exists(), "per-node playbooks are replaced by tags"


def role_main_text() -> str:
    """Text of the role's main task file.

    Returns:
        str: File content.
    """
    return (ROLE_DIR / "tasks" / "main.yml").read_text()


def test_role_has_no_always_changed_tasks_and_does_not_restart_or_start_nodes() -> None:
    for path in (ROLE_DIR / "tasks").glob("*.yml"):
        assert "changed_when: true" not in path.read_text(), path.name
    text = role_main_text()
    assert "state: restarted" not in text and "state: started" not in text and "'started'" not in text
    handlers = load(ROLE_DIR / "handlers" / "main.yml")
    restart = next(h for h in handlers if h["name"] == "Restart ROS2 node")
    assert "ansible.builtin.set_fact" in restart, "the handler only queues the restart"
    assert "ros2_nodes_pending_restart" in str(restart["ansible.builtin.set_fact"])


def test_poetry_install_runs_only_for_a_new_dependency_hash_and_stamps_after_success() -> None:
    tasks = {t.get("name"): t for t, _ in effective_tasks(load(ROLE_DIR / "tasks" / "main.yml"))}
    poetry = tasks["Install node Python dependencies via Poetry"]
    assert "node_deps_stale | bool" in poetry["when"]
    cmd = poetry["ansible.builtin.shell"]
    assert cmd.index("poetry install --only main") < cmd.index(".poetry-deps"), "stamp only after a successful install"
    assert "changed_when" not in poetry
    probe = tasks["Detect node source and Python dependency changes"]["ansible.builtin.shell"]
    for needle in ("poetry.lock", "pyproject.toml", "shared/pyproject.toml", "rev-parse HEAD:shared", '"HEAD:$src"'):
        assert needle in probe, needle
    source_stamp = tasks["Record the deployed node source (restart when it changed)"]
    assert source_stamp["notify"] == "Restart ROS2 node" and "node_src_key" in source_stamp["ansible.builtin.copy"]["content"]


def test_web_ui_npm_steps_run_only_when_their_inputs_changed() -> None:
    tasks = {t.get("name"): t for t, _ in effective_tasks(load(ROLE_DIR / "tasks" / "main.yml"))}
    ci = tasks["Run npm ci for web_ui frontend"]
    build = tasks["Run npm run build for web_ui frontend"]
    assert any("probe.stdout | from_json).ci" in c for c in ci["when"])
    assert any("probe.stdout | from_json).build" in c for c in build["when"])
    assert build["notify"] == "Restart ROS2 node"
    copy = tasks["Copy built web_ui frontend to static dir"]
    assert copy["when"] == "web_ui_npm_build is changed"
    probe = tasks["Fingerprint the web_ui frontend"]["ansible.builtin.shell"]
    for needle in ("package-lock.json", "rev-parse HEAD:nodes/web_ui/frontend", "node_modules", "dist/index.html", "static/index.html"):
        assert needle in probe, needle
    # stamps are written after the build and the copy succeeded
    names = list(tasks)
    assert names.index("Copy built web_ui frontend to static dir") < names.index("Stamp the built web_ui frontend sources")


def render_probe(script: str, **values: str) -> str:
    """Fill the Jinja variables of a probe script.

    Args:
        script: Shell text with {{ var }} placeholders.
        **values: Variable values.

    Returns:
        str: The script ready for bash.
    """
    for key, value in values.items():
        script = script.replace("{{ " + key + " }}", value)
    return script


def git(repo: Path, *args: str) -> None:
    """Run git in a repo with a fixed identity.

    Args:
        repo: Repository directory.
        *args: git arguments.
    """
    subprocess.run(
        ["git", "-c", "user.name=t", "-c", "user.email=t@t", *args], cwd=repo, check=True, capture_output=True
    )


@pytest.fixture
def probe_repo(tmp_path: Path) -> Path:
    """A repo shaped like the deployed checkout: a node using shared/ and the web_ui frontend.

    Args:
        tmp_path: pytest temp dir.

    Returns:
        Path: Repository root.
    """
    repo = tmp_path / "repo"
    files = {
        "nodes/demo/pyproject.toml": 'ros2-common = { path = "../../shared", develop = true }\n',
        "nodes/demo/poetry.lock": "lock-1\n",
        "nodes/demo/demo/__init__.py": "x = 1\n",
        "shared/pyproject.toml": "[project]\n",
        "shared/lib.py": "y = 1\n",
        "nodes/web_ui/frontend/package.json": "{}\n",
        "nodes/web_ui/frontend/package-lock.json": "{}\n",
        "nodes/web_ui/frontend/src/App.tsx": "a\n",
        "nodes/web_ui/web_ui/__init__.py": "",
    }
    for rel, text in files.items():
        (repo / rel).parent.mkdir(parents=True, exist_ok=True)
        (repo / rel).write_text(text)
    git(repo, "init", "-q")
    git(repo, "add", "-A")
    git(repo, "commit", "-qm", "init")
    return repo


def run_bash(script: str) -> dict:
    """Run a probe script and parse its JSON line.

    Args:
        script: Rendered shell script.

    Returns:
        dict: The probe result.
    """
    out = subprocess.run(["bash", "-c", script], capture_output=True, text=True, check=True)
    return json.loads(out.stdout.strip().splitlines()[-1])


def test_dependency_probe_detects_source_dependency_and_shared_changes(probe_repo: Path, tmp_path: Path) -> None:
    tasks = {t.get("name"): t for t, _ in effective_tasks(load(ROLE_DIR / "tasks" / "main.yml"))}
    template = tasks["Detect node source and Python dependency changes"]["ansible.builtin.shell"]
    venv = tmp_path / "opt"
    script = render_probe(template, repo_dest=str(probe_repo), node_src_dir="nodes/demo", node_name="demo")
    script = script.replace("/opt/ros2-nodes/demo", str(venv))
    first = run_bash(script)
    assert first["stored_deps_key"] == "" and first["deps_key"], "a new venv needs an install"
    (venv / "venv").mkdir(parents=True)
    (venv / "venv" / ".poetry-deps").write_text(first["deps_key"])
    assert run_bash(script)["stored_deps_key"] == first["deps_key"], "stamp matches: no reinstall"
    (probe_repo / "nodes/demo/demo/__init__.py").write_text("x = 2\n")
    git(probe_repo, "commit", "-qam", "src")
    changed_src = run_bash(script)
    assert changed_src["src_key"] != first["src_key"] and changed_src["deps_key"] == first["deps_key"]
    (probe_repo / "shared/lib.py").write_text("y = 2\n")
    git(probe_repo, "commit", "-qam", "shared source")
    shared = run_bash(script)
    assert shared["src_key"] != changed_src["src_key"], "shared/ changes restart its dependents"
    assert shared["deps_key"] == first["deps_key"], "develop mode: shared source needs no reinstall"
    (probe_repo / "shared/pyproject.toml").write_text("[project]\nversion = 2\n")
    assert run_bash(script)["deps_key"] != first["deps_key"], "shared metadata changes reinstall"
    (probe_repo / "nodes/demo/poetry.lock").write_text("lock-2\n")
    assert run_bash(script)["deps_key"] != first["deps_key"]


def test_web_ui_probe_builds_only_when_frontend_inputs_change(probe_repo: Path) -> None:
    tasks = {t.get("name"): t for t, _ in effective_tasks(load(ROLE_DIR / "tasks" / "main.yml"))}
    script = render_probe(tasks["Fingerprint the web_ui frontend"]["ansible.builtin.shell"], repo_dest=str(probe_repo))
    frontend = probe_repo / "nodes/web_ui/frontend"
    fresh = run_bash(script)
    assert fresh["ci"] is True and fresh["build"] is True
    (frontend / "node_modules").mkdir()
    (frontend / "dist").mkdir()
    (frontend / "dist/index.html").write_text("<html>")
    (probe_repo / "nodes/web_ui/web_ui/static").mkdir()
    (probe_repo / "nodes/web_ui/web_ui/static/index.html").write_text("<html>")
    (frontend / ".npm-ci-stamp").write_text(fresh["lock"])
    (frontend / ".build-stamp").write_text(fresh["tree"])
    assert run_bash(script)["ci"] is False and run_bash(script)["build"] is False, "nothing changed: no npm"
    (frontend / "src/App.tsx").write_text("b\n")
    git(probe_repo, "commit", "-qam", "ui")
    changed = run_bash(script)
    assert changed["ci"] is False and changed["build"] is True, "source change builds without npm ci"
    (frontend / "package-lock.json").write_text('{"v": 2}\n')
    assert run_bash(script)["ci"] is True
    (probe_repo / "nodes/web_ui/web_ui/static/index.html").unlink()
    (frontend / ".npm-ci-stamp").write_text(run_bash(script)["lock"])
    assert run_bash(script)["build"] is True, "missing static output rebuilds"


def test_heavy_steps_stop_the_nodes_only_when_they_will_run() -> None:
    main = load(ROLE_DIR / "tasks" / "main.yml")
    stops = [t for t, _ in effective_tasks(main) if "stop_for_build.yml" in str(t.get("ansible.builtin.include_tasks", ""))]
    assert len(stops) == 2 and all("when" in t for t in stops)
    colcon = load(ROLE_DIR / "tasks" / "colcon_source_package.yml")
    colcon_stop = next(t for t in colcon if "stop_for_build.yml" in str(t.get("ansible.builtin.include_tasks", "")))
    assert colcon_stop["when"] == "not _colcon_stamp_stat.stat.exists"
    for text in (PLAYBOOKS_DIR / "deploy_nodes_client.yml", PLAYBOOKS_DIR / "deploy_nodes_server.yml"):
        assert "stop_for_build" not in text.read_text() and "stop_ros_nodes" not in text.read_text()
    stop_tasks = load(ROLE_DIR / "tasks" / "stop_for_build.yml")
    assert "ros2_nodes_stopped_for_build" in stop_tasks[0]["when"], "stopped once per play"
    assert stop_tasks[1]["ansible.builtin.set_fact"]["ros2_nodes_stopped_for_build"] is True


def test_colcon_clone_patch_and_build_are_skipped_when_the_stamp_exists() -> None:
    tasks = load(ROLE_DIR / "tasks" / "colcon_source_package.yml")
    by_module = {next(k for k in t if k.startswith("ansible.builtin.")): t for t in tasks}
    for module in ("ansible.builtin.git", "ansible.builtin.patch", "ansible.builtin.command"):
        assert by_module[module]["when"] == "not _colcon_stamp_stat.stat.exists", module
    names = [t["name"] for t in tasks]
    assert names.index("Look for the build stamp") < names.index("Clone the pinned source commit")
    assert names.index("Drop the build stamp when the installed workspace is missing") < names.index("Look for the build stamp")


def test_start_ros_nodes_restarts_queued_and_starts_stopped_nodes_in_order_sleeping_only_after_acting() -> None:
    tasks = load(TASKS_DIR / "start_ros_nodes.yml")
    script = next(t for t in tasks if "ansible.builtin.shell" in t)["ansible.builtin.shell"]
    assert "ros2_nodes_pending_restart" in script and "continue" in script
    assert script.index("continue") < script.index("systemctl restart") < script.index("sleep {{ ros2_node_start_interval_s")
    assert "for n in {{ ros2_nodes_to_start | join(' ') }}" in script
    assert "namespace" in str(tasks[0]), "names keep ros2_nodes order"
    assert "ros2_nodes_pending_restart: []" in (TASKS_DIR / "start_ros_nodes.yml").read_text()


def test_claude_agent_token_change_queues_a_restart_directly() -> None:
    tasks = load(TASKS_DIR / "claude_agent_setup.yml")
    queue = next(t for t in tasks if "ros2_nodes_pending_restart" in str(t.get("ansible.builtin.set_fact", "")))
    assert queue["when"] == "claude_agent_token_written is changed"
    assert "notify" not in (TASKS_DIR / "claude_agent_setup.yml").read_text().replace("No restart notify", "")


@pytest.mark.parametrize("target", TARGETS)
def test_apt_packages_are_installed_in_one_batched_task(target: str) -> None:
    play = load(PLAYBOOKS_DIR / f"deploy_nodes_{target}.yml")[0]
    assert play["vars"]["ros2_apt_batched"] is True
    assert any("apt_nodes.yml" in str(t.get("ansible.builtin.include_tasks", "")) for t in play["pre_tasks"])
    batched = load(TASKS_DIR / "apt_nodes.yml")
    apt = next(t for t in batched if "ansible.builtin.apt" in t)["ansible.builtin.apt"]
    assert apt["name"] == "{{ ros2_apt_packages }}" and apt["cache_valid_time"] == 3600
    role_apt = next(t for t, _ in effective_tasks(load(ROLE_DIR / "tasks" / "main.yml")) if "ansible.builtin.apt" in t)
    assert any("ros2_apt_batched" in c for c in role_apt["when"])


def test_verify_role_checks_all_units_in_one_command_per_round() -> None:
    tasks = load(VERIFY_DIR / "tasks" / "main.yml")
    checks = [t for t in tasks if "systemctl is-active" in str(t.get("ansible.builtin.command", ""))]
    assert len(checks) == 2
    for check in checks:
        assert "loop" not in check and "_ros2_verify_units | join(' ')" in check["ansible.builtin.command"]
