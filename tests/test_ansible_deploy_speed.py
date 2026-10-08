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


def flat(tasks: list[dict]) -> list[dict]:
    """Expand the deploy playbook's single block into its tasks.

    Args:
        tasks: A play's task list.

    Returns:
        list[dict]: The tasks, with block children in place of the block.
    """
    return [c for t in tasks for c in (t["block"] if "block" in t else [t])]


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
    return (
        sorted((ROLE_DIR / "tasks").glob("*.yml"))
        + sorted(VERIFY_DIR.glob("tasks/*.yml"))
        + sorted(TASKS_DIR.glob("*.yml"))
    )


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
    tasks = flat(load(PLAYBOOKS_DIR / f"deploy_nodes_{target}.yml")[0]["tasks"])
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
    includes = client["pre_tasks"] + flat(client["tasks"])
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
        Path(
            str(next(iter(v for k, v in t.items() if k.startswith("ansible.builtin.") and isinstance(v, str)), ""))
        ).name: t
        for t in play["pre_tasks"] + play["post_tasks"]
    }
    for filename in (
        "select_run.yml",
        "repo_sync.yml",
        "ros_packages_sync.yml",
        "apt_nodes.yml",
        "start_ros_nodes.yml",
    ):
        assert "always" in by_file[filename]["tags"], filename
    assert "always" in next(t for t in play["post_tasks"] if t.get("ansible.builtin.include_role"))["tags"]
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
    assert calls == [
        "ansible-playbook -i inventory playbooks/deploy_nodes_client.yml -l client --tags web_ui,mcp_server"
    ]


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
    assert "ansible.builtin.lineinfile" in restart, "the handler only queues the restart"
    queue = restart["ansible.builtin.lineinfile"]
    assert queue["path"] == "{{ ros2_restart_queue }}" and queue["line"] == "{{ ros2_node_restart_now }}"
    assert queue["create"] is True and restart["become"] is True


def test_uv_sync_runs_only_for_a_new_dependency_hash_and_stamps_after_success() -> None:
    tasks = {t.get("name"): t for t, _ in effective_tasks(load(ROLE_DIR / "tasks" / "main.yml"))}
    venv = tasks["Create node venv with system-site-packages"]["ansible.builtin.command"]
    assert venv["cmd"] == "python3 -m venv --system-site-packages /opt/ros2-nodes/{{ node_name }}/venv"
    sync = tasks["Install node Python dependencies with uv"]
    assert "node_deps_stale | bool" in sync["when"]
    cmd = sync["ansible.builtin.shell"]
    assert cmd.index("/usr/local/bin/uv sync --frozen --no-dev") < cmd.index(".uv-deps"), "stamp only after success"
    assert sync["args"]["chdir"] == "{{ repo_dest }}/{{ node_src_dir }}", "path dependencies resolve in the checkout"
    assert sync["environment"] == {
        "UV_PROJECT_ENVIRONMENT": "/opt/ros2-nodes/{{ node_name }}/venv",
        "UV_PYTHON": "/opt/ros2-nodes/{{ node_name }}/venv/bin/python3",
        "UV_PYTHON_DOWNLOADS": "never",
    }
    assert "changed_when" not in sync and sync["notify"] == "Restart ROS2 node"
    probe = tasks["Detect node source and Python dependency changes"]["ansible.builtin.shell"]
    for needle in ("uv.lock", "pyproject.toml", "shared/pyproject.toml", '"HEAD:$p"', "node_src_paths", ".uv-deps"):
        assert needle in probe, needle
    assert "grep" not in probe, "shared/ is declared per node type (src_shared), not guessed from pyproject"
    queue = tasks["Queue a restart (Python dependencies installed)"]
    assert "node_uv_sync is changed" in queue["when"] and sync["register"] == "node_uv_sync"
    source_stamp = tasks["Record the deployed node source (restart when it changed)"]
    assert (
        source_stamp["notify"] == "Restart ROS2 node"
        and "node_src_key" in source_stamp["ansible.builtin.copy"]["content"]
    )
    assert "when" not in source_stamp, "launch-only nodes (no node_src_dir) get a source stamp too"


def uv_install_task() -> dict:
    """The ros2_base task that installs the pinned uv.

    Returns:
        dict: The parsed task.
    """
    tasks = {t["name"]: t for t in load(ANSIBLE_DIR / "roles" / "ros2_base" / "tasks" / "main.yml")}
    return tasks["Install the pinned uv via pipx (system-wide)"]


def test_ros2_base_installs_the_pinned_uv_system_wide() -> None:
    assert load(ANSIBLE_DIR / "roles" / "ros2_base" / "defaults" / "main.yml")["ros2_uv_version"] == "0.11.29"
    task = uv_install_task()
    assert task["environment"] == {"PIPX_HOME": "/opt/pipx", "PIPX_BIN_DIR": "/usr/local/bin"}
    assert task["become"] is True and task["register"] == "_uv_install"
    assert task["changed_when"] == "'already installed' not in _uv_install.stdout"
    assert 'pipx install --force "uv=={{ ros2_uv_version }}"' in task["ansible.builtin.shell"]


@pytest.mark.parametrize(
    ("installed", "installs"),
    [
        ("uv 0.11.29", False),
        ("uv 0.11.29 (901092ee1 2026-07-15 aarch64-unknown-linux-gnu)", False),
        ("uv 0.11.2", True),
        ("uv 0.11.290", True),
        ("uv 0.12.0", True),
        (None, True),
    ],
)
def test_ros2_base_uv_install_is_idempotent_and_fixes_a_wrong_version(
    tmp_path: Path, installed: str | None, installs: bool
) -> None:
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    calls = tmp_path / "pipx-calls"
    (bin_dir / "pipx").write_text(f'#!/bin/bash\necho "$@" >> {calls}\n')
    fake_uv = bin_dir / "uv"
    if installed is not None:
        fake_uv.write_text(f'#!/bin/bash\necho "{installed}"\n')
    for f in bin_dir.iterdir():
        f.chmod(0o755)
    script = render_probe(uv_install_task()["ansible.builtin.shell"], ros2_uv_version="0.11.29")
    script = script.replace("/usr/local/bin/uv", str(fake_uv))
    env = {**os.environ, "PATH": f"{bin_dir}:/usr/bin:/bin"}
    out = subprocess.run(["bash", "-c", script], capture_output=True, text=True, check=True, env=env)
    if installs:
        assert calls.read_text() == "install --force uv==0.11.29\n"
        assert "already installed" not in out.stdout
    else:
        assert not calls.exists() and "already installed" in out.stdout


def test_ansible_tree_has_no_poetry_left_outside_the_readme() -> None:
    hits = [
        str(p.relative_to(REPO_ROOT))
        for p in ANSIBLE_DIR.rglob("*")
        if p.is_file() and p != ANSIBLE_README and "poetry" in p.read_text(errors="ignore").lower()
    ]
    assert hits == []


def test_lint_ansible_script_runs_ansible_lint_through_uv() -> None:
    text = (REPO_ROOT / "scripts" / "lint-ansible.sh").read_text()
    assert "poetry" not in text.lower()
    assert "uv run ansible-lint" in text and "uvx --from" in text


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
    for needle in (
        "package-lock.json",
        "rev-parse HEAD:nodes/web_ui/frontend",
        "node_modules",
        "dist/index.html",
        "static/index.html",
    ):
        assert needle in probe, needle
    # stamps are written after the build and the copy succeeded
    names = list(tasks)
    assert names.index("Copy built web_ui frontend to static dir") < names.index(
        "Stamp the built web_ui frontend sources"
    )


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
    assert "{{" not in script, script
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
        "nodes/demo/pyproject.toml": '[tool.uv.sources]\nros2-common = { path = "../../shared", editable = true }\n',
        "nodes/demo/uv.lock": "lock-1\n",
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
    script = render_probe(
        template,
        repo_dest=str(probe_repo),
        node_name="demo",
        **{
            "node_src_dir | default('')": "nodes/demo",
            "node_src_paths | join(' ')": "nodes/demo shared",
            "node_src_shared | default(false) | bool | lower": "true",
        },
    )
    script = script.replace("/opt/ros2-nodes/demo", str(venv))
    first = run_bash(script)
    assert first["stored_deps_key"] == "" and first["deps_key"], "a new venv needs an install"
    (venv / "venv").mkdir(parents=True)
    (venv / "venv" / ".poetry-deps").write_text(first["deps_key"])
    assert run_bash(script)["stored_deps_key"] == "", "a stamp left by the old Poetry install never skips uv sync"
    (venv / "venv" / ".uv-deps").write_text(first["deps_key"])
    assert run_bash(script)["stored_deps_key"] == first["deps_key"], "stamp matches: no reinstall"
    (probe_repo / "nodes/demo/demo/__init__.py").write_text("x = 2\n")
    git(probe_repo, "commit", "-qam", "src")
    changed_src = run_bash(script)
    assert changed_src["src_key"] != first["src_key"] and changed_src["deps_key"] == first["deps_key"]
    (probe_repo / "shared/lib.py").write_text("y = 2\n")
    git(probe_repo, "commit", "-qam", "shared source")
    shared = run_bash(script)
    assert shared["src_key"] != changed_src["src_key"], "shared/ changes restart its dependents"
    assert shared["deps_key"] == first["deps_key"], "editable install: shared source needs no reinstall"
    (probe_repo / "shared/pyproject.toml").write_text("[project]\nversion = 2\n")
    assert run_bash(script)["deps_key"] != first["deps_key"], "shared metadata changes reinstall"
    (probe_repo / "nodes/demo/uv.lock").write_text("lock-2\n")
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
    stops = [
        t for t, _ in effective_tasks(main) if "stop_for_build.yml" in str(t.get("ansible.builtin.include_tasks", ""))
    ]
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
    assert names.index("Drop the build stamp when the installed workspace is missing") < names.index(
        "Look for the build stamp"
    )


def render_start_script(tmp_path: Path, nodes: list[str], scope: list[str]) -> str:
    """Render the start_ros_nodes.yml shell script with a queue file and node lists.

    Args:
        tmp_path: Directory holding the queue file.
        nodes: Enabled node names in deploy order.
        scope: Nodes the run covers.

    Returns:
        str: Script ready for bash.
    """
    tasks = load(TASKS_DIR / "start_ros_nodes.yml")
    script = next(t for t in tasks if "ansible.builtin.shell" in t)["ansible.builtin.shell"]
    return render_probe(
        script,
        ros2_restart_queue=str(tmp_path / "pending-restart"),
        ros2_stopped_for_build_file=str(tmp_path / "stopped-for-build"),
        **{
            "ros2_scope_nodes | default(ros2_nodes_to_start) | join(' ')": " ".join(scope),
            "ros2_nodes_to_start | join(' ')": " ".join(nodes),
            "ros2_node_start_interval_s | default(5)": "0",
        },
    )


def run_start(tmp_path: Path, script: str, active: list[str], failing: str = "") -> subprocess.CompletedProcess[str]:
    """Run a rendered start script against a fake systemctl.

    Args:
        tmp_path: Work directory (fake systemctl, log, active list).
        script: Rendered script.
        active: Units reported active.
        failing: Unit whose restart/start fails.

    Returns:
        subprocess.CompletedProcess[str]: The finished process.
    """
    fake = tmp_path / "bin" / "systemctl"
    fake.parent.mkdir(exist_ok=True)
    fake.write_text(
        "#!/bin/bash\n"
        'if [ "$1" = is-active ]; then grep -qx -- "$3" "$ACTIVE"; exit $?; fi\n'
        '[ "$1" = daemon-reload ] && exit 0\n'
        '[ "$2" = "$FAILING" ] && exit 1\n'
        'echo "$1 $2" >> "$LOG"\n'
    )
    fake.chmod(0o755)
    (tmp_path / "active").write_text("".join(f"ros2-{n}\n" for n in active))
    env = {
        **os.environ,
        "PATH": f"{fake.parent}:{os.environ['PATH']}",
        "ACTIVE": str(tmp_path / "active"),
        "LOG": str(tmp_path / "log"),
        "FAILING": f"ros2-{failing}" if failing else "-",
    }
    return subprocess.run(["bash", "-c", script], capture_output=True, text=True, env=env, check=False)


def test_start_script_restarts_the_queue_starts_stopped_nodes_in_scope_and_clears_the_queue(tmp_path: Path) -> None:
    (tmp_path / "pending-restart").write_text("b\nz\n")
    script = render_start_script(tmp_path, ["a", "b", "c", "d"], scope=["a", "b", "c", "d"])
    result = run_start(tmp_path, script, active=["a", "b"])
    assert result.returncode == 0, result.stderr
    assert result.stdout.split("\n")[:-1] == ["restarted ros2-b", "started ros2-c", "started ros2-d"]
    assert (tmp_path / "log").read_text().split("\n")[:-1] == ["restart ros2-b", "start ros2-c", "start ros2-d"]
    assert (tmp_path / "pending-restart").read_text() == "", "queue cleared after success"


def test_start_script_with_a_node_filter_only_starts_selected_nodes_but_restarts_every_queued_one(
    tmp_path: Path,
) -> None:
    (tmp_path / "pending-restart").write_text("c\n")
    script = render_start_script(tmp_path, ["a", "b", "c"], scope=["a"])
    result = run_start(tmp_path, script, active=[])
    assert result.stdout.split("\n")[:-1] == ["started ros2-a", "restarted ros2-c"], "b is out of scope and not queued"


def test_start_script_fails_on_a_failed_restart_and_keeps_the_queue(tmp_path: Path) -> None:
    (tmp_path / "pending-restart").write_text("a\nb\n")
    script = render_start_script(tmp_path, ["a", "b"], scope=["a", "b"])
    result = run_start(tmp_path, script, active=["a", "b"], failing="a")
    assert result.returncode != 0
    assert (tmp_path / "pending-restart").read_text() == "a\nb\n", (
        "a failed run keeps its queued restarts for the next run"
    )


def test_start_script_without_a_queue_file_acts_only_on_stopped_nodes(tmp_path: Path) -> None:
    script = render_start_script(tmp_path, ["a", "b"], scope=["a", "b"])
    result = run_start(tmp_path, script, active=["a"])
    assert result.returncode == 0 and result.stdout.split("\n")[:-1] == ["started ros2-b"]


def test_start_ros_nodes_keeps_deploy_order_and_sleeps_only_after_acting() -> None:
    tasks = load(TASKS_DIR / "start_ros_nodes.yml")
    script = next(t for t in tasks if "ansible.builtin.shell" in t)["ansible.builtin.shell"]
    assert (
        script.index("continue")
        < script.index('systemctl "$verb"')
        < script.index("sleep {{ ros2_node_start_interval_s")
    )
    assert "set -euo pipefail" in script
    assert "namespace" in str(tasks[0]), "names keep ros2_nodes order"
    assert "ros2_restarted_nodes" in str(tasks[-1])


def test_restart_queue_is_a_host_file_written_by_the_handler_and_claude_agent_setup() -> None:
    all_vars = load(ANSIBLE_DIR / "group_vars" / "all.yml")
    assert all_vars["ros2_restart_queue"].startswith("{{ ros2_deploy_state_dir }}/")
    assert all_vars["ros2_deploy_state_dir"] == "/var/lib/ros2-deploy"
    tasks = load(TASKS_DIR / "claude_agent_setup.yml")
    queue = next(t for t in tasks if "ansible.builtin.lineinfile" in t)
    assert queue["ansible.builtin.lineinfile"]["path"] == "{{ ros2_restart_queue }}"
    assert queue["ansible.builtin.lineinfile"]["line"] == "claude_agent"
    assert queue["when"] == "claude_agent_token_written is changed"
    assert "ros2_nodes_pending_restart" not in "".join(p.read_text() for p in ANSIBLE_DIR.rglob("*.yml"))
    for target in TARGETS:
        pre = load(PLAYBOOKS_DIR / f"deploy_nodes_{target}.yml")[0]["pre_tasks"]
        mk = next(t for t in pre if "ansible.builtin.file" in t and "state" in t["ansible.builtin.file"])
        assert "{{ ros2_deploy_state_dir }}/stamps" in mk["loop"] and "always" in mk["tags"]


@pytest.mark.parametrize("target", TARGETS)
def test_deploy_tasks_run_in_a_block_whose_rescue_starts_nodes_and_still_fails_the_run(target: str) -> None:
    """A failure after a heavy step stopped the nodes must not leave them down (incident 2026-10-07)."""
    play = load(PLAYBOOKS_DIR / f"deploy_nodes_{target}.yml")[0]
    assert len(play["tasks"]) == 1 and "block" in play["tasks"][0]
    rescue = play["tasks"][0]["rescue"]
    start, unlock, fail = rescue
    assert start["ignore_errors"] is True, "a failing start must not mask the original failure"
    assert "start_ros_nodes.yml" in str(start["block"])
    assert (
        unlock["ansible.builtin.file"]["path"] == "{{ ros2_deploy_lock }}"
        and unlock["ansible.builtin.file"]["state"] == "absent"
    )
    msg = fail["ansible.builtin.fail"]["msg"]
    assert "ansible_failed_task" in msg and "ansible_failed_result.stderr" in msg and "ansible_failed_result.rc" in msg
    assert all("always" in t["tags"] for t in (start, unlock, fail))
    assert any("start_ros_nodes.yml" in str(t) for t in play["post_tasks"]), "success path starts nodes too"


def test_scope_facts_limit_start_and_verify_to_the_selected_nodes() -> None:
    select = (TASKS_DIR / "select_run.yml").read_text()
    assert "ros2_scope_nodes" in select and "ros2_selected_nodes | length == 0" in select
    verify = (VERIFY_DIR / "tasks" / "main.yml").read_text()
    assert "ros2_scope_nodes" in verify and "ros2_restarted_nodes" in verify


def node_type_paths(group: dict, entry: dict) -> list[str]:
    """Source paths a node resolves to, mirroring resolve_and_deploy.yml.

    Args:
        group: Parsed group_vars file.
        entry: A ros2_nodes entry.

    Returns:
        list[str]: Repo-relative paths hashed for the node's restart stamp.
    """
    node_type = group["ros2_node_type_defaults"][entry["node_type"]]
    base = node_type.get("src_paths", [node_type.get("node_src_dir", "")])
    return (
        [p for p in base if p]
        + node_type.get("src_extra_paths", [])
        + (["shared"] if node_type.get("src_shared") else [])
    )


@pytest.mark.parametrize("target", TARGETS)
def test_every_present_node_resolves_to_existing_source_paths(target: str) -> None:
    """Launch/config-only nodes (no node_src_dir) must restart when their repo files change: every node hashes paths."""
    group = load(ANSIBLE_DIR / "group_vars" / f"{target}.yml")
    for entry in group["ros2_nodes"]:
        if entry.get("present", True) is False:
            continue
        paths = node_type_paths(group, entry)
        if group["ros2_node_type_defaults"][entry["node_type"]].get("src_paths") == []:
            continue  # nothing in the repo to watch: constant key, restarts only for config/unit changes
        assert paths, f"{entry['name']} has no source paths"
        for path in paths:
            assert (REPO_ROOT / path).exists(), f"{entry['name']}: {path} is not in the repo"
    text = (TASKS_DIR / "resolve_and_deploy.yml").read_text()
    assert "node_src_paths:" in text and "src_extra_paths" in text and "src_shared" in text


def test_mcp_server_restarts_when_the_web_ui_urdf_it_loads_changes() -> None:
    client = load(ANSIBLE_DIR / "group_vars" / "client.yml")
    entry = next(n for n in client["ros2_nodes"] if n["name"] == "mcp_server")
    assert "nodes/web_ui/urdf" in node_type_paths(client, entry)
    assert (REPO_ROOT / "nodes" / "web_ui" / "urdf" / "so101_arm.urdf").exists()


def test_every_pyproject_depending_on_shared_is_declared_src_shared() -> None:
    client = load(ANSIBLE_DIR / "group_vars" / "client.yml")["ros2_node_type_defaults"]
    server = load(ANSIBLE_DIR / "group_vars" / "server.yml")["ros2_node_type_defaults"]
    declared = {t["node_src_dir"] for t in {**server, **client}.values() if t.get("src_shared")}
    found = {
        str(p.parent.relative_to(REPO_ROOT))
        for p in (REPO_ROOT / "nodes").rglob("pyproject.toml")
        if "node_modules" not in p.parts and "../../shared" in p.read_text()
    }
    assert found and found <= declared, f"declare src_shared for {sorted(found - declared)}"


def test_docs_say_build_and_config_filters_skip_apt() -> None:
    text = ANSIBLE_README.read_text()
    assert "`--tags build` and `--tags config` do not install apt packages" in text


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


@pytest.mark.parametrize("target", TARGETS)
def test_verify_runs_after_the_end_of_play_restarts(target: str) -> None:
    """Verify must see the restarted/started nodes: the start_ros_nodes include comes before the verify role."""
    play = load(PLAYBOOKS_DIR / f"deploy_nodes_{target}.yml")[0]
    flat_play = play["pre_tasks"] + flat(play["tasks"]) + play["post_tasks"]
    start = next(
        i for i, t in enumerate(flat_play) if "start_ros_nodes.yml" in str(t.get("ansible.builtin.include_tasks", ""))
    )
    verify = next(i for i, t in enumerate(flat_play) if "ansible.builtin.include_role" in t)
    assert start < verify and verify == len(flat_play) - 2, "verify is the last step, followed only by the lock release"
    assert flat_play[-1]["ansible.builtin.file"]["state"] == "absent"


def task_by_name(tasks: list[dict], name: str) -> dict:
    """Find a task by name in a flattened task list.

    Args:
        tasks: Parsed tasks.
        name: Task name.

    Returns:
        dict: The task.
    """
    return next(t for t in tasks if t.get("name") == name)


def test_every_restart_causing_task_is_followed_directly_by_a_queue_task() -> None:
    """The restart intent must reach the host file the moment a change is applied, not at the late handler flush
    (a failure in between lost the restart forever)."""
    for path in (ROLE_DIR / "tasks" / "main.yml", ROLE_DIR / "tasks" / "colcon_source_package.yml"):
        flat_tasks = [t for t, _ in effective_tasks(load(path))]
        causing = [
            i
            for i, t in enumerate(flat_tasks)
            if t.get("notify") == "Restart ROS2 node" or "Restart ROS2 node" in t.get("notify", [])
        ]
        assert causing, path.name
        for i in causing:
            task, nxt = flat_tasks[i], flat_tasks[i + 1]
            assert task.get("register"), f"{task['name']} needs a register"
            queue = nxt["ansible.builtin.lineinfile"]
            assert queue["path"] == "{{ ros2_restart_queue }}" and queue["line"] == "{{ node_name }}", task["name"]
            assert f"{task['register']} is changed" in str(nxt["when"]), task["name"]
            assert "node_enabled | default(true) | bool" in str(nxt["when"]), "disabled nodes are never queued"


def test_source_stamp_is_written_after_config_launcher_and_unit() -> None:
    names = [t["name"] for t, _ in effective_tasks(load(ROLE_DIR / "tasks" / "main.yml"))]
    stamp = names.index("Record the deployed node source (restart when it changed)")
    for before in (
        "Deploy node config file (native)",
        "Deploy launcher script",
        "Create native systemd unit for ROS2 node",
    ):
        assert names.index(before) < stamp, before
    assert names.index("Flush handlers (native)") > stamp


def render_stop_script(tmp_path: Path) -> str:
    """Render the stop_for_build.yml shell script.

    Args:
        tmp_path: Directory for the stopped-for-build file.

    Returns:
        str: Script ready for bash.
    """
    task = load(ROLE_DIR / "tasks" / "stop_for_build.yml")[0]
    return render_probe(task["ansible.builtin.shell"], ros2_stopped_for_build_file=str(tmp_path / "stopped-for-build"))


def test_stop_for_build_records_the_stopped_units_before_stopping_them(tmp_path: Path) -> None:
    fake = tmp_path / "bin" / "systemctl"
    fake.parent.mkdir()
    fake.write_text(
        "#!/bin/bash\n"
        'if [ "$1" = list-units ]; then echo "ros2-a.service loaded active running x"; '
        'echo "ros2-b.service loaded active running y"; exit 0; fi\n'
        'echo "$@" >> "$LOG"\n'
    )
    fake.chmod(0o755)
    env = {**os.environ, "PATH": f"{fake.parent}:{os.environ['PATH']}", "LOG": str(tmp_path / "log")}
    result = subprocess.run(
        ["bash", "-c", render_stop_script(tmp_path)], capture_output=True, text=True, env=env, check=False
    )
    assert result.returncode == 0, result.stderr
    assert (tmp_path / "stopped-for-build").read_text().split() == ["ros2-a.service", "ros2-b.service"]
    assert (tmp_path / "log").read_text().split() == ["stop", "ros2-a.service", "ros2-b.service"]
    script = render_stop_script(tmp_path)
    assert script.index("stopped-for-build") < script.index("xargs systemctl stop"), "recorded before the stop"


def test_start_script_starts_every_unit_stopped_for_a_build_whatever_the_scope(tmp_path: Path) -> None:
    (tmp_path / "stopped-for-build").write_text("ros2-a.service\nros2-c.service\n")
    script = render_start_script(tmp_path, ["a", "b", "c"], scope=["b"])
    result = run_start(tmp_path, script, active=[])
    assert result.returncode == 0, result.stderr
    assert result.stdout.split("\n")[:-1] == ["started ros2-a", "started ros2-b", "started ros2-c"]
    assert (tmp_path / "stopped-for-build").read_text() == "", "cleared after success"


def test_start_script_keeps_the_stopped_list_when_a_start_fails_and_reloads_systemd_first(tmp_path: Path) -> None:
    (tmp_path / "stopped-for-build").write_text("ros2-a.service\n")
    script = render_start_script(tmp_path, ["a"], scope=["a"])
    assert run_start(tmp_path, script, active=[], failing="a").returncode != 0
    assert (tmp_path / "stopped-for-build").read_text() == "ros2-a.service\n"
    assert script.index("daemon-reload") < script.index("for n in"), "queued unit/launcher changes need a reload first"


RECOVER_SCRIPT = PLAYBOOKS_DIR / "files" / "ros2-deploy-recover.sh"


def run_recover(tmp_path: Path, file_age_min: float | None, lock_age_min: float | None) -> list[str]:
    """Run the recovery script in a temp state dir.

    Args:
        tmp_path: Work directory.
        file_age_min: Age of the stopped-for-build file in minutes (None: absent).
        lock_age_min: Age of the deploy lock in minutes (None: absent).

    Returns:
        list[str]: systemctl calls made.
    """
    import time

    state = tmp_path / "state"
    state.mkdir(exist_ok=True)
    for name, age, text in (
        ("stopped-for-build", file_age_min, "ros2-b.service\nros2-a.service\nros2-b.service\n"),
        ("deploy.lock", lock_age_min, ""),
    ):
        path = state / name
        path.unlink(missing_ok=True)
        if age is not None:
            path.write_text(text)
            stamp = time.time() - age * 60
            os.utime(path, (stamp, stamp))
    fake = tmp_path / "bin" / "systemctl"
    fake.parent.mkdir(exist_ok=True)
    fake.write_text('#!/bin/bash\necho "$@" >> "$LOG"\n')
    fake.chmod(0o755)
    log = tmp_path / "log"
    log.unlink(missing_ok=True)
    env = {
        **os.environ,
        "PATH": f"{fake.parent}:{os.environ['PATH']}",
        "LOG": str(log),
        "ROS2_DEPLOY_DIR": str(state),
        "ROS2_RECOVER_SLEEP": "0",
    }
    result = subprocess.run(
        ["bash", str(RECOVER_SCRIPT), "15", "240"], capture_output=True, text=True, env=env, check=False
    )
    assert result.returncode == 0, result.stderr
    return log.read_text().split("\n")[:-1] if log.exists() else []


def test_recover_script_starts_the_stopped_units_only_when_the_list_is_old_and_no_deploy_is_running(
    tmp_path: Path,
) -> None:
    assert run_recover(tmp_path, None, None) == [], "nothing recorded"
    assert run_recover(tmp_path, 5, None) == [], "recent: a deploy may still be building"
    assert run_recover(tmp_path, 30, 10) == [], "a live deploy lock (fresh) blocks recovery"
    calls = run_recover(tmp_path, 30, None)
    assert calls == ["start ros2-a.service", "start ros2-b.service"], "unique, sorted, started one by one"
    assert (tmp_path / "state" / "stopped-for-build").read_text() == "", "cleared"
    assert run_recover(tmp_path, 30, 300) == ["start ros2-a.service", "start ros2-b.service"], "a stale lock is ignored"


def test_recover_timer_is_installed_by_the_deploy_playbooks_with_lock_taken_and_released() -> None:
    install = load(TASKS_DIR / "deploy_guard.yml")
    text = (TASKS_DIR / "deploy_guard.yml").read_text()
    assert (
        "ros2-deploy-recover.timer" in text and "OnUnitActiveSec=2min" in text and "ros2-deploy-recover.service" in text
    )
    assert "ros2_recover_after_min" in text and "ros2_deploy_lock_max_age_min" in text
    timer = next(
        t
        for t in install
        if "ansible.builtin.systemd" in t and t["ansible.builtin.systemd"].get("name") == "ros2-deploy-recover.timer"
    )
    assert (
        timer["ansible.builtin.systemd"]["enabled"] is True and timer["ansible.builtin.systemd"]["state"] == "started"
    )
    all_vars = load(ANSIBLE_DIR / "group_vars" / "all.yml")
    assert all_vars["ros2_deploy_lock"] == "{{ ros2_deploy_state_dir }}/deploy.lock"
    assert all_vars["ros2_stopped_for_build_file"] == "{{ ros2_deploy_state_dir }}/stopped-for-build"
    for target in TARGETS:
        play = load(PLAYBOOKS_DIR / f"deploy_nodes_{target}.yml")[0]
        pre = play["pre_tasks"]
        guard = next(
            i for i, t in enumerate(pre) if "deploy_guard.yml" in str(t.get("ansible.builtin.include_tasks", ""))
        )
        lock = next(
            i for i, t in enumerate(pre) if t.get("ansible.builtin.file", {}).get("path") == "{{ ros2_deploy_lock }}"
        )
        assert guard < lock == len(pre) - 1, "guard installed first, lock taken last in pre_tasks"
        assert pre[lock]["ansible.builtin.file"]["state"] == "touch" and "always" in pre[lock]["tags"]
        last = play["post_tasks"][-1]
        assert last["ansible.builtin.file"]["state"] == "absent" and "always" in last["tags"], "released after verify"


@pytest.mark.parametrize("target", TARGETS)
def test_ros2_master_and_fastdds_watch_no_repo_path(target: str) -> None:
    """They run no repo code: a constant source key (restart only for config/unit changes), not a README-only dir."""
    types = load(ANSIBLE_DIR / "group_vars" / f"{target}.yml")["ros2_node_type_defaults"]
    for name in ("ros2_master", "fastdds_discovery_server"):
        assert types[name]["src_paths"] == [], name
