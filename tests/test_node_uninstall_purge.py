"""colcon_purge: the uninstall path removes a retired C++ node's colcon source, build, install and stamps.

The behaviour test runs the real task file through ansible-playbook against a temp workspace.
"""

import re
import subprocess
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
ROLE_TASKS = REPO_ROOT / "ansible/roles/ros2_node_deploy/tasks"
PURGE_TASKS = ROLE_TASKS / "colcon_purge.yml"
ANSIBLE_PLAYBOOK = REPO_ROOT / ".venv/bin/ansible-playbook"
STAMP_KEYS = ("aaa111", "bbb222")


def make_workspace(root: Path) -> Path:
    """Create a workspace holding pkg_a and pkg_b output plus install/setup.bash.

    :param root: parent directory.
    :return: the workspace path.
    """
    ws = root / "ws"
    for package in ("pkg_a", "pkg_b"):
        for sub in ("src", "build", "install"):
            (ws / sub / package).mkdir(parents=True)
            (ws / sub / package / "file.txt").write_text("x")
    (ws / "install/setup.bash").write_text("# setup")
    for key in STAMP_KEYS:
        (ws / f".built-pkg_a-{key}").write_text("")
    (ws / ".built-pkg_b-ccc333").write_text("")
    return ws


def run_purge(tmp_path: Path, ws: Path) -> str:
    """Run colcon_purge.yml for pkg_a in a wrapper play.

    :param tmp_path: directory for the wrapper play.
    :param ws: workspace to purge.
    :return: ansible-playbook stdout.
    """
    play = [
        {
            "hosts": "localhost",
            "gather_facts": False,
            "vars": {"node_colcon_purge": [{"package": "pkg_a", "workspace": str(ws)}]},
            "tasks": [{"ansible.builtin.include_tasks": str(PURGE_TASKS)}],
        }
    ]
    play_file = tmp_path / "play.yml"
    play_file.write_text(yaml.safe_dump(play))
    result = subprocess.run(
        [str(ANSIBLE_PLAYBOOK), "-c", "local", "-i", "localhost,", str(play_file)],
        capture_output=True,
        text=True,
        cwd=tmp_path,
        env={"ANSIBLE_LOCALHOST_WARNING": "False", "PATH": "/usr/bin:/bin", "HOME": str(tmp_path)},
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return result.stdout


def test_purge_removes_only_the_named_package_and_is_idempotent(tmp_path: Path) -> None:
    ws = make_workspace(tmp_path)
    run_purge(tmp_path, ws)
    for sub in ("src", "build", "install"):
        assert not (ws / sub / "pkg_a").exists()
        assert (ws / sub / "pkg_b/file.txt").exists()
    assert not list(ws.glob(".built-pkg_a-*"))
    assert (ws / ".built-pkg_b-ccc333").exists()
    assert (ws / "install/setup.bash").exists()
    second = run_purge(tmp_path, ws)
    assert re.search(r"changed=0\b", second), second


def test_uninstall_block_includes_colcon_purge_with_length_guard() -> None:
    tasks = yaml.safe_load((ROLE_TASKS / "main.yml").read_text())
    block = next(t for t in tasks if t.get("name") == "Uninstall ROS2 node (present is false)")
    assert "node_present" in block["when"] and "not" in block["when"]
    names = [t.get("name") for t in block["block"]]
    include = next(t for t in block["block"] if "colcon_purge.yml" in str(t.get("ansible.builtin.include_tasks", "")))
    assert include["ansible.builtin.include_tasks"]["apply"]["become"] is True
    assert "node_colcon_purge" in include["when"] and "length > 0" in include["when"]
    assert names.index(include["name"]) > names.index("Remove ROS2 node config directory")
    assert names.index(include["name"]) < names.index("Reload systemd after uninstall")


def test_purge_task_file_sets_no_become() -> None:
    assert "become" not in PURGE_TASKS.read_text()


def test_resolve_and_deploy_maps_colcon_purge_with_empty_default() -> None:
    text = (REPO_ROOT / "ansible/playbooks/tasks/resolve_and_deploy.yml").read_text()
    assert re.search(r"node_colcon_purge:.*\.colcon_purge \| default\(\[\]\)", text)
