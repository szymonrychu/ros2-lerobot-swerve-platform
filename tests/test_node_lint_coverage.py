"""Root lint tasks must cover every Python node project and every C++ package (discovered from the tree, not a
hand-kept list)."""

import tomllib
from pathlib import Path

import yaml

from tests.test_cpp_scaffolding import discover_cpp_package_dirs, poe_task_body

REPO_ROOT = Path(__file__).resolve().parent.parent
LINT_SCRIPT = REPO_ROOT / "scripts" / "lint-all-nodes.sh"
ROOT_PYPROJECT = REPO_ROOT / "pyproject.toml"
EXCLUDED_PARTS = {".venv", "node_modules", "worktrees", "__pycache__"}
LINT_COMMAND = "uv run --frozen poe lint"
WORKFLOW_PATH = REPO_ROOT / ".github" / "workflows" / "ci.yml"


def discover_node_dirs() -> list[str]:
    """Find every node project directory.

    Returns:
        list[str]: Repo-relative POSIX directories under nodes/ holding a pyproject.toml and its uv.lock, sorted (a
            pyproject.toml without a uv.lock is a sub-package built by its node's uv sync).
    """
    found = []
    for pyproject in (REPO_ROOT / "nodes").rglob("pyproject.toml"):
        if (
            EXCLUDED_PARTS.isdisjoint(pyproject.relative_to(REPO_ROOT).parts)
            and (pyproject.parent / "uv.lock").is_file()
        ):
            found.append(pyproject.parent.relative_to(REPO_ROOT).as_posix())
    return sorted(found)


def lint_nodes_task() -> str:
    """Return the shell body of the root `lint-nodes` poe task.

    Returns:
        str: The task's shell script.
    """
    tasks = tomllib.loads(ROOT_PYPROJECT.read_text())["tool"]["poe"]["tasks"]
    task = tasks["lint-nodes"]
    return task["shell"] if isinstance(task, dict) else task


def test_node_discovery_finds_the_known_projects() -> None:
    """Guards the discovery itself: an empty list would make the coverage tests pass vacuously."""
    dirs = discover_node_dirs()
    assert len(dirs) >= 18
    assert "nodes/mcp_server" in dirs and "nodes/steamdeck_ui/bridge" in dirs


def test_node_discovery_skips_sub_packages_without_a_lock() -> None:
    """A pyproject.toml without its own uv.lock (nodes/mcp_server/mcp_core_cpp, a path dependency built by its node's
    uv sync) is part of its node project, not a node project of its own."""
    dirs = discover_node_dirs()
    assert "nodes/mcp_server/mcp_core_cpp" not in dirs
    assert all((REPO_ROOT / d / "uv.lock").is_file() for d in dirs)


def test_lint_all_nodes_script_covers_every_node_project() -> None:
    """scripts/lint-all-nodes.sh names every nodes/**/pyproject.toml directory."""
    text = LINT_SCRIPT.read_text()
    missing = [d for d in discover_node_dirs() if d not in text]
    assert not missing, f"lint-all-nodes.sh misses {missing}"


def test_lint_all_nodes_script_uses_uv_and_fails_loudly() -> None:
    """The script runs `uv run --frozen poe lint` per node, stops on the first failure and never uses Poetry."""
    text = LINT_SCRIPT.read_text()
    assert LINT_COMMAND in text
    assert "poetry" not in text.lower()
    assert "set -e" in text or "set -euo pipefail" in text
    assert "|| true" not in text and "| tee" not in text


def test_root_lint_nodes_task_covers_every_node_project() -> None:
    """The `lint-nodes` poe task lints every node: either by delegating to the script or by listing each directory."""
    body = lint_nodes_task()
    assert "set -e" in body or "lint-all-nodes.sh" in body
    if "scripts/lint-all-nodes.sh" in body:
        return
    missing = [d for d in discover_node_dirs() if d not in body]
    assert not missing, f"lint-nodes misses {missing}"
    assert LINT_COMMAND in body or body.count("uv run --frozen poe lint") >= len(discover_node_dirs())


def test_every_cpp_node_has_a_ci_row_and_is_linted() -> None:
    """A new nodes/<x>_cpp fails here until the cpp CI matrix lists it; lint-cpp reaches it through its glob."""
    matrix = yaml.safe_load(WORKFLOW_PATH.read_text())["jobs"]["cpp"]["strategy"]["matrix"]["package"]
    lint_cpp = poe_task_body("lint-cpp")
    for pkg_dir in discover_cpp_package_dirs():
        assert pkg_dir.name in matrix, f"{pkg_dir.name} has no row in the cpp CI matrix"
        rel = pkg_dir.relative_to(REPO_ROOT).as_posix()
        assert rel in lint_cpp or (rel.startswith("nodes/") and "nodes/*_cpp" in lint_cpp), f"lint-cpp misses {rel}"


def test_cpp_packages_are_not_python_node_projects() -> None:
    """The Python lint discovery never picks up a C++ package (they carry no pyproject.toml)."""
    cpp_dirs = {d.relative_to(REPO_ROOT).as_posix() for d in discover_cpp_package_dirs()}
    assert cpp_dirs, "C++ package discovery found nothing"
    assert cpp_dirs.isdisjoint(discover_node_dirs())
