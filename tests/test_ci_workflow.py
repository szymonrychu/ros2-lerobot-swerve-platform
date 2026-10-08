"""Guards for .github/workflows/ci.yml: uv-based, pinned, and covering every node project."""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
WORKFLOW_PATH = REPO_ROOT / ".github" / "workflows" / "ci.yml"
NODES_DIR = REPO_ROOT / "nodes"
UV_VERSION = "0.11.29"


def load_workflow() -> dict:
    """Parse the CI workflow.

    Returns:
        The workflow mapping.
    """
    return yaml.safe_load(WORKFLOW_PATH.read_text())


def discover_node_dirs() -> set[str]:
    """Find every node project on the filesystem.

    Returns:
        Directories of nodes/**/pyproject.toml, relative to nodes/.
    """
    return {p.parent.relative_to(NODES_DIR).as_posix() for p in NODES_DIR.rglob("pyproject.toml")}


def test_workflow_has_no_poetry() -> None:
    assert "poetry" not in WORKFLOW_PATH.read_text().lower()


def test_uv_pinned_in_every_job() -> None:
    jobs = load_workflow()["jobs"]
    for name, job in jobs.items():
        setups = [s for s in job["steps"] if str(s.get("uses", "")).startswith("astral-sh/setup-uv@v")]
        assert setups, f"job {name} does not install uv"
        for step in setups:
            assert str(step["with"]["version"]) == UV_VERSION
            assert step["with"]["python-version"] == "3.12"


def test_root_tests_job_present() -> None:
    jobs = load_workflow()["jobs"]
    runs = [s.get("run", "") for job in jobs.values() for s in job["steps"]]
    assert any("uv run --frozen pytest tests" in r for r in runs)
    assert any("uv run --frozen poe lint" in r for r in runs)


def test_node_matrix_equals_filesystem() -> None:
    job = load_workflow()["jobs"]["nodes"]
    matrix = set(job["strategy"]["matrix"]["node"])
    assert matrix == discover_node_dirs()
    assert len(matrix) == 18
    run = "\n".join(s.get("run", "") for s in job["steps"])
    for cmd in ("uv lock --check", "uv run --frozen poe lint", "uv run --frozen pytest -q"):
        assert cmd in run


def test_ansible_job_installs_collections_before_syntax_check() -> None:
    """Roles use ansible.posix modules (mount), so the collection must be installed before --syntax-check."""
    runs = [str(s.get("run", "")) for s in load_workflow()["jobs"]["lint-ansible"]["steps"]]
    galaxy = "ansible-galaxy collection install -r ansible/requirements.yml"
    install = next(i for i, r in enumerate(runs) if galaxy in r)
    check = next(i for i, r in enumerate(runs) if "poe test-ansible" in r)
    assert install < check
