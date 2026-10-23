"""Every third-party module the node imports at runtime is a runtime dependency, not a dev-only one."""

import tomllib
from pathlib import Path

PYPROJECT = Path(__file__).resolve().parents[1] / "pyproject.toml"
RUNTIME_IMPORTS = ["httpx"]


def test_runtime_imports_are_runtime_dependencies() -> None:
    project = tomllib.loads(PYPROJECT.read_text())["project"]
    names = {dep.split(">")[0].split("=")[0].split("<")[0].split("[")[0].strip() for dep in project["dependencies"]}
    for module in RUNTIME_IMPORTS:
        assert module in names, f"{module} is imported by the node at runtime but is not in [project] dependencies"
