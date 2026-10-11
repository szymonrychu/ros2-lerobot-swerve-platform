"""Packaging metadata checks for costmap_zones."""

import tomllib
from pathlib import Path

NODE_DIR = Path(__file__).resolve().parent.parent
README_NAME = "README.md"


def test_pyproject_declares_existing_readme() -> None:
    """pyproject.toml must declare readme = README.md and the file must exist."""
    project = tomllib.loads((NODE_DIR / "pyproject.toml").read_text())["project"]
    assert project.get("readme") == README_NAME
    assert (NODE_DIR / README_NAME).is_file()
