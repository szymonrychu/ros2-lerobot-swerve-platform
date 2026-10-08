"""Guards the root .pre-commit-config.yaml against the legacy black/isort/flake8 toolchain."""

from pathlib import Path

import yaml

CONFIG_PATH = Path(__file__).resolve().parent.parent / ".pre-commit-config.yaml"
LEGACY_HOOK_IDS = {"black", "isort", "autoflake", "flake8"}
RUFF_REV = "v0.15.20"


def load_hooks() -> dict[str, str]:
    """Return a mapping of hook id to repo rev for every hook in the config."""
    config = yaml.safe_load(CONFIG_PATH.read_text())
    return {hook["id"]: repo.get("rev", "") for repo in config["repos"] for hook in repo["hooks"]}


def test_no_legacy_python_hooks() -> None:
    assert not LEGACY_HOOK_IDS & set(load_hooks())


def test_ruff_hooks_pinned() -> None:
    hooks = load_hooks()
    assert hooks.get("ruff-check") == RUFF_REV
    assert hooks.get("ruff-format") == RUFF_REV
