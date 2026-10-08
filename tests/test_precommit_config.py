"""Guards the root .pre-commit-config.yaml against the legacy black/isort/flake8 toolchain."""

import re
from pathlib import Path

import yaml

CONFIG_PATH = Path(__file__).resolve().parent.parent / ".pre-commit-config.yaml"
LEGACY_HOOK_IDS = {"black", "isort", "autoflake", "flake8"}
RUFF_REV = "v0.15.20"
ANSIBLE_FILES = ("ansible/group_vars/client.yml", "ansible/roles/ros2_node_deploy/tasks/main.yaml")


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


def test_ansible_lint_hook_matches_ansible_yaml() -> None:
    config = yaml.safe_load(CONFIG_PATH.read_text())
    hook = next(hook for repo in config["repos"] for hook in repo["hooks"] if hook["id"] == "ansible-lint")
    for path in ANSIBLE_FILES:
        assert re.search(hook["files"], path), f"{hook['files']!r} does not match {path}"
    assert not re.search(hook["files"], "nodes/web_ui/config.yml")


def test_ansible_lint_hook_uses_pinned_script() -> None:
    config = yaml.safe_load(CONFIG_PATH.read_text())
    hook = next(hook for repo in config["repos"] for hook in repo["hooks"] if hook["id"] == "ansible-lint")
    assert hook["entry"] == "bash scripts/lint-ansible.sh"
