"""Pins the vanilla superpowers flow: plugin enabled, flow rules present, docs/plans untracked and ignored."""

import json
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SETTINGS = REPO / ".claude" / "settings.json"
RULES = REPO / ".claude" / "rules"
IGNORED_PATHS = (".worktrees/x", ".scratchpad/x", ".logs/x", "docs/plans/x")


def git(*args: str) -> str:
    """Run git in the repo root.

    Args:
        *args (str): The git arguments.

    Returns:
        str: The standard output.
    """
    return subprocess.run(["git", *args], cwd=REPO, check=True, capture_output=True, text=True).stdout


def test_official_superpowers_is_enabled() -> None:
    """The project enables the official superpowers plugin and keeps the attribution block."""
    settings = json.loads(SETTINGS.read_text())
    assert settings["enabledPlugins"]["superpowers@claude-plugins-official"] is True
    assert settings["attribution"] == {"commit": "", "pr": ""}


def test_flow_rules_are_present() -> None:
    """The superpowers-flow rule carries the six amendments and housekeeping the orchestrator rule."""
    flow = (RULES / "superpowers-flow.md").read_text()
    for term in (
        "workbench:research-docs",
        "One human gate",
        "docs/plans/<name>/implementation.md",
        "retro.md",
        "orchestrator never updates code",
        "workbench.py logs",
    ):
        assert term in flow, f"superpowers-flow.md lacks {term!r}"
    assert "orchestrator never edits" in (RULES / "housekeeping.md").read_text()


def test_docs_plans_is_untracked() -> None:
    """The legacy docs/plans tree is no longer tracked."""
    assert git("ls-files", "docs/plans") == ""


def test_per_improvement_paths_are_ignored() -> None:
    """Worktrees, scratchpad, logs and plans are gitignored."""
    assert git("check-ignore", *IGNORED_PATHS).split() == list(IGNORED_PATHS)
