"""Planning skill: it loads and carries the workflow choice and the generous breakdown rules."""

import re

from claude_agent.skills import PLUGIN_DIR, check_skill, read_skill

MAX_SKILL_BYTES = 8 * 1024
TRIGGERS = {
    "discovery": ("explore", "map", "survey", "look around"),
    "drive_to_target": ("go to", "drive to", "find and go"),
    "pick_place": ("pick up", "put down", "bring", "move an object"),
}


def load() -> str:
    skill = read_skill(PLUGIN_DIR, "planning")
    assert check_skill(skill, "planning") == []
    assert len((PLUGIN_DIR / "skills" / "planning" / "SKILL.md").read_bytes()) < MAX_SKILL_BYTES
    return skill.body


def test_skill_exists_and_passes_check() -> None:
    load()


def test_names_plan_tools() -> None:
    body = load()
    for word in ("start_workflow", "dry_run", "set_task_plan", "revise_plan"):
        assert word in body


def test_workflows_with_trigger_words() -> None:
    body = load()
    for name, words in TRIGGERS.items():
        assert name in body
        for word in words:
            assert word in body


def test_breakdown_rules() -> None:
    body = load().lower()
    for phrase in (
        "orient first",
        "verification phase after every effector phase",
        "an `if`",
        "a `while`",
        "recovery branch",
        "mandatory steps are never silently dropped",
        "poi recording phases",
        "caps and time budget with margin",
        "about twice the estimate",
    ):
        assert phrase in body
    assert re.search(r"`agent\.revise_plan`.*\bonce\b", body)
