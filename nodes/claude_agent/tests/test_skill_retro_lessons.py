"""Skills and discovery workflow carry the lessons of the 2026-10-22 retrospectives."""

from pathlib import Path

import yaml

from claude_agent.skills import PLUGIN_DIR, check_skill, read_skill

MAX_SKILL_BYTES = 8 * 1024
DISCOVERY_YAML = Path(__file__).resolve().parent.parent / "claude_agent" / "workflow_templates" / "discovery.yaml"


def load(name: str) -> str:
    skill = read_skill(PLUGIN_DIR, name)
    assert check_skill(skill, name) == []
    assert len((PLUGIN_DIR / "skills" / name / "SKILL.md").read_bytes()) < MAX_SKILL_BYTES
    return skill.body


def test_sightseeing_mirror_and_recheck() -> None:
    body = load("sightseeing")
    for word in ("get_topdown_view", "list_pois", "mirror", "window", "glass", "ghost", "keep-out"):
        assert word in body
    assert "once" in body and "draw" in body
    assert "get_exploration_status" in body.split("After the last survey")[1]
    assert "map_epoch_reset" in body and "map_reset" in body
    assert "frontier ids restart" in body and "old frontier ids" in body


def test_survey_object_positions_from_pixel_to_ground() -> None:
    body = load("survey")
    for word in ("pixel_to_ground", "mark_candidate_points", "get_camera_image", "montage", "not calibrated", "rough"):
        assert word in body
    assert "fresh" in body and "current pose" in body
    assert "pixel_to_point" in body and "interior" in body and "height" in body


def test_recover_stopbox_translate_first() -> None:
    body = load("recover-from-failure")
    for word in (
        "StopBox",
        "move_relative",
        "drive",
        "0.1 m/s",
        "1 s",
        "before any turn",
        "invalid source",
        "sensor outage",
    ):
        assert word in body
    assert "retry the goal once" in body


def test_discovery_visit_cap_and_summary_parks_arm() -> None:
    plan = yaml.safe_load(DISCOVERY_YAML.read_text())["plan"]
    phases = {}

    def walk(node: dict) -> None:
        if node["type"] == "phase":
            phases[node["id"]] = node
        for child in node.get("children", []):
            walk(child)
        if "body" in node:
            walk(node["body"])

    walk(plan)
    visit = phases["visit"]
    assert visit["turn_cap"] == 80
    assert "batch" in visit["goal"].lower() and "one turn" in visit["goal"]
    summary = " ".join(phases["summary"]["goal"].split())
    assert summary.startswith("Take the arm lease and release_control(subsystem='all') so the arm parks, then report")
