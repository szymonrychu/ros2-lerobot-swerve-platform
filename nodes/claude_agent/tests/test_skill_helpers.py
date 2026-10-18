"""Helper skills: approach-and-verify and safe-pick-place exist, pass the structure check and name the key steps."""

import pytest

from claude_agent.skills import PLUGIN_DIR, check_skill, discover_skills, read_skill

APPROACH = "approach-and-verify"
PICK_PLACE = "safe-pick-place"

APPROACH_TERMS = [
    "list_pois",
    "list_objects",
    "list_lookups",
    "suggest_approach_pose",
    "navigate_to_pose",
    "keep-out",
    "slow",
    "get_annotated_camera_image",
    "pixel_to_ground",
    "update_poi",
    "remember_object",
    "complete_phase",
]
PICK_PLACE_TERMS = [
    "get_body_state",
    "check_guard_rules",
    "get_arm_reach",
    "wrist roll",
    "lease",
    "plan_grasp",
    "grasp_object",
    "confirm_grasp",
    "release_control",
    "keep_position",
    "carried_object_radius_m",
    "release_object",
    "recover-from-failure",
]


@pytest.mark.parametrize("name", [APPROACH, PICK_PLACE])
def test_skill_exists_and_passes_check(name: str) -> None:
    assert name in discover_skills(PLUGIN_DIR)
    assert check_skill(read_skill(PLUGIN_DIR, name), name) == []


@pytest.mark.parametrize("term", APPROACH_TERMS)
def test_approach_and_verify_names(term: str) -> None:
    assert term in read_skill(PLUGIN_DIR, APPROACH).body


@pytest.mark.parametrize("term", PICK_PLACE_TERMS)
def test_safe_pick_place_names(term: str) -> None:
    assert term in read_skill(PLUGIN_DIR, PICK_PLACE).body


def test_safe_pick_place_does_not_encourage_unsafe_arm_poses() -> None:
    body = read_skill(PLUGIN_DIR, PICK_PLACE).body.lower()
    assert "fully open" in body  # stated as something to avoid
    assert "never" in body
