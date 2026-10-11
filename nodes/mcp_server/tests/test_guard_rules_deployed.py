"""The guard rules deployed in ansible/group_vars/client.yml validate and start in dry_run."""

from pathlib import Path
from typing import Any

import yaml

from mcp_server.config import MotionQueueSettings
from mcp_server.guard_rules import GuardRule, GuardRuleSettings
from mcp_server.tools import TOOL_SUBSYSTEM

REPO_ROOT = Path(__file__).resolve().parents[3]
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
EXPECTED_RULES = [
    "base_motion_arm_outside_footprint",
    "base_motion_carrying_without_object_footprint",
    "no_arm_retake_from_human",
    "wrist_roll_needs_lifted_arm",
    "no_stretched_shoulder_overreach",
]
# Every message must name a way out (a tool or action the model can take instead).
WAY_OUT_WORDS = (
    "arm_home",
    "release_object",
    "carried_object_radius_m",
    "base_motion",
    "wait",
    "retake",
    "lift",
    "park",
)


def deployed_mcp_config() -> dict[str, Any]:
    """Load the client.yml mcp_server config block.

    Returns:
        dict[str, Any]: The parsed `config:` block of the mcp_server node.
    """
    doc = yaml.safe_load(CLIENT_VARS.read_text())
    node = next(n for n in doc["ros2_nodes"] if n["name"] == "mcp_server")
    return yaml.safe_load(node["config"])


def deployed_rules() -> list[GuardRule]:
    """Validate the deployed guard_rules section.

    Returns:
        list[GuardRule]: The validated rules.
    """
    return GuardRuleSettings.model_validate(deployed_mcp_config()["guard_rules"]).rules


def test_guard_rules_section_validates_with_the_five_rules_in_order() -> None:
    settings = GuardRuleSettings.model_validate(deployed_mcp_config()["guard_rules"])
    assert settings.enabled is True
    assert [r.name for r in settings.rules] == EXPECTED_RULES


def test_every_deployed_rule_starts_in_dry_run() -> None:
    assert {r.mode for r in deployed_rules()} == {"dry_run"}


def test_every_message_names_a_way_out() -> None:
    for rule in deployed_rules():
        assert any(word in rule.message.lower() for word in WAY_OUT_WORDS), rule.message


def test_r1_names_both_exits() -> None:
    r1 = next(r for r in deployed_rules() if r.name == "base_motion_arm_outside_footprint")
    assert "arm_home" in r1.message
    assert "release_control" in r1.message
    assert "keep_position" in r1.message


def test_scopes_match_the_design() -> None:
    by_name = {r.name: r for r in deployed_rules()}
    base_tools = {name for name, need in TOOL_SUBSYSTEM.items() if need == "base"}
    base_steps = ["base_relative", "navigate_to_pose"]
    for name in EXPECTED_RULES[:2]:
        assert set(by_name[name].applies_to.tools) == base_tools
        assert by_name[name].applies_to.steps == base_steps
    assert by_name["no_arm_retake_from_human"].applies_to.tools == ["take_control"]
    assert "grasp_object" not in by_name["wrist_roll_needs_lifted_arm"].applies_to.arm_motion_from
    assert {"grasp_object", "grasp"} <= set(by_name["no_stretched_shoulder_overreach"].applies_to.arm_motion_from)
    assert by_name["no_arm_retake_from_human"].when_unknown == "allow"
    assert all(by_name[n].when_unknown == "reject" for n in EXPECTED_RULES if n != "no_arm_retake_from_human")


def test_thresholds_match_the_design() -> None:
    by_name = {r.name: r for r in deployed_rules()}
    r1 = {(c.field, c.op): c.value for c in by_name[EXPECTED_RULES[0]].conditions()}
    assert r1 == {("footprint.kind", "eq"): "static", ("arm.outside_footprint_m", "gt"): 0.03}
    r5 = {(c.field, c.op): c.value for c in by_name[EXPECTED_RULES[4]].conditions()}
    assert r5 == {("target.joint.shoulder_lift", "gt"): 1.85, ("target.joint.elbow_flex", "lt"): 0.0}


def test_motion_queue_watch_settings_validate() -> None:
    settings = MotionQueueSettings.model_validate(deployed_mcp_config()["motion_queue"])
    assert settings.watch_rate_hz == 10
    assert settings.watch_max_per_call == 8
