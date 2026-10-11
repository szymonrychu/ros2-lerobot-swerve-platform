"""Guard rule refusal counter (agent_guard_refusals_total) and the prompt guidance for guard rules and watches."""

import ast
from pathlib import Path

from claude_agent_sdk import ToolResultBlock, UserMessage
from prometheus_client import REGISTRY

from claude_agent.config import ClaudeAgentConfig
from claude_agent.prompt import build_system_prompt
from claude_agent.runner import GUARD_REFUSAL_PATTERN

from .test_runner import make_runner

MCP_GUARD_RULES = Path(__file__).resolve().parents[2] / "mcp_server" / "mcp_server" / "guard_rules.py"
METRIC = "agent_guard_refusals_total"
REFUSAL = (
    "guard rule 'no_arm_retake_from_human' refused take_control: a person took the arm over 4 s ago, wait or ask "
    "(events.human_takeover_age_s=4.0)"
)
WAIT_TOOL = "mcp__robot__wait_for_event"
MOVE_TOOL = "mcp__robot__navigate_to_pose"


def count(rule: str) -> float:
    return REGISTRY.get_sample_value(METRIC, {"rule": rule}) or 0.0


def result(tool_use_id: str, text: str, is_error: bool) -> UserMessage:
    return UserMessage(
        content=[ToolResultBlock(tool_use_id=tool_use_id, content=[{"type": "text", "text": text}], is_error=is_error)]
    )


def feed(tmp_path: Path, tool: str, text: str, is_error: bool, tool_use_id: str = "t1") -> None:
    runner, _, _ = make_runner(tmp_path)
    runner.tools_in_flight[tool_use_id] = tool
    runner.note_tool_results(result(tool_use_id, text, is_error))


def test_pattern_is_pinned_to_the_mcp_server_source() -> None:
    tree = ast.parse(MCP_GUARD_RULES.read_text())
    values = [
        ast.literal_eval(node.value)
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "GUARD_REFUSAL_PATTERN" for t in node.targets)
    ]
    assert values == [GUARD_REFUSAL_PATTERN]


def test_error_refusal_is_counted_once(tmp_path: Path) -> None:
    before = count("no_arm_retake_from_human")
    feed(tmp_path, MOVE_TOOL, f"{REFUSAL}; again: {REFUSAL}", True)
    assert count("no_arm_retake_from_human") == before + 1


def test_several_rules_in_one_text(tmp_path: Path) -> None:
    text = (
        "guard rule 'rule_one' refused navigate_to_pose: stow the arm (arm.outside_m=0.05); "
        "guard rule 'rule_two' refused queued base_relative step: hold on (battery_v=10.1)"
    )
    before = (count("rule_one"), count("rule_two"))
    feed(tmp_path, MOVE_TOOL, text, True)
    assert (count("rule_one"), count("rule_two")) == (before[0] + 1, before[1] + 1)


def test_wait_for_event_result_is_counted_even_when_not_an_error(tmp_path: Path) -> None:
    before = count("queued_rule")
    text = '{"events": [{"type": "step_failed", "message": "guard rule \'queued_rule\' refused queued base_relative step: no"}]}'
    feed(tmp_path, WAIT_TOOL, text, False)
    assert count("queued_rule") == before + 1


def test_dry_run_log_lines_are_never_counted(tmp_path: Path) -> None:
    before = count("dry_rule")
    text = 'guard_rule_would_refuse {"rule": "dry_rule", "mode": "dry_run", "what": "navigate_to_pose"}'
    feed(tmp_path, WAIT_TOOL, text, True)
    feed(tmp_path, MOVE_TOOL, text, False, "t2")
    assert count("dry_rule") == before


def test_non_matching_errors_and_other_results_are_not_counted(tmp_path: Path) -> None:
    before = count("sneaky")
    feed(tmp_path, MOVE_TOOL, "tilt guard: base disabled", True)
    feed(tmp_path, "mcp__robot__get_robot_state", "guard rule 'sneaky' refused navigate_to_pose: x", False, "t2")
    assert count("sneaky") == before


def test_prompt_has_guard_rule_and_watch_guidance() -> None:
    text = build_system_prompt(ClaudeAgentConfig())
    for phrase in (
        "guard rule '<name>' refused",
        "check_guard_rules",
        "watch_constraints",
        "resume_motions",
        "gripper_effort",
        "tilt_deg",
        "distance_travelled_m",
        "pause",
        "abort",
    ):
        assert phrase in text
