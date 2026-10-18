"""Runner wiring of the skill plugin: SDK options, init-message check and the permission gate kind."""

import asyncio
import logging
from pathlib import Path

import pytest
from claude_agent_sdk import PermissionResultAllow, PermissionResultDeny, SystemMessage, ToolPermissionContext
from pydantic import ValidationError

from claude_agent.config import ClaudeAgentConfig
from claude_agent.runner import build_options
from claude_agent.skills import PLUGIN_DIR, PLUGIN_NAME, discover_skills, qualified_names
from claude_agent.tools import NOTES_TOOLS, EffectorGate, classify_tool
from tests.test_runner import make_result, make_runner

QUALIFIED = qualified_names(discover_skills(PLUGIN_DIR))
STILL_DENIED = ["Bash", "Task", "Agent", "TodoWrite", "SlashCommand"]


def decide(gate: EffectorGate, tool: str, tool_input: dict) -> object:
    return asyncio.run(gate.can_use_tool(tool, tool_input, ToolPermissionContext(tool_use_id="t1")))


def init_message(**data) -> SystemMessage:
    return SystemMessage(subtype="init", data=data)


def test_build_options_with_skills(config: ClaudeAgentConfig) -> None:
    assert QUALIFIED
    opts = build_options(config, "tok", EffectorGate(config), {}, "SYS")
    assert opts.plugins == [{"type": "local", "path": str(PLUGIN_DIR)}]
    assert opts.skills == QUALIFIED
    assert opts.tools == [*NOTES_TOOLS, "Skill"]
    assert "Skill" not in opts.disallowed_tools
    assert set(STILL_DENIED) <= set(opts.disallowed_tools)
    assert opts.allowed_tools == []
    assert opts.setting_sources == []
    assert opts.strict_mcp_config is True


def test_build_options_without_skills(config: ClaudeAgentConfig) -> None:
    cfg = config.model_copy(update={"skills_enabled": False})
    opts = build_options(cfg, "tok", EffectorGate(cfg), {}, "SYS")
    assert not opts.plugins
    assert opts.skills is None
    assert opts.tools == NOTES_TOOLS
    assert "Skill" in opts.disallowed_tools
    assert set(STILL_DENIED) <= set(opts.disallowed_tools)


def test_config_refuses_plugin_dir_inside_workdir(tmp_path: Path) -> None:
    inside = tmp_path / "work" / "plugin"
    with pytest.raises(ValidationError) as err:
        ClaudeAgentConfig(workdir=str(tmp_path / "work"), skills_plugin_dir=str(inside))
    assert str(inside.resolve()) in str(err.value)
    assert str((tmp_path / "work").resolve()) in str(err.value)
    with pytest.raises(ValidationError):
        ClaudeAgentConfig(workdir=str(tmp_path / "work"), skills_plugin_dir=str(tmp_path / "work"))
    assert ClaudeAgentConfig(workdir=str(tmp_path / "work"), skills_plugin_dir=str(tmp_path / "other"))


def test_config_defaults() -> None:
    cfg = ClaudeAgentConfig()
    assert cfg.skills_enabled is True
    assert cfg.skills_plugin_dir == ""


def test_init_message_stores_tools_and_reports_missing_skills_once(tmp_path: Path, caplog) -> None:
    runner, events, _ = make_runner(tmp_path)
    assert runner.session_tools is None
    msg = init_message(tools=["Read", "Skill"], skills=[{"name": "other:x"}])
    with caplog.at_level(logging.ERROR, logger="claude_agent"):
        runner.handle_init(msg)
        runner.handle_init(msg)
    assert runner.session_tools == {"Read", "Skill"}
    errors = [e for e in events.history() if e["type"] == "error"]
    assert len(errors) == 1
    assert all(name in errors[0]["message"] for name in QUALIFIED)
    assert len([r for r in caplog.records if r.levelno == logging.ERROR]) == 1


def test_init_message_with_all_skills_is_quiet(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path)
    runner.handle_init(init_message(tools=["Read"], skills=list(QUALIFIED)))
    assert not [e for e in events.history() if e["type"] == "error"]


def test_init_message_without_skills_key_warns_once(tmp_path: Path, caplog) -> None:
    runner, events, _ = make_runner(tmp_path)
    with caplog.at_level(logging.WARNING, logger="claude_agent"):
        runner.handle_init(init_message(tools=["Read"]))
        runner.handle_init(init_message(tools=["Read"]))
    warnings = [r for r in caplog.records if "skill listing not reported; cannot check" in r.getMessage()]
    assert len(warnings) == 1
    assert not [e for e in events.history() if e["type"] == "error"]


def test_non_init_system_message_is_ignored(tmp_path: Path) -> None:
    runner, _, _ = make_runner(tmp_path)
    runner.handle_init(SystemMessage(subtype="other", data={"tools": ["X"]}))
    assert runner.session_tools is None


def test_init_check_skipped_when_skills_disabled(tmp_path: Path) -> None:
    cfg = ClaudeAgentConfig(skills_enabled=False)
    runner, events, _ = make_runner(tmp_path, config=cfg)
    runner.handle_init(init_message(tools=["Read"], skills=[]))
    assert runner.session_tools == {"Read"}
    assert not [e for e in events.history() if e["type"] == "error"]


def test_classify_skill(config: ClaudeAgentConfig) -> None:
    assert classify_tool(config, "Skill") == "skill"


def test_gate_allows_configured_skill_and_refuses_others(config: ClaudeAgentConfig) -> None:
    gate = EffectorGate(config)
    assert isinstance(decide(gate, "Skill", {"skill": QUALIFIED[0]}), PermissionResultAllow)
    unknown = decide(gate, "Skill", {"skill": f"{PLUGIN_NAME}:nope"})
    assert isinstance(unknown, PermissionResultDeny)
    assert f"{PLUGIN_NAME}:nope" in unknown.message
    assert isinstance(decide(gate, "Skill", {}), PermissionResultDeny)


def test_gate_denies_skill_when_disabled(config: ClaudeAgentConfig) -> None:
    cfg = config.model_copy(update={"skills_enabled": False})
    assert isinstance(decide(EffectorGate(cfg), "Skill", {"skill": QUALIFIED[0]}), PermissionResultDeny)


async def test_consume_response_runs_the_init_check(tmp_path: Path) -> None:
    script = [init_message(tools=["Read", "Skill"], skills=[{"name": "other:x"}]), make_result()]
    runner, events, _ = make_runner(tmp_path, script=script)
    await runner.start_instruction("hi")
    await runner.wait_idle()
    assert runner.session_tools == {"Read", "Skill"}
    errors = [e for e in events.history() if e["type"] == "error"]
    assert len(errors) == 1
    assert all(name in errors[0]["message"] for name in QUALIFIED)


def test_skill_call_is_free_and_not_refused_by_plan_or_time(config: ClaudeAgentConfig) -> None:
    gate = EffectorGate(config, time_left=lambda: 0.0)
    before = (gate.plan.ro_used, gate.plan.rw_used, gate.plan.turns_used)
    for _ in range(3):
        assert isinstance(decide(gate, "Skill", {"skill": QUALIFIED[0]}), PermissionResultAllow)
    assert (gate.plan.ro_used, gate.plan.rw_used, gate.plan.turns_used) == before
    # no plan is set and no time is left: an effector call is refused, the Skill call above was not
    assert isinstance(decide(gate, "mcp__robot__set_gripper", {"subsystem": "x"}), PermissionResultDeny)
