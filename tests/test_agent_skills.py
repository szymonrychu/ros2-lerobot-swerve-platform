"""Static invariants of the agent skills in nodes/claude_agent (tools exist, optional tools fenced, read-only)."""

import re
from pathlib import Path

import yaml

from tests.test_claude_agent_config import NODE_DIR, REPO_ROOT, client_vars, mcp_tool_names, node_entry

PLUGIN_DIR = NODE_DIR / "claude_agent" / "agent_plugin"
PLAN_TOOLS = NODE_DIR / "claude_agent" / "plan_tools.py"
REPO_SYNC = REPO_ROOT / "ansible" / "playbooks" / "tasks" / "repo_sync.yml"
OPTIONAL_SECTION = "## When extra perception tools exist"
OPTIONAL_TOOLS = (
    "set_head_angles",
    "head_look_at",
    "head_home",
    "get_3d_map_status",
    "query_3d_map_at",
    "get_3d_free_space",
)
BACKTICKED = re.compile(r"`([^`\n]+)`")
TOOL_CALL = re.compile(r"^(?:agent\.)?([a-z][a-z0-9]*(?:_[a-z0-9]+)+|[a-z]+)\(")
AGENT_TOOL = re.compile(r"^agent\.[a-z_]+")
BARE_NAME = re.compile(r"^[a-z]+(?:_[a-z0-9]+)+$")
# Backticked snake_case words that are arguments or values, not tools.
# Argument and result field names the skills put in backticks (not tools).
NON_TOOL_NAMES = {
    "p_retry_success",
    "retry_change",
    "problem_type",
    "support_z",
    "height_m",
    "position_source",
    "grasp_ready",
    "surface_height_m",
}
PLAN_TOOL_NAMES = re.compile(r'^[A-Z_]+_TOOL = "([a-z_]+)"$', re.MULTILINE)


def skill_files() -> list[Path]:
    return sorted(PLUGIN_DIR.glob("skills/*/SKILL.md"))


def plan_tool_names() -> set[str]:
    return set(PLAN_TOOL_NAMES.findall(PLAN_TOOLS.read_text()))


def split_optional(text: str) -> tuple[str, str]:
    """Return (text outside the optional section, the optional section)."""
    start = text.find(OPTIONAL_SECTION)
    if start < 0:
        return text, ""
    rest = text[start + len(OPTIONAL_SECTION) :]
    end = re.search(r"^## ", rest, re.MULTILINE)
    stop = start + len(OPTIONAL_SECTION) + (end.start() if end else len(rest))
    return text[:start] + text[stop:], text[start:stop]


def tool_references(text: str) -> set[str]:
    """Backticked tool calls (`name(...)`), `agent.<tool>` names and bare snake_case tool names."""
    refs = set()
    for span in BACKTICKED.findall(text):
        match = TOOL_CALL.match(span)
        agent_tool = AGENT_TOOL.match(span)
        if agent_tool:
            refs.add(agent_tool.group(0))
        elif match:
            refs.add(match.group(1))
        elif BARE_NAME.match(span) and span not in NON_TOOL_NAMES:
            refs.add(span)
    return refs


def test_skills_exist() -> None:
    assert (PLUGIN_DIR / "skills" / "recover-from-failure" / "SKILL.md").is_file()


def test_every_referenced_tool_exists() -> None:
    known = mcp_tool_names() | {f"agent.{n}" for n in plan_tool_names()} | {"Skill"}
    assert {"agent.revise_plan", "agent.complete_phase"} <= known
    for path in skill_files():
        unknown = tool_references(path.read_text()) - known
        assert not unknown, f"{path.parent.name} names unknown tools {sorted(unknown)}"


def test_tool_reference_scanner_flags_bare_unknown_tool() -> None:
    refs = tool_references("Check `list_pois` first, then `fly_away`, set `retry_change`, use `agent.nope`.")
    assert refs == {"list_pois", "fly_away", "agent.nope"}


def test_tool_reference_scanner_flags_unknown_tool() -> None:
    assert tool_references("Call `fly_away()` and `agent.nope` then `stop()`.") == {"fly_away", "stop", "agent.nope"}


def test_optional_tools_only_in_optional_section() -> None:
    for path in skill_files():
        outside, inside = split_optional(path.read_text())
        for word in (*OPTIONAL_TOOLS, "zoom"):
            assert not re.search(rf"\b{word}\b", outside), f"{path.parent.name}: {word} outside the optional section"
        if inside:
            assert "look_around" in inside, f"{path.parent.name}: optional section lacks the look_around fallback"


def test_split_optional_isolates_section() -> None:
    text = f"a head_look_at\n{OPTIONAL_SECTION}\nb zoom look_around\n## Next\nc\n"
    outside, inside = split_optional(text)
    assert "head_look_at" in outside and "zoom" not in outside and "zoom" in inside and "c" in outside


def test_plugin_ships_read_only_from_root_checkout() -> None:
    sync = yaml.safe_load(REPO_SYNC.read_text())
    clone = next(t for t in sync if "ansible.builtin.git" in t)
    assert clone["become"] is True
    assert client_vars()["ros2_node_type_defaults"]["claude_agent"]["user"] == "claude_agent"
    workdir = yaml.safe_load(node_entry("claude_agent")["config"])["workdir"]
    assert PLUGIN_DIR.is_relative_to(NODE_DIR)
    assert not Path(workdir).is_relative_to(REPO_ROOT)
