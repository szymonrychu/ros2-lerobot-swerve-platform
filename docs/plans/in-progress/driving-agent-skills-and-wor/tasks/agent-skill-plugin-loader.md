---
title: Agent skill plugin loader
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Skills live as read-only files shipped with the node, checked for structure and for tools that exist."
files: [nodes/claude_agent/claude_agent/skills.py, nodes/claude_agent/claude_agent/agent_plugin/.claude-plugin/plugin.json, nodes/claude_agent/claude_agent/agent_plugin/skills/recover-from-failure/SKILL.md, nodes/claude_agent/tests/test_skills.py, nodes/claude_agent/README.md, tests/test_agent_skills.py, tests/README.md]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_skills.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_agent_skills.py -q && uv run poe lint"
depends_on: [shared-poi-category-tags]
owns: [skills-plugin]
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Agent skill plugin loader

## Requirement

- New `nodes/claude_agent/claude_agent/agent_plugin/.claude-plugin/plugin.json` (D1, U1): `{"name": "robot-skills", "version": "0.1.0", "description": ...}`.
- New `nodes/claude_agent/claude_agent/skills.py`, constants at the top (`PLUGIN_NAME = "robot-skills"`, `PLUGIN_DIR = Path(__file__).parent / "agent_plugin"`, `OPTIONAL_SECTION = "## When extra perception tools exist"`), module-level functions: `plugin_dir(configured: str) -> Path` ("" -> PLUGIN_DIR, else the resolved path); `discover_skills(root: Path) -> list[str]` (sorted directory names under `skills/` holding a SKILL.md); `qualified_names(names: list[str]) -> list[str]` (`robot-skills:<name>`); `read_skill(root: Path, name: str) -> SkillFile` (pydantic: name, description, body) parsing YAML frontmatter with pyyaml; `check_skill(skill: SkillFile, directory: str) -> list[str]` (problems: frontmatter name != directory, empty or > 1024-char description, empty body, a reference to a file next to SKILL.md - skills must be self-contained because the gate keeps file tools inside the workdir); `missing_skills(init_data: dict[str, Any], expected: list[str]) -> list[str] | None` (None when the init message has no `skills` key; entries may be strings or objects with `name`).
- New skill `nodes/claude_agent/claude_agent/agent_plugin/skills/recover-from-failure/SKILL.md` (D2): frontmatter name/description ("Use when a phase failed, a motion was refused or interrupted, or the robot got stuck"); body: stop motion safely (stop, cancel_motions), re-sense (get_body_state, get_control_state, cameras), classify the failure with the complete_phase failure types and p_retry_success honestly, retry only with a change (retry_change), else agent.revise_plan or report; mark a stuck place or hazard with add_poi category problem (problem_type) or a keep-out area; never work around a guard; record what happened for the retrospective. Only existing tools named in backticks.
- New root test `tests/test_agent_skills.py`, written first: (a) every backticked identifier in every SKILL.md that looks like a tool (snake_case followed by `(` or listed as a tool) is a registered mcp_server tool (reuse `mcp_tool_names` from `tests/test_claude_agent_config.py` by import), an `agent.<tool>` planning tool (claude_agent `plan_tools.TOOL_NAMES`, read by import or regex) or `Skill`; (b) the optional tools `set_head_angles`, `head_look_at`, `head_home`, `get_3d_map_status`, `query_3d_map_at`, `get_3d_free_space` and the word `zoom` appear only inside the `## When extra perception tools exist` section, and every skill that has that section also states the base-rotation fallback (the section contains `look_around`); (c) read-only shipping (resolved unknown, D1): `ansible/playbooks/tasks/repo_sync.yml` clones with `become: true`, the claude_agent `node_user` in group_vars is `claude_agent` (not root), and the plugin dir lies under `nodes/claude_agent/` (deployed checkout) while the claude_agent `workdir` in client.yml is outside the repo.
- `nodes/claude_agent/tests/test_skills.py`, written first: discovery and qualified names; plugin.json valid with name robot-skills; every discovered skill passes `check_skill`; `check_skill` reports each problem on synthetic files; `missing_skills` with string and object entries and without the key.
- `nodes/claude_agent/README.md`: new `## Skills` section (where they live, read-only on the robot and why, self-contained rule, the optional-tools section rule, how to add a skill). `tests/README.md` lists `tests/test_agent_skills.py` and its tests.

## Acceptance conditions

- a1: skills are discovered and qualified as robot-skills:<name> [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skills.py -q -k 'discover or qualified']
- a2: every shipped skill passes the structure check and synthetic bad files are reported [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skills.py -q -k check_skill]
- a3: missing_skills reads the init message skills list [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skills.py -q -k missing_skills]
- a4: skills name only registered tools; optional perception tools only in the fenced section with the fallback [evidence: command: uv run pytest tests/test_agent_skills.py -q -k 'tool_references or optional_tools']
- a5: the plugin ships in the root-owned checkout outside the agent workdir [evidence: command: uv run pytest tests/test_agent_skills.py -q -k read_only]
- a6: README has a Skills section [evidence: command: grep -q '^## Skills' nodes/claude_agent/README.md]
- a7: claude_agent lint and root lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen poe lint && cd ../.. && uv run poe lint]

## Files

- `nodes/claude_agent/claude_agent/skills.py`
- `nodes/claude_agent/claude_agent/agent_plugin/.claude-plugin/plugin.json`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/recover-from-failure/SKILL.md`
- `nodes/claude_agent/tests/test_skills.py`
- `nodes/claude_agent/README.md`
- `tests/test_agent_skills.py`
- `tests/README.md`

## Interfaces

- `claude_agent/skills.py:PLUGIN_NAME`, `PLUGIN_DIR`, `plugin_dir`, `discover_skills`, `qualified_names`, `read_skill`, `check_skill`, `missing_skills`, `SkillFile` (contract `skills-plugin`).

## Expected outcome

A plugin directory with one real skill and tests that keep every future skill well-formed and honest about tools.

## Steps

- Write both test files; run them; expect failures.
- Implement skills.py, plugin.json, the skill and docs; run the verify command; commit `feat(claude_agent): read-only skills plugin`.

## Notes

- SDK naming: plugin skills are `<plugin>:<skill>` (subprocess_cli.py `_validate_skill_name`).
- Resolved unknown (design D1): checkout cloned as root (repo_sync.yml become), editable uv install, service User=claude_agent.
