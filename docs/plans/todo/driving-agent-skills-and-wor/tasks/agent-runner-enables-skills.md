---
title: Agent runner enables skills
status: todo
repo: ros2-lerobot-sverve-platform
value: The agent can load its skills on demand while every other built-in tool stays off.
files: [nodes/claude_agent/claude_agent/runner.py, nodes/claude_agent/claude_agent/tools.py, nodes/claude_agent/claude_agent/config.py, ansible/group_vars/client.yml, nodes/claude_agent/tests/test_skill_options.py, nodes/claude_agent/README.md]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_options.py tests/test_runner.py tests/test_tools.py tests/test_config.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_claude_agent_config.py -q && uv run poe lint-ansible"
depends_on: [agent-skill-plugin-loader, mcp-exploration-status-tool]
owns: []
uses: [skills-plugin]
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

# Agent runner enables skills

## Requirement

- `config.py` (D1): `skills_enabled: bool = True`, `skills_plugin_dir: str = ""` ("" = the packaged plugin dir); a model validator refuses a resolved plugin dir equal to or inside the resolved workdir (error names both paths); docstring entries. `ansible/group_vars/client.yml` claude_agent config gets both keys with a comment (client.yml equals the code defaults).
- `runner.py` `build_options`: when skills_enabled, `plugins=[{"type": "local", "path": str(plugin_dir)}]`, `skills=qualified_names(discover_skills(plugin_dir))`, `tools=NOTES_TOOLS + ["Skill"]`, `disallowed_tools` = BUILTIN_TOOLS without "Skill"; `setting_sources=[]`, `strict_mcp_config`, `allowed_tools` otherwise empty (the SDK adds the `Skill(<name>)` rules). When disabled, the options are exactly as today.
- `runner.py`: on the session's init `SystemMessage` (subtype init), store `data.get("tools")` as the session tool names (`self.session_tools: set[str] | None`) and call `missing_skills`; missing skills -> one error log line and one `error` event per session naming them; no `skills` key -> one warning log line ("skill listing not reported; cannot check").
- `tools.py`: `classify_tool` returns a new kind `skill` for "Skill"; the gate allows it (free, not counted, not refused by plan or time) when the input `skill` is one of the configured qualified names, refuses others with a clear message; Skill stays denied when skills are disabled.
- `README.md`: the Skills section gains the runner wiring (options, init check, gate kind).
- Tests first in new `tests/test_skill_options.py`, seen failing: build_options with skills on/off (plugins, skills, tools, disallowed_tools contents; Bash, Task, Agent, TodoWrite, SlashCommand still disallowed); config refuses a plugin dir inside the workdir; init message handling stores tools and reports missing skills once; gate allows a configured skill and refuses an unknown one and Skill when disabled.

## Acceptance conditions

- a1: build_options enables exactly the plugin skills and keeps every other built-in denied [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_options.py -q -k build_options]
- a2: a plugin dir inside the workdir is refused [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_options.py -q -k inside_workdir]
- a3: the init message is read for tools and missing skills are reported once [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_options.py -q -k init_message]
- a4: the gate allows configured skills only [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_options.py -q -k gate]
- a5: client.yml equals the code defaults [evidence: command: uv run pytest tests/test_claude_agent_config.py -q -k equals_the_code_defaults]
- a6: existing runner, tools and config tests and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_runner.py tests/test_tools.py tests/test_config.py -q && uv run --frozen poe lint]

## Files

- `nodes/claude_agent/claude_agent/runner.py`
- `nodes/claude_agent/claude_agent/tools.py`
- `nodes/claude_agent/claude_agent/config.py`
- `ansible/group_vars/client.yml`
- `nodes/claude_agent/tests/test_skill_options.py`
- `nodes/claude_agent/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
