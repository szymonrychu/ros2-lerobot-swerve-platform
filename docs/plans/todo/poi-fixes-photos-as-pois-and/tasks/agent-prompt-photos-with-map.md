---
title: Agent prompt photos with map
status: todo
repo: ros2-lerobot-sverve-platform
value: "The agent tells the person the truth about where photos go on save, load, delete and cleanups"
files: [nodes/claude_agent/claude_agent/prompt.py, nodes/claude_agent/tests/test_prompt.py, nodes/claude_agent/README.md]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q && uv run --frozen poe lint"
depends_on: [mcp-poi-tools-photos]
owns: []
uses: []
model: ""
estimate: 30
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Agent prompt photos with map

## Requirement

D3, D7, D8 (claude_agent): the agent's system prompt states the new photo rules so the agent does not promise photos survive cleanups.

- `nodes/claude_agent/claude_agent/prompt.py`:
  - Named maps paragraph: a map holds its POIs and photos (save_map stores both, load_map replaces both, delete_map removes both); clear_pois defaults to your own POIs and photos and keeps zones; an unrecoverable move and a web UI Reset map clear the live POIs and photos; saved maps are never deleted by a move or a reset.
  - Lookups paragraph: replace "Photos survive sessions and restarts." with: photos survive sessions and restarts, belong to the live map, travel with saved maps, and are deleted with the map or by POI cleanups that include them.
- Tests first, seen failing, in `nodes/claude_agent/tests/test_prompt.py`: the prompt says photos travel with saved maps, that clear_pois includes your own photos by default, and that a Reset map clears the live POIs and photos; it no longer contains a sentence saying photos are never cleared.
- `nodes/claude_agent/README.md`: the prompt summary line about photos, when it has one.

## Acceptance conditions

- a1: the prompt states photos travel with maps and are cleared by cleanups that include them [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q -k photo]
- a2: claude_agent prompt tests and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q && uv run --frozen poe lint]

## Files

- `nodes/claude_agent/claude_agent/prompt.py`
- `nodes/claude_agent/tests/test_prompt.py`
- `nodes/claude_agent/README.md`

## Interfaces

- `nodes/claude_agent/claude_agent/prompt.py` system prompt text (Named maps and Lookups paragraphs).

## Expected outcome

The agent's instructions match the new behaviour: photos travel with saved maps and are removed by map and POI cleanups that include them.

## Steps

- Add the photo assertions to `nodes/claude_agent/tests/test_prompt.py`; run `cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q -k photo`; see them fail.
- Edit the two paragraphs; rerun `uv run --frozen pytest tests/test_prompt.py -q && uv run --frozen poe lint`.
- Commit `docs(claude_agent): prompt states photos travel with maps`.

## Notes

- Keep the existing consent rules (clear the person's POIs or zones only when asked, delete_map only after confirmation).
- New agent session deletes nothing (unchanged, driving-agent new-session-keeps-pois).
