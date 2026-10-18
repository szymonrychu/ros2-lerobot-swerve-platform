---
title: MCP exploration status tool
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The agent knows when the surroundings are completely mapped and surveyed, and when to stop for the battery."
files: [nodes/mcp_server/mcp_server/frontier_tools.py, nodes/mcp_server/mcp_server/perception_models.py, nodes/mcp_server/tests/test_exploration_tools.py, nodes/mcp_server/README.md, nodes/claude_agent/claude_agent/config.py, ansible/group_vars/client.yml]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration_tools.py tests/test_frontier_tools.py -q && uv run --frozen poe lint && cd ../claude_agent && uv run --frozen pytest tests/test_config.py -q && cd ../.. && uv run pytest tests/test_claude_agent_config.py tests/test_mcp_server_config.py -q && uv run poe lint-ansible"
depends_on: [mcp-exploration-coverage, mcp-poi-category-args, shared-poi-category-tags, costmap-slow-zone-cost]
owns: [exploration-status]
uses: [exploration-coverage, poi-category-tags]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/driving-agent-skills-and-wor--mcp-exploration-status-tool
branch: driving-agent-skills-and-wor--mcp-exploration-status-tool
verified_head: ""
run: ""
provides: []
---

# MCP exploration status tool

## Requirement

- `frontier_tools.py` (D6, U8) registers two tools:
  - `mark_survey_spot(note: str = "", x: float | None = None, y: float | None = None)`: adds an agent point POI named "survey spot" at the robot pose (or x, y; error without a pose and without x/y) with note tags `#survey #survey-spot` (shared `category_tags`/`apply_tags`) plus the note text; no motion; result = the poi_store answer.
  - `get_exploration_status()`: refreshes the frontier snapshot, computes reachable frontiers (reachable true) with id, path_distance_m and looked (last_lookup_facing not null), counts reachable/unreachable/unlooked, reads survey spots = open agent POIs with `#survey-spot` (shared `has_tag`), computes `exploration.coverage` on the reachable mask, and battery = {pack_v, stop_v = cells * (cutoff_cell_v + exploration.stop_margin_cell_v), limit_reached} from the battery guard the motion gates use (null fields with a note when no guard or no reading). `complete` = robot pose known AND no reachable frontier AND no unsurveyed area; `reasons` lists every unmet condition in words. Null with a note (never a guess) when /map or the pose is unavailable. No motion, no lease.
- `perception_models.py`: `ExplorationStatus`, `ExplorationFrontier`, `BatteryLimit` models with field descriptions; tool descriptions explain completion, coverage and the battery limit.
- `nodes/claude_agent/claude_agent/config.py` `DEFAULT_SENSOR_TOOLS` and `ansible/group_vars/client.yml` claude_agent `sensor_tools` gain both tools (every registered tool classified exactly once); client.yml mcp_server config gets an `exploration:` block with the defaults and comments (survey_radius_m, stop_margin_cell_v).
- `nodes/mcp_server/README.md`: the two tools, the ExplorationSettings table and the completion rule.
- Tests first in new `tests/test_exploration_tools.py` (fakes: map, frontier tracker, poi list, battery guard), seen failing: complete true only with no reachable frontier and full coverage; reasons name remaining frontiers and unsurveyed areas; battery limit flips below stop_v; null + note without map or pose; mark_survey_spot writes the tags at the robot pose and errors without a pose and coordinates; only `#survey-spot` POIs count as spots.

## Acceptance conditions

- a1: complete is true only with no reachable frontier and no unsurveyed area, with reasons otherwise [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration_tools.py -q -k complete]
- a2: battery limit_reached flips below the stop voltage [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration_tools.py -q -k battery]
- a3: null values with a note when map or pose is missing [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration_tools.py -q -k unavailable]
- a4: mark_survey_spot writes survey tags at the robot pose [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_exploration_tools.py -q -k mark_survey_spot]
- a5: both tools are sensor-classified in code defaults and client.yml [evidence: command: uv run pytest tests/test_claude_agent_config.py -q]
- a6: mcp_server frontier tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_frontier_tools.py -q && uv run --frozen poe lint]
- a7: README documents the tools and the completion rule [evidence: command: grep -q 'get_exploration_status' nodes/mcp_server/README.md && grep -q 'mark_survey_spot' nodes/mcp_server/README.md]

## Files

- `nodes/mcp_server/mcp_server/frontier_tools.py`
- `nodes/mcp_server/mcp_server/perception_models.py`
- `nodes/mcp_server/tests/test_exploration_tools.py`
- `nodes/mcp_server/README.md`
- `nodes/claude_agent/claude_agent/config.py`
- `ansible/group_vars/client.yml`

## Interfaces

- Tools `get_exploration_status`, `mark_survey_spot`; `perception_models.py:ExplorationStatus` (contract `exploration-status`).

## Expected outcome

The agent can mark survey spots and ask whether exploration is complete, with coverage, gaps and the battery limit.

## Steps

- Write `tests/test_exploration_tools.py`; run it; expect failures.
- Implement, classify in claude_agent config and client.yml; run the verify command; commit `feat(mcp_server): exploration status and survey spots`.

## Notes

- Root `tests/test_claude_agent_config.py` fails if a registered tool is unclassified: classify in the same commit.
- Battery guard: shared ros2_common.battery.BatteryGuard behind ctx.battery_gate.
