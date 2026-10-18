---
title: MCP clear POIs tool
status: done
repo: ros2-lerobot-sverve-platform
value: "The agent can clear POIs in one call, its own by default, keeping zones and the person's POIs unless the person asked."
files: [nodes/mcp_server/mcp_server/perception_tools.py, nodes/mcp_server/mcp_server/poi_client.py, nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/tests/test_clear_pois_tool.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_clear_pois_tool.py tests/test_poi_client.py -q && uv run --frozen poe lint"
depends_on: [mcp-map-tools, poi-store-clear-filters]
owns: []
uses: [poi-clear-filters]
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/named-maps-with-pois-and-rel--mcp-clear-pois-tool
branch: named-maps-with-pois-and-rel--mcp-clear-pois-tool
verified_head: 666a155f557be5548854ce0dbe8cc3a2fbdea63d
run: named-maps-with-pois-and-rel
provides: []
acs_for_audit: []
process_gaps: []
---

# MCP clear POIs tool

## Requirement

`clear_pois(creator="agent", kinds=None, include_zones=False)` (design D7, D11).

- `poi_client.PoiRequests.clear(created_by: str | None, statuses=None, kinds=None, keep_zones=False)`: sends only the
  given fields; `created_by=None` omits it (every creator). The existing callers (the admin clear route) keep
  their payload.
- `ros_iface` passes the new arguments through.
- Tool: `creator` in agent/user/all (all sends no created_by), `kinds` subset of point/area/object, `include_zones`
  false sends `keep_zones: true`. Description: clearing the person's POIs (`user`/`all`) or zones
  (`include_zones=True`) only when the person asked for it; photos are never cleared. Returns `{"removed": n}`.
- `tests/test_clear_pois_tool.py` (test first): default payload (agent, keep_zones true), all/user, kinds, include
  zones, the admin New-session payload unchanged, poi_store down.

## Acceptance conditions

- a1: clear_pois defaults to the agent's POIs and keeps zones [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_clear_pois_tool.py -q -k default]
- a2: creator all, user, kinds and include_zones map to the poi_store filters [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_clear_pois_tool.py -q]
- a3: the New-session admin clear payload is unchanged [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_client.py -q]

## Files

- `nodes/mcp_server/mcp_server/perception_tools.py`
- `nodes/mcp_server/mcp_server/poi_client.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/tests/test_clear_pois_tool.py`

## Interfaces

- `clear_pois(creator, kinds, include_zones)` in `nodes/mcp_server/mcp_server/perception_tools.py`; `PoiRequests.clear(created_by, statuses, kinds, keep_zones)`.

## Expected outcome

The agent clears its own POIs in one call, zones kept, and can clear more only by explicit arguments.

## Steps

- Write the test(s) named in the requirement first; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_clear_pois_tool.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/mcp_server && uv run --frozen pytest tests/test_clear_pois_tool.py tests/test_poi_client.py -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(mcp_server): clear_pois tool`.

## Notes

The admin route `/admin/clear_agent_pois` (tools.py) keeps its payload; `test_poi_client.py` guards it.
- Servo safety (CLAUDE.md): this task moves no servo.
