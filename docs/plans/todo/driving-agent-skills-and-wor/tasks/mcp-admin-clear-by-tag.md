---
title: MCP admin clear by tag
status: todo
repo: ros2-lerobot-sverve-platform
value: claude_agent can ask mcp_server to clear only the survey POIs on New session.
files: [nodes/mcp_server/mcp_server/tools.py, nodes/mcp_server/mcp_server/tool_context.py, nodes/mcp_server/mcp_server/ros_iface.py, nodes/mcp_server/tests/fakes.py, nodes/mcp_server/tests/test_admin_clear.py, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_admin_clear.py tests/test_tools.py -q && uv run --frozen poe lint"
depends_on: [poi-store-clear-by-tag, mcp-poi-category-args]
owns: [admin-clear-tag]
uses: []
model: sonnet
estimate: 40
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# MCP admin clear by tag

## Requirement

- `tools.py` admin route `/admin/clear_agent_pois` (D11, U9): body `{"statuses": [...]?, "tag": "#..."?}` (both optional, either may be given; an empty body keeps clearing every status); `tag` validated (starts with `#`, no whitespace) -> 400 with the expected body shape otherwise; passes `tag` to `robot.poi_clear(created_by, statuses, tag)`.
- `tool_context.py` and `ros_iface.py` `poi_clear(created_by, statuses=None, tag=None)` put `tag` into the /poi/command clear message only when given; `tests/fakes.py` fake mirrors the signature and records the tag.
- `README.md`: the admin route documents `tag`.
- Tests first in new `tests/test_admin_clear.py` (move nothing out of test_tools.py; extend there only if a shared fixture requires it), seen failing: a tag body reaches poi_clear with the tag; statuses + tag both pass; a bad tag is 400; no token is 401; the existing statuses-only behaviour is unchanged.

## Acceptance conditions

- a1: the admin route forwards a tag filter to poi_clear [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_admin_clear.py -q -k tag]
- a2: a malformed tag gets 400 and a missing token 401 [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_admin_clear.py -q -k 'bad_tag or unauthorized']
- a3: existing admin and tools tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_tools.py -q && uv run --frozen poe lint]
- a4: README documents the tag body [evidence: command: grep -q 'clear_agent_pois' nodes/mcp_server/README.md && grep -q '"tag"' nodes/mcp_server/README.md]

## Files

- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/mcp_server/tool_context.py`
- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/tests/fakes.py`
- `nodes/mcp_server/tests/test_admin_clear.py`
- `nodes/mcp_server/README.md`

## Interfaces

- Admin route body `{statuses?, tag?}` (contract `admin-clear-tag`); `RobotApi.poi_clear(created_by, statuses, tag)`.

## Expected outcome

The admin route can clear agent POIs by tag.

## Steps

- Write `tests/test_admin_clear.py`; run it; expect failures.
- Implement; run the verify command; commit `feat(mcp_server): admin clear by tag`.

## Notes

- The admin route is plain HTTP behind the bearer token, never an MCP tool (the model cannot call it).
- Epic named-maps-with-pois-and-rel (todo, task mcp-clear-pois-tool) touches ros_iface.py poi_clear too: keep `tag` an independent keyword argument.
