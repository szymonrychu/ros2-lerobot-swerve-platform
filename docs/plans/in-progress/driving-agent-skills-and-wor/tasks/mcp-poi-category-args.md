---
title: MCP POI category args
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The agent records targets, slow zones, keep-outs, problem spots and details consistently, and sees them back."
files: [nodes/mcp_server/mcp_server/perception_tools.py, nodes/mcp_server/mcp_server/perception_models.py, nodes/mcp_server/tests/test_poi_category_tools.py, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_category_tools.py tests/test_perception_tools.py tests/test_object_memory.py -q && uv run --frozen poe lint"
depends_on: [shared-poi-category-tags]
owns: []
uses: [poi-category-tags]
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

# MCP POI category args

## Requirement

- `perception_tools.py` (D4, U2): `add_poi` and `update_poi` take `category` (Literal of CATEGORIES, optional), `problem_type` (Literal of PROBLEM_TYPES, optional) and `survey` (bool, default False). With a category: `check_category` (shared) runs on the resulting kind/zone/speed_limit_pct (update: on the POI's current values merged with the change, read from the store) and the note is rewritten with `apply_tags` + `category_tags`; `survey=true` adds `#survey`. For category keepout/slow the caller still passes zone and speed_limit_pct explicitly (no implicit zone). Errors are ValueError text naming the rule. `remember_object` takes `survey: bool` (adds `#survey`, category target).
- `list_pois`: each `PoiView` (perception_models.py) gets `category`, `problem_type`, `survey` parsed with `parse_tags` (null/false when untagged).
- `ZONE_HELP` (D5): slow zones make the planner prefer routes around them (non-lethal path cost) and cap the speed inside; keepout unchanged. The tool descriptions name the categories and the mapping in one sentence each, and say: creating a keep-out is free; cancelling or deleting one needs the person's consent.
- `README.md`: a "POI categories" subsection under the POI tools (args, validation, list_pois fields, link to shared/README.md).
- Tests first in new `tests/test_poi_category_tools.py` (existing fakes for the robot/poi_store), seen failing: add_poi with each category writes the right tags and zone; invalid combinations (keepout on a point, slow without speed_limit_pct, problem without problem_type, problem_type with target) are rejected before any poi_store request; update_poi changing the category keeps the free text and the `#survey` tag; survey=true adds `#survey`; remember_object survey tag; list_pois returns category/problem_type/survey.

## Acceptance conditions

- a1: add_poi writes category tags and validates the kind/zone combination [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_category_tools.py -q -k add_poi]
- a2: update_poi re-tags without losing free text or the survey tag [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_category_tools.py -q -k update_poi]
- a3: invalid combinations are refused before any poi_store request [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_category_tools.py -q -k invalid]
- a4: list_pois exposes category, problem_type and survey [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_category_tools.py -q -k list_pois]
- a5: existing perception and object tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_perception_tools.py tests/test_object_memory.py -q && uv run --frozen poe lint]
- a6: README documents POI categories [evidence: command: grep -q 'POI categories' nodes/mcp_server/README.md]

## Files

- `nodes/mcp_server/mcp_server/perception_tools.py`
- `nodes/mcp_server/mcp_server/perception_models.py`
- `nodes/mcp_server/tests/test_poi_category_tools.py`
- `nodes/mcp_server/README.md`

## Interfaces

- `perception_tools.py:add_poi/update_poi(category, problem_type, survey)`, `remember_object(survey)`; `perception_models.py:PoiView.category/problem_type/survey`.

## Expected outcome

POI writes carry a validated category in the note; list_pois shows it.

## Steps

- Write `tests/test_poi_category_tools.py`; run it; expect failures.
- Implement; run the verify command; commit `feat(mcp_server): POI category arguments`.

## Notes

- No new tool is registered, so claude_agent classification is unchanged.
- mcp_server depends on ros2-common already (ros2_common.lookups).
