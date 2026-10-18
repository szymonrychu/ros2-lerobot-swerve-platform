---
title: MCP POI tools photos
status: todo
repo: ros2-lerobot-sverve-platform
value: "The agent can clear photos with its POIs, sees both counts, and list_pois never breaks on photos"
files: [nodes/mcp_server/mcp_server/perception_models.py, nodes/mcp_server/mcp_server/poi_client.py, nodes/mcp_server/mcp_server/perception_tools.py, nodes/mcp_server/tests/test_poi_photo_tools.py, nodes/mcp_server/tests/test_clear_pois_tool.py, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_photo_tools.py tests/test_clear_pois_tool.py -q && uv run --frozen poe lint"
depends_on: [poi-cleanups-delete-photos]
owns: []
uses: [poi-clear-photo-kind, poi-list-photo-entries]
model: ""
estimate: 55
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# MCP POI tools photos

## Requirement

D2, D7, D9 (mcp_server): the agent's POI tools handle photo POIs: clear can select them, list_pois stays photo-free, results report both counts.

- `nodes/mcp_server/mcp_server/perception_models.py`: `ClearKind` adds "photo".
- `nodes/mcp_server/mcp_server/poi_client.py`: `clear` docstring covers photos; `describe_pois` skips entries of kind "photo" and any entry whose x or y is not a number instead of raising.
- `nodes/mcp_server/mcp_server/perception_tools.py`:
  - `list_pois` filters out kind "photo" (photos are listed by `list_lookups`); its description keeps saying so.
  - `clear_pois` returns `{"removed": n, "pois": a, "photos": b}` from poi_store's result data; the description says kinds is a subset of point/area/object/photo, the default (every kind) includes your own photos, the person's photos only when asked, and photos removed lose their images.
  - `delete_poi` description: a photo id deletes that photo and its image; deleting a POI keeps the photos linked to it.
- Tests first, seen failing, in new `nodes/mcp_server/tests/test_poi_photo_tools.py` (existing fakes): `clear_pois(kinds=["photo"])` sends kinds ["photo"] and returns both counts; the default clear sends no kinds and returns both counts; `list_pois` with photo entries in the store (one with null x/y) returns only non-photo POIs without raising; `describe_pois` skips a photo entry. Update `tests/test_clear_pois_tool.py` (its description assertion no longer expects "photos are never cleared").
- `nodes/mcp_server/README.md`: the `clear_pois`, `delete_poi` and `list_pois` rows.

## Acceptance conditions

- a1: clear_pois can select photos and returns POI and photo counts [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_photo_tools.py -q -k clear]
- a2: list_pois and describe_pois tolerate photo entries and keep them out [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_photo_tools.py -q -k 'list or describe']
- a3: existing clear and category tool tests and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests/test_clear_pois_tool.py tests/test_poi_category_tools.py -q && uv run --frozen poe lint]
- a4: README documents clearing photos [evidence: command: grep -q 'point/area/object/photo' nodes/mcp_server/README.md]

## Files

- `nodes/mcp_server/mcp_server/perception_models.py`
- `nodes/mcp_server/mcp_server/poi_client.py`
- `nodes/mcp_server/mcp_server/perception_tools.py`
- `nodes/mcp_server/tests/test_poi_photo_tools.py`
- `nodes/mcp_server/tests/test_clear_pois_tool.py`
- `nodes/mcp_server/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
