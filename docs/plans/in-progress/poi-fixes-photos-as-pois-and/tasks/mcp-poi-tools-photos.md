---
title: MCP POI tools photos
status: done
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
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/poi-fixes-photos-as-pois-and--mcp-poi-tools-photos
branch: poi-fixes-photos-as-pois-and--mcp-poi-tools-photos
verified_head: 8911ab0f76db0251943bc57805c30607736199dc
run: poi-fixes-photos-as-pois-and
provides: []
acs_for_audit: []
process_gaps: []
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

- `nodes/mcp_server/mcp_server/perception_models.py:ClearKind` with "photo".
- `clear_pois` returns `{"removed": int, "pois": int, "photos": int}`.
- `nodes/mcp_server/mcp_server/poi_client.py:describe_pois` skips photo and position-less entries.

## Expected outcome

The agent can clear photos (its own by default), sees both counts, and `list_pois` keeps working when `/poi/list` holds photos.

## Steps

- Write `nodes/mcp_server/tests/test_poi_photo_tools.py` and update `tests/test_clear_pois_tool.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_poi_photo_tools.py tests/test_clear_pois_tool.py -q`; see them fail (describe_pois raises on null x, clear returns one count).
- Implement; rerun; run `uv run --frozen pytest tests/test_poi_category_tools.py -q && uv run --frozen poe lint`.
- Update the README; commit `feat(mcp_server): POI tools handle photo POIs`.

## Notes

- Built on driving-agent mcp-poi-category-args (category args in `add_poi`/`update_poi`, PoiView fields); do not redo them.
- `list_lookups` stays the photo listing tool.
- Open question Q2 covers whether the default clear includes the agent's photos; the plan follows the recommended answer (yes).
