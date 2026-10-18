---
title: Maps contract photos clear
status: done
repo: ros2-lerobot-sverve-platform
value: Map summaries can show photo counts and the web UI can ask map_manager to clear the live set
files: [shared/ros2_common/maps.py, tests/test_shared_maps.py, tests/README.md, shared/README.md]
verify: "uv run pytest tests/test_shared_maps.py -q && uv run poe lint"
depends_on: []
owns: [map-contract-photos-clear-live]
uses: []
model: ""
estimate: 30
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/poi-fixes-photos-as-pois-and--maps-contract-photos-clear
branch: poi-fixes-photos-as-pois-and--maps-contract-photos-clear
verified_head: 7cb397bb24f71c367f4784b10c7e26457f56a103
run: poi-fixes-photos-as-pois-and
provides: []
acs_for_audit: []
process_gaps: []
---

# Maps contract photos clear

## Requirement

Foundation for D3 and D6: the shared map_manager contract carries photo counts and the `clear_live` op.

- `shared/ros2_common/maps.py`:
  - `MapOp` gains "clear_live". `MapCommand.check_op_fields`: `clear_live` takes no name (like relocalise), no pose, note, overwrite, since or discard_on_fail; the existing error texts for relocalise stay and a `clear_live` with a name is refused with "clear_live takes no map name".
  - `MapSummary` gains `photo_count: int = Field(default=0, ge=0)`; the docstring says `poi_count` counts the non-photo POIs and `photo_count` the photos of the bundle (0 for bundles saved before photos were bundled).
- Tests first, seen failing, in `tests/test_shared_maps.py`: a `clear_live` command validates without a name and is refused with a name, a pose, a note, overwrite, since or discard_on_fail; `MapSummary` without `photo_count` parses to 0 (old status payloads) and a negative value is refused; the existing contract tests keep passing.
- `shared/README.md` (Named maps contract section) documents `clear_live` and `photo_count`; `tests/README.md` lists the new tests under `test_shared_maps.py`.

## Acceptance conditions

- a1: clear_live validates without a name and refuses every other field [evidence: command: uv run pytest tests/test_shared_maps.py -q -k clear_live]
- a2: MapSummary photo_count defaults to 0 and refuses negatives [evidence: command: uv run pytest tests/test_shared_maps.py -q -k photo_count]
- a3: root tests for the shared contract and root lint pass [evidence: command: uv run pytest tests/test_shared_maps.py -q && uv run poe lint]
- a4: shared README documents clear_live and photo_count [evidence: command: grep -q 'clear_live' shared/README.md && grep -q 'photo_count' shared/README.md]

## Files

- `shared/ros2_common/maps.py`
- `tests/test_shared_maps.py`
- `tests/README.md`
- `shared/README.md`

## Interfaces

- `shared/ros2_common/maps.py:MapOp` with "clear_live"; `MapCommand.check_op_fields` (part `map-contract-photos-clear-live`).
- `shared/ros2_common/maps.py:MapSummary.photo_count: int = 0`.

## Expected outcome

map_manager, web_ui and the frontend share one contract for the photo count of a saved map and the `clear_live` command.

## Steps

- Add the tests to `tests/test_shared_maps.py`; run `uv run pytest tests/test_shared_maps.py -q`; see them fail.
- Implement the op and the field; rerun until green; run `uv run poe lint`.
- Update `shared/README.md` and `tests/README.md`; commit `feat(shared): map contract photo_count and clear_live`.

## Notes

- `NAMED_OPS` stays ("save", "load", "delete"); `clear_live` behaves like relocalise for the name rule.
- Old `/map_manager/status` payloads without `photo_count` must still parse (default 0).
