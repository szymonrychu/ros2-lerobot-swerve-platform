---
title: Shared POI category tags
status: todo
repo: ros2-lerobot-sverve-platform
value: "Every node reads and writes the POI categories the same way, without a new schema field."
files: [shared/ros2_common/poi_categories.py, tests/test_shared_poi_categories.py, shared/README.md, tests/README.md]
verify: "uv run pytest tests/test_shared_poi_categories.py tests/test_shared_package.py -q && uv run poe lint"
depends_on: []
owns: [poi-category-tags]
uses: []
model: sonnet
estimate: 50
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Shared POI category tags

## Requirement

- New `shared/ros2_common/poi_categories.py` (D4, U2), the one definition of the POI category grammar carried in the existing POI `note` field. Constants at the top: `CATEGORIES = ("target", "slow", "keepout", "problem", "detail")`; `PROBLEM_TYPES = ("window", "mirror", "glass", "tight", "overhang", "low_obstacle", "drop", "cable", "soft_floor", "other")` (overhang and low_obstacle are obstacles above or below the lidar plane); `SURVEY_TAG = "#survey"`; `SURVEY_SPOT_TAG = "#survey-spot"`; `MAX_NOTE_LEN = 2000` (poi_store's limit).
- Tag grammar: the first line of the note holds space-separated tags: `#<category>` for target, slow, keepout and detail, `#problem/<type>` for a problem spot, plus optional `#survey` and `#survey-spot`. The rest of the note is free text.
- Pydantic model `PoiTags` (frozen): `category: str | None`, `problem_type: str | None`, `survey: bool`, `survey_spot: bool`.
- Functions (module level, typed, docstrings with types): `category_tags(category: str | None, problem_type: str | None = None, survey: bool = False, survey_spot: bool = False) -> list[str]`; `parse_tags(note: str) -> PoiTags` (reads only the first line; unknown `#words` are ignored; a first line without any known tag yields an empty PoiTags); `apply_tags(note: str, tags: list[str]) -> str` (replaces the category/problem tags of an existing tag line, keeps `#survey`/`#survey-spot` already present, never duplicates a tag, keeps the free text; raises ValueError naming the length when the result exceeds MAX_NOTE_LEN); `strip_tags(note: str) -> str` (free text only); `has_tag(note: str, tag: str) -> bool`; `check_category(category: str, kind: str, zone: str, problem_type: str | None, speed_limit_pct: int | None) -> None` raising ValueError with an explicit message per rule: target = point, object or area with zone none; slow = area with zone slow and speed_limit_pct; keepout = area with zone keepout; problem = point or area with zone none or slow and a problem_type from PROBLEM_TYPES; detail = point or area with zone none; problem_type given for another category is an error; an unknown category or problem type lists the valid ones.
- `shared/README.md`: a `ros2_common.poi_categories` row in the Modules table and a "POI categories" section with the mapping table (category -> kind, zone, required fields, tag), the tag grammar with examples, and the rule that creating a keep-out is free while cancelling or deleting one needs the person's consent (D4).
- Tests first in new `tests/test_shared_poi_categories.py`, seen failing before the module exists: round trip `parse_tags(apply_tags(note, category_tags(...)))` for every category and every problem type; `apply_tags` on a note that already has tags replaces the category and keeps the survey tags and free text; no duplicate tags after repeated calls; the length error; `check_category` accepts each valid combination and rejects each invalid one with a message naming the rule; `has_tag` and `strip_tags`. `tests/README.md` lists the file and its tests.

## Acceptance conditions

- a1: tags round-trip for every category and problem type [evidence: command: uv run pytest tests/test_shared_poi_categories.py -q -k round_trip]
- a2: check_category accepts the mapping and rejects every invalid combination with a rule message [evidence: command: uv run pytest tests/test_shared_poi_categories.py -q -k check_category]
- a3: apply_tags replaces category tags, keeps survey tags and text, never duplicates, enforces the length [evidence: command: uv run pytest tests/test_shared_poi_categories.py -q -k apply_tags]
- a4: shared README documents the category mapping and keep-out consent rule [evidence: command: grep -q 'poi_categories' shared/README.md && grep -qi 'consent' shared/README.md]
- a5: root lint passes [evidence: command: uv run poe lint]

## Files

- `shared/ros2_common/poi_categories.py`
- `tests/test_shared_poi_categories.py`
- `shared/README.md`
- `tests/README.md`

## Interfaces

- `shared/ros2_common/poi_categories.py`: `CATEGORIES`, `PROBLEM_TYPES`, `SURVEY_TAG`, `SURVEY_SPOT_TAG`, `PoiTags`, `category_tags`, `parse_tags`, `apply_tags`, `strip_tags`, `has_tag`, `check_category` (contract `poi-category-tags`).

## Expected outcome

One importable grammar for POI categories that mcp_server, poi_store and claude_agent share; the web UI mirrors it.

## Steps

- Write `tests/test_shared_poi_categories.py`; run `uv run pytest tests/test_shared_poi_categories.py -q`; expect ImportError failures.
- Implement the module and README; run the verify command; commit `feat(shared): POI category tags`.

## Notes

- poi_store `Poi` has `extra="ignore"`: a new field would be dropped, so categories live in the note (U2).
- Zones: `zone` none|keepout|slow, area only, active only while status open (poi_store/models.py).
- Keep pydantic for PoiTags (CLAUDE.md library rule).
