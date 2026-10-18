---
title: Photo retention age optional
status: todo
repo: ros2-lerobot-sverve-platform
value: A loaded map keeps its older photos instead of losing them to the hourly age prune
files: [nodes/poi_store/poi_store/config.py, nodes/poi_store/poi_store/photo_store.py, nodes/poi_store/tests/test_photo_store.py, nodes/poi_store/tests/test_node_photos.py, nodes/poi_store/tests/test_config.py, ansible/group_vars/client.yml, nodes/poi_store/README.md]
verify: "cd nodes/poi_store && uv run --frozen pytest tests/test_photo_store.py tests/test_node_photos.py tests/test_config.py -q && uv run --frozen poe lint"
depends_on: [photo-store-sync-ops]
owns: []
uses: []
model: ""
estimate: 40
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Photo retention age optional

## Requirement

D10: photos are map POIs restored by a map load, so the age limit of the photo retention must not delete them.

- `nodes/poi_store/poi_store/config.py`: `PhotoSettings.max_age_days` becomes `float | None` (default None; when set it must be > 0 and finite). Docstring says None disables the age limit; `max_records` and `max_bytes` stay as disk guards.
- `nodes/poi_store/poi_store/photo_store.py`: `PhotoStore` accepts `max_age_days: float | None`; with None the prune never drops a record for its age (count and bytes limits still apply); `summary()["limits"]["max_age_days"]` is null then.
- `ansible/group_vars/client.yml` poi_store config: `max_age_days: null` with a comment saying photos are map POIs and leave with their map, count and bytes limits guard the disk.
- Tests first, seen failing: in `nodes/poi_store/tests/test_photo_store.py` a store with `max_age_days=None` keeps a record 30 days old through `prune_now`, while count and bytes limits still prune; in `nodes/poi_store/tests/test_node_photos.py` (or `test_config.py`) the default config has no age limit and the summary reports `max_age_days` null; an explicit positive value still prunes by age (existing tests keep passing with an explicit value).
- `nodes/poi_store/README.md` config and retention sections updated.

## Acceptance conditions

- a1: with no age limit old photos survive the prune while count and bytes limits still apply [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_photo_store.py -q -k age]
- a2: the default config has no age limit and the summary reports it as null [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests/test_node_photos.py tests/test_config.py -q]
- a3: the client config disables the age limit [evidence: command: grep -q 'max_age_days: null' ansible/group_vars/client.yml]
- a4: poi_store suite and lint pass [evidence: command: cd nodes/poi_store && uv run --frozen pytest tests -q && uv run --frozen poe lint]
- a5: ansible lint passes [evidence: command: uv run poe lint-ansible]

## Files

- `nodes/poi_store/poi_store/config.py`
- `nodes/poi_store/poi_store/photo_store.py`
- `nodes/poi_store/tests/test_photo_store.py`
- `nodes/poi_store/tests/test_node_photos.py`
- `nodes/poi_store/tests/test_config.py`
- `ansible/group_vars/client.yml`
- `nodes/poi_store/README.md`

## Interfaces

- `nodes/poi_store/poi_store/config.py:PhotoSettings.max_age_days: float | None` (default None).
- `nodes/poi_store/poi_store/photo_store.py:PhotoStore.__init__(..., max_age_days: float | None, ...)`; `summary()["limits"]["max_age_days"]` null when off.

## Expected outcome

With the deployed config, poi_store never deletes a photo for its age; count and bytes limits still guard the disk.

## Steps

- Add the age tests to `nodes/poi_store/tests/test_photo_store.py` and the default-config test to `tests/test_node_photos.py` or `tests/test_config.py`; run `cd nodes/poi_store && uv run --frozen pytest tests/test_photo_store.py tests/test_node_photos.py tests/test_config.py -q`; see them fail.
- Make `max_age_days` optional in config and PhotoStore; rerun until green; run the whole suite and `uv run --frozen poe lint`.
- Set `max_age_days: null` in the poi_store block of `ansible/group_vars/client.yml`; run `uv run poe lint-ansible` at the root.
- Update the README; commit `feat(poi_store): photo age retention off by default`.

## Notes

- Decision D10: photos are map POIs restored by a load; the hourly age prune would delete a restored map's older photos.
- Existing tests that construct a PhotoStore with `max_age_days=14.0` keep that explicit value.
- Deploy after merge: `./scripts/deploy-nodes.sh client poi_store` (run by the orchestrator at phase 6).
