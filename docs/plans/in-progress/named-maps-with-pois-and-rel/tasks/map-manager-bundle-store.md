---
title: Map manager bundle store
status: todo
repo: ros2-lerobot-sverve-platform
value: "Named maps exist on disk as self-contained bundles (pose graph, grid, POIs, last pose) that are written atomically and can be listed and deleted."
files: [nodes/map_manager/pyproject.toml, nodes/map_manager/uv.lock, nodes/map_manager/map_manager/__init__.py, nodes/map_manager/map_manager/bundle.py, nodes/map_manager/tests/__init__.py, nodes/map_manager/tests/test_bundle.py]
verify: "cd nodes/map_manager && uv run --frozen pytest tests/test_bundle.py -q && uv run --frozen poe lint"
depends_on: [map-manager-contract, shared-scan-agreement]
owns: [map-bundle-store]
uses: [map-manager-contract, scan-agreement]
model: sonnet
estimate: 70
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Map manager bundle store

## Requirement

Create the `nodes/map_manager` Python package (uv project like `nodes/poi_store`: hatchling, ruff 0.15.20, vulture,
poethepoet `lint`/`lint-fix`/`test` tasks, pytest `pythonpath = ["."]`; dependencies `numpy`, `pydantic>=2.5`,
`pyyaml`, `ros2-common` and `ros2-metrics` as editable path dependencies; numpy is explicit because the node imports
sensor_msgs later) and its bundle store `map_manager/bundle.py` (design D2), no ROS imports.

- `BundleStore(root: Path)` with root `/var/lib/ros2/maps/named` in production.
- `begin(name) -> Path`: a fresh temp directory `root/.tmp-<name>-<unix ms>/` (the caller tells slam_toolbox to
  serialize to `<tmp>/map`).
- `commit(tmp, name, bundle: BundleMeta, grid: GridMap, overwrite: bool) -> MapSummary`: writes `occupancy.npz`
  (int8 cells with -1 kept, resolution, origin x/y/yaw) and `bundle.json` into tmp, checks `map.posegraph` and
  `map.data` exist there, then renames tmp to `root/<name>`. An existing name without overwrite raises
  `BundleExists`; with overwrite the old directory is renamed aside, the new one moved in, the old one removed. Any
  failure removes tmp and leaves an existing bundle untouched.
- `BundleMeta` (pydantic, `version: Literal[1]`, `name`, `saved_at`, `note`, `last_pose: Pose2D`, `pois: list[dict]`,
  `poi_revision`, `map_info` {resolution, width, height, origin_x, origin_y, origin_yaw}, `laser_frame`).
- `load(name) -> LoadedBundle` (meta, GridMap from occupancy.npz, posegraph base path `root/<name>/map`); a missing or
  invalid file raises `BundleInvalid` naming it.
- `list() -> list[MapSummary]` sorted by name; temp directories are ignored; a directory with an unreadable or
  invalid bundle.json is listed with `valid: false` and the error.
- `delete(name)`: removes the directory; unknown name raises `BundleNotFound`.
- `cleanup_temp()`: removes leftover `.tmp-*` directories (called at node start).
- Names go through `ros2_common.maps.validate_map_name`; the resolved path must stay directly inside root.
- `tests/test_bundle.py` (test first) covers commit, overwrite refusal and swap, failure leaves nothing under the final
  name, occupancy round trip including unknown cells, list with an invalid bundle, delete, temp cleanup, path
  traversal refusal.

## Acceptance conditions

- a1: commit writes all four files and renames atomically; a failure leaves no bundle under the final name [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_bundle.py -q]
- a2: an existing name is refused without overwrite and swapped with overwrite [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_bundle.py -q -k overwrite]
- a3: occupancy.npz round-trips cells including -1 and the origin [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_bundle.py -q -k occupancy]
- a4: list reports an invalid bundle with valid false and its error [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_bundle.py -q -k invalid]
- a5: the package declares numpy and ros2-common and its lint passes [evidence: command: cd nodes/map_manager && grep -q numpy pyproject.toml && grep -q ros2-common pyproject.toml && uv run --frozen poe lint]

## Files

- `nodes/map_manager/pyproject.toml`
- `nodes/map_manager/uv.lock`
- `nodes/map_manager/map_manager/__init__.py`
- `nodes/map_manager/map_manager/bundle.py`
- `nodes/map_manager/tests/__init__.py`
- `nodes/map_manager/tests/test_bundle.py`

## Interfaces

- `nodes/map_manager/map_manager/bundle.py`: `BundleStore`, `BundleMeta`, `LoadedBundle`, `BundleExists`, `BundleInvalid`, `BundleNotFound`.

## Expected outcome

A bundle directory with map.posegraph, map.data, occupancy.npz and bundle.json appears only complete under its name; list, load and delete work on it.

## Steps

- Write the test(s) named in the requirement first; run `cd nodes/map_manager && uv run --frozen pytest tests/test_bundle.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `cd nodes/map_manager && uv run --frozen pytest tests/test_bundle.py -q && uv run --frozen poe lint`; expect every test passed and lint clean.
- Commit: `feat(map_manager): named map bundle store`.

## Notes

Run `uv lock` once after writing pyproject.toml so `uv run --frozen` works. Copy the poi_store pyproject layout (ruff, vulture, poe tasks). GridMap comes from `ros2_common.scan_agreement`.
- Servo safety (CLAUDE.md): this task moves no servo.
