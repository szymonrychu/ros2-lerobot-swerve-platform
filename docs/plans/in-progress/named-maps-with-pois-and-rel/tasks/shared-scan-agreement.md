---
title: Shared scan agreement
status: done
repo: ros2-lerobot-sverve-platform
value: "map_manager can judge a relocalisation with exactly the scan-to-map agreement metric map_guard already uses on this robot, without a second implementation."
files: [shared/ros2_common/scan_agreement.py, tests/test_shared_scan_agreement.py, nodes/map_guard/map_guard/agreement.py, nodes/map_guard/pyproject.toml, nodes/map_guard/uv.lock, nodes/map_guard/README.md, shared/README.md]
verify: "uv run pytest tests/test_shared_scan_agreement.py tests/test_shared_package.py -q && uv run poe lint && cd nodes/map_guard && uv run --frozen pytest -q && uv run --frozen poe lint && uv run --frozen python ../map_guard_cpp/test/gen_fixtures.py --check"
depends_on: [map-manager-contract]
owns: [scan-agreement]
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/named-maps-with-pois-and-rel--shared-scan-agreement
branch: named-maps-with-pois-and-rel--shared-scan-agreement
verified_head: 0659db4b5a0bbf55469a219ee2de778d18029641
run: named-maps-with-pois-and-rel
provides: [scan-agreement]
acs_for_audit: []
process_gaps: []
---

# Shared scan agreement

## Requirement

Move the pure scan-to-map agreement functions out of `nodes/map_guard/map_guard/agreement.py` into
`shared/ros2_common/scan_agreement.py` (design D6) with unchanged behaviour and signatures: `UNKNOWN`, `GridMap`,
`PreparedMap`, `AgreementResult`, `dilate`, `prepare_map`, `scan_endpoints`, `yaw_quaternion`, `quaternion_matrix`,
`transform_points`, `agreement`. Add `grid_from_arrays(resolution, origin_x, origin_y, origin_yaw, cells) -> GridMap`
only if a consumer needs it (map_manager builds GridMap from occupancy.npz).

- `nodes/map_guard/map_guard/agreement.py` imports these names from `ros2_common.scan_agreement` (so
  `from map_guard.agreement import ...` keeps working for `node.py`, `detectors.py`, `guard.py`, the map_guard tests and
  `nodes/map_guard_cpp/test/gen_fixtures.py`) and keeps `MismatchMonitor` (it depends on map_guard's `HoldTimer`).
- `nodes/map_guard/pyproject.toml` gains `ros2-common` as an editable path dependency
  (`{ path = "../../shared", editable = true }`, the form `tests/test_shared_package.py` checks); `uv.lock` regenerated
  with `uv lock`.
- `tests/test_shared_scan_agreement.py` ports the pure-function cases of `nodes/map_guard/tests/test_agreement.py`
  (room grid, dilation, endpoints, transform, scored scans, no result without a map / with too few points) against the
  shared module. Test first: written and seen failing before the module exists.
- map_guard's own tests, its lint and `gen_fixtures.py --check` (golden fixtures of map_guard_cpp) stay green with no
  fixture regenerated.
- `shared/README.md` gains a `## Scan agreement` section; `nodes/map_guard/README.md` says where the functions now live.

## Acceptance conditions

- a1: the shared module reproduces the agreement results of the former map_guard functions on the room fixtures [evidence: test: tests/test_shared_scan_agreement.py]
- a2: map_guard tests and lint pass unchanged after the move [evidence: command: cd nodes/map_guard && uv run --frozen pytest -q && uv run --frozen poe lint]
- a3: the map_guard_cpp golden fixtures are not stale [evidence: command: cd nodes/map_guard && uv run --frozen python ../map_guard_cpp/test/gen_fixtures.py --check]
- a4: map_guard depends on ros2-common by the relative editable path [evidence: test: tests/test_shared_package.py]
- a5: shared/README.md documents the module [evidence: doc-section: shared/README.md#Scan agreement]

## Files

- `shared/ros2_common/scan_agreement.py`
- `tests/test_shared_scan_agreement.py`
- `nodes/map_guard/map_guard/agreement.py`
- `nodes/map_guard/pyproject.toml`
- `nodes/map_guard/uv.lock`
- `nodes/map_guard/README.md`
- `shared/README.md`

## Interfaces

- `shared/ros2_common/scan_agreement.py`: `GridMap`, `PreparedMap`, `AgreementResult`, `dilate`, `prepare_map`, `scan_endpoints`, `yaw_quaternion`, `quaternion_matrix`, `transform_points`, `agreement` (signatures of `nodes/map_guard/map_guard/agreement.py` today).
- `nodes/map_guard/map_guard/agreement.py` keeps `MismatchMonitor` and re-exports the moved names.

## Expected outcome

One agreement implementation in shared/, used by map_guard unchanged and available to map_manager; map_guard_cpp fixtures stay current.

## Steps

- Write the test(s) named in the requirement first; run `uv run pytest tests/test_shared_scan_agreement.py -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `uv run pytest tests/test_shared_scan_agreement.py tests/test_shared_package.py -q && uv run poe lint && cd nodes/map_guard && uv run --frozen pytest -q && uv run --frozen poe lint && uv run --frozen python ../map_guard_cpp/test/gen_fixtures.py --check`; expect every test passed and lint clean.
- Commit: `refactor(shared): move scan-to-map agreement into ros2_common`.

## Notes

`nodes/map_guard_cpp/test/gen_fixtures.py` imports `map_guard.agreement` and `tests.test_agreement` helpers; the re-export keeps it working. Do not regenerate fixtures: `--check` must pass as is.
- Servo safety (CLAUDE.md): this task moves no servo.
