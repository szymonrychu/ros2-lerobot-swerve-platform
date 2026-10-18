---
title: Relocalise requires a minimum evaluated scan fraction
status: todo
repo: ros2-lerobot-sverve-platform
value: "A partially overlapping match can no longer be accepted as the robot's pose"
files: [nodes/map_manager/map_manager/relocalise.py, nodes/map_manager/map_manager/config.py, nodes/map_manager/tests/test_relocalise.py, nodes/map_manager/tests/test_config.py, nodes/map_manager/README.md]
verify: "cd nodes/map_manager && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: []
owns: []
uses: []
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Relocalise requires a minimum evaluated scan fraction

## Requirement

`map_manager/relocalise.py` `search_candidates` (and the verdict it feeds) must not report a confident pose that was scored on only part of the scan. Discovered on the live robot: a hint outside the candidate window returned a wrong pose with ratio 1.00 over 126 of ~330 scan points and no ambiguity flag; only the later agreement check guards against it.

- Add a config key `min_evaluated_fraction` (pydantic, 0 < x <= 1, default 0.6) to MapManagerConfig (the deployed config may omit it; the default applies): a candidate is only eligible when the fraction of valid scan points that fell inside the map grid (evaluated / total valid) is at least this value.
- A search whose best candidate is below the fraction reports not-ok with a reason naming the fraction (`evaluated 126/330 = 0.38 < 0.60`), so load/relocalise ends "not localised" instead of accepting the pose.
- Tests first (seen failing): a synthetic grid + scan where the true pose is far outside the hint window and a partial-overlap wrong pose scores 1.00 on a minority of points -> rejected; a normal case still localises; config validation rejects 0 and >1.
- README (map_manager) documents the key and the failure reason.

## Acceptance conditions

- a1: a candidate scored on fewer than min_evaluated_fraction of the valid scan points is rejected with a reason naming the fraction [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q]
- a2: min_evaluated_fraction is a validated config key (default 0.6) [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_config.py -q]
- a3: nodes/map_manager/README.md documents min_evaluated_fraction [evidence: command: grep -q min_evaluated_fraction nodes/map_manager/README.md]

## Files

- `nodes/map_manager/map_manager/relocalise.py`
- `nodes/map_manager/map_manager/config.py`
- `nodes/map_manager/tests/test_relocalise.py`
- `nodes/map_manager/tests/test_config.py`
- `nodes/map_manager/README.md`

## Interfaces

- `map_manager/config.py:MapManagerConfig.min_evaluated_fraction: float` (0 < x <= 1, default 0.6); `map_manager/relocalise.py:search_candidates` result carries evaluated/total and rejects below the fraction.

## Expected outcome

- A partial-overlap candidate (evaluated fraction below the threshold) never yields ok; load/relocalise ends not localised with a reason naming the fraction.

## Steps

- Add the failing tests in tests/test_relocalise.py and tests/test_config.py; `cd nodes/map_manager && uv run --frozen pytest tests -q` shows them failing.
- Implement the config key and the eligibility check; the same command passes; `uv run --frozen poe lint` clean.
- Document the key and reason in nodes/map_manager/README.md; commit `fix(map_manager): require a minimum evaluated scan fraction`.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
