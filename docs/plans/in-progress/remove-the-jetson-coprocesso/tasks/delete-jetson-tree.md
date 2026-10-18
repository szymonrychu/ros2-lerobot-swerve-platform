---
title: Delete jetson tree
status: done
repo: ros2-lerobot-sverve-platform
value: "The Jetson-only depthd, calibration and model code and its end-to-end check leave the repo, pinned by a retirement test."
files: [jetson/, scripts/jetson_e2e_check.py, scripts/jetson_e2e_check.yaml, tests/test_jetson_e2e_check.py, tests/test_jetson_retired.py, scripts/README.md, ansible/playbooks/optimize.yml, tests/README.md]
verify: "uv run pytest tests/test_jetson_retired.py -q && uv run poe lint"
depends_on: [remove-jetson-deploy-target]
owns: [jetson-retired-test]
uses: []
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/remove-the-jetson-coprocesso--delete-jetson-tree
branch: remove-the-jetson-coprocesso--delete-jetson-tree
verified_head: 454b2bdd15fa61073ceae16eb344d18421fdf75c
run: remove-the-jetson-coprocesso
provides: [jetson-retired-test]
acs_for_audit: [a5]
process_gaps: ["The implementer's report lists no files changed and no red/green evidence (tests.red, verification.evidence). The independent verification flagged this as a high finding."]
---

# Delete jetson tree

## Requirement

- New `tests/test_jetson_retired.py`, written first and seen failing: module constants `REMOVED_PATHS` (repo-relative paths that must not exist) and `ABSENT_TOKENS` (`(path, token)` pairs: the file must not contain the token, case-sensitive), and exactly two parametrized tests, `test_removed_path_is_gone(path)` and `test_removed_token_is_absent(path, token)`. Later tasks of this epic extend only these two lists (plus the extra tests named in their own requirements).
- Initial `REMOVED_PATHS`: `jetson`, `scripts/jetson_e2e_check.py`, `scripts/jetson_e2e_check.yaml`, `tests/test_jetson_e2e_check.py`. Initial `ABSENT_TOKENS`: (`scripts/README.md`, `jetson_e2e_check`), (`ansible/playbooks/optimize.yml`, `jetson`).
- `jetson/` (all tracked files: calibrate, depthd, models), `scripts/jetson_e2e_check.py`, `scripts/jetson_e2e_check.yaml` and `tests/test_jetson_e2e_check.py` are deleted with git rm.
- `scripts/README.md` has no `jetson_e2e_check.py` section.
- The `ansible/playbooks/optimize.yml` header comment no longer mentions the jetson group (it targets the Raspberry Pis).
- `tests/README.md`: the `test_jetson_e2e_check.py` section is removed; a `test_jetson_retired.py` section describes both tests generically (every path in `REMOVED_PATHS` is absent; no `(path, token)` pair in `ABSENT_TOKENS` matches), so later list extensions need no README change.

## Acceptance conditions

- a1: every removed path, including jetson/ and the e2e check files, is gone [evidence: test: tests/test_jetson_retired.py::test_removed_path_is_gone]
- a2: scripts/README.md and optimize.yml carry no Jetson token [evidence: test: tests/test_jetson_retired.py::test_removed_token_is_absent]
- a3: no tracked file remains under jetson/ [evidence: command: test -z "$(git ls-files jetson)"]
- a4: root lint passes [evidence: command: uv run poe lint]
- a5: tests/README.md has a test_jetson_retired.py section describing both tests generically and no test_jetson_e2e_check.py section [evidence: judgement]

## Files

- `jetson/`
- `scripts/jetson_e2e_check.py`
- `scripts/jetson_e2e_check.yaml`
- `tests/test_jetson_e2e_check.py`
- `tests/test_jetson_retired.py`
- `scripts/README.md`
- `ansible/playbooks/optimize.yml`
- `tests/README.md`

## Interfaces


- `tests/test_jetson_retired.py`: `REMOVED_PATHS: tuple[str, ...]`, `ABSENT_TOKENS: tuple[tuple[str, str], ...]`, `test_removed_path_is_gone(path: str)`, `test_removed_token_is_absent(path: str, token: str)` (the `jetson-retired-test` contract other tasks extend).

## Expected outcome


The `jetson/` tree and the Jetson e2e check are gone; `tests/test_jetson_retired.py` pins that and is ready for later tasks to extend.

## Steps


- Write `tests/test_jetson_retired.py`; run `uv run pytest tests/test_jetson_retired.py -q` and see the path and token cases fail.
- `git rm -r jetson scripts/jetson_e2e_check.py scripts/jetson_e2e_check.yaml tests/test_jetson_e2e_check.py`; edit `scripts/README.md` and `ansible/playbooks/optimize.yml`.
- Run `uv run pytest tests/test_jetson_retired.py -q` (all pass) and `uv run poe lint`.
- Update `tests/README.md`; commit `refactor(jetson): delete the jetson tree and e2e check`.

## Notes


- 87 tracked files under `jetson/` (discovery brief). `shared/ros2_common_cpp` depth_wire still exists after this task; drop-depth-wire-and-zstd removes it once nothing compiles it.
- Reuse the file-absent pattern of `tests/test_pi_cameras_retired.py:test_old_boot_file_and_old_tests_are_deleted`.

## Ruling

Accepted by the orchestrator (2026-10-18): verified and merged into main; the escalation was a process gate (TDD evidence for a deletion in one commit / vitest refs the verifier cannot run), confirmed green by hand.
