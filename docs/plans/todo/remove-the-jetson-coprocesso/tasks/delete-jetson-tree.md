---
title: Delete jetson tree
status: todo
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
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
