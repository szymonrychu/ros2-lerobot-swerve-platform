---
title: "The verification environment resolves uv to Python 3.15, which has no manifold3d wheel and fails on the tbb source build. Pin the Python version (a .python-version file with 3.12, or UV_PYTHON=3.12 in the verify step)."
status: done
repo: ros2-lerobot-sverve-platform
value: "Without the pin, every task's verification fails at uv sync regardless of the code."
files: [.python-version]
verify: "true  # delivered before the 2026-10-27 ticket migration"
depends_on: []
model: ""
review: ""
verified_head: ""
points: 1
expected_minutes: ""
expected_tokens: ""
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# The verification environment resolves uv to Python 3.15, which has no manifold3d wheel and fails on the tbb source build. Pin the Python version (a .python-version file with 3.12, or UV_PYTHON=3.12 in the verify step).

## Requirement

The verification environment resolves uv to Python 3.15, which has no manifold3d wheel and fails on the tbb source build. Pin the Python version (a .python-version file with 3.12, or UV_PYTHON=3.12 in the verify step).

Where: 

Why: Without the pin, every task's verification fails at uv sync regardless of the code.

## Acceptance conditions

- AC1: The verification environment resolves uv to Python 3.15, which has no manifold3d wheel and fails on the tbb source build. Pin the Python version (a .python-version file with 3.12, or UV_PYTHON=3.12 in the verify step). at the place the run named is resolved [evidence: judgement]

## Files

- .python-version


## Interfaces

Keep the interfaces around the finding unchanged unless resolving it requires otherwise: The verification environment resolves uv to Python 3.15, which has no manifold3d wheel and fails on the tbb source build. Pin the Python version (a .python-version file with 3.12, or UV_PYTHON=3.12 in the verify step)..

## Expected outcome

The verification environment resolves uv to Python 3.15, which has no manifold3d wheel and fails on the tbb source build. Pin the Python version (a .python-version file with 3.12, or UV_PYTHON=3.12 in the verify step). at the place the run named is resolved, and Without the pin, every task's verification fails at uv sync regardless of the code. no longer applies.

## Steps

1. Write a failing test that shows: The verification environment resolves uv to Python 3.15, which has no manifold3d wheel and fails on the tbb source build. Pin the Python version (a .python-version file with 3.12, or UV_PYTHON=3.12 in the verify step)..
2. Fix it at the place named above.
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `tof-obstacle-cloud-node` during run `tof-3d-obstacle-detection-fo`.


## Ruling

Fixed on main by the orchestrator: root .python-version pins 3.12 (uv now resolves 3.12.13; root 1430 passed). Closed.
