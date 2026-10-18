---
title: "The verification environment resolves uv to Python 3.15, which has no manifold3d wheel and fails on the tbb source build. Pin the Python version (a .python-version file with 3.12, or UV_PYTHON=3.12 in the verify step)."
status: todo
repo: ros2-lerobot-sverve-platform
value: "Without the pin, every task's verification fails at uv sync regardless of the code."
files: []
verify: ""
depends_on: []
owns: []
uses: []
model: ""
estimate: ""
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# The verification environment resolves uv to Python 3.15, which has no manifold3d wheel and fails on the tbb source build. Pin the Python version (a .python-version file with 3.12, or UV_PYTHON=3.12 in the verify step).

## Requirement

The verification environment resolves uv to Python 3.15, which has no manifold3d wheel and fails on the tbb source build. Pin the Python version (a .python-version file with 3.12, or UV_PYTHON=3.12 in the verify step).

Where: 

Why: Without the pin, every task's verification fails at uv sync regardless of the code.

## Acceptance conditions

- AC1: The verification environment resolves uv to Python 3.15, which has no manifold3d wheel and fails on the tbb source build. Pin the Python version (a .python-version file with 3.12, or UV_PYTHON=3.12 in the verify step). at the place the run named is resolved [evidence: judgement]

## Files



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

