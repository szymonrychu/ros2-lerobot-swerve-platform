---
title: Ground source tag depth not tof
status: todo
repo: ros2-lerobot-sverve-platform
value: "Ground warnings name the real sensor; no consumer depends on the retired ToF"
files: [nodes/ground_estimator_cpp/src/core/cell_class.cpp, nodes/ground_estimator_cpp/src/core/traversability.cpp, nodes/ground_estimator_cpp/test, nodes/ground_estimator_cpp/README.md, nodes/web_ui/web_ui/ground_recorder.py, nodes/web_ui/tests]
verify: "scripts/cpp-node-test.sh ground_estimator_cpp && (cd nodes/web_ui && uv run pytest tests -q && uv run poe lint) && uv run pytest tests -q"
depends_on: [ground-estimator-on-d435-inp]
model: sonnet
review: ""
verified_head: ""
points: 2
expected_minutes: 5
expected_tokens: 487766
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Ground source tag depth not tof

## Requirement

The ground_estimator_cpp published warning areas / cell classes still tag their source as "tof" (src/core/cell_class.cpp, src/core/traversability.cpp), and web_ui ground_recorder.py reads that tag; the head depth sensor is now the D435 (found by ground-estimator-on-d435-inp, 2026-10-28). Rename the source value to "depth" end to end: producer, every consumer (grep web_ui backend and frontend, mcp_server, claude_agent, scripts, replay fixtures readers) and their tests, in one change so no consumer sees an unknown tag. Document the value in the ground_estimator_cpp README.

## Acceptance conditions

- a1: no "tof" source value remains in the ground state contract or its consumers [evidence: command:! grep -rn "\"tof\"" nodes/ground_estimator_cpp/src nodes/web_ui/web_ui nodes/mcp_server/mcp_server]
- a2: producer and consumer tests pass with "depth" [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp && (cd nodes/web_ui && uv run pytest tests -q)]
- a3: root suite passes [evidence: command:uv run pytest tests -q]

## Files

- `nodes/ground_estimator_cpp/src/core/cell_class.cpp`
- `nodes/ground_estimator_cpp/src/core/traversability.cpp`
- `nodes/ground_estimator_cpp/test`
- `nodes/ground_estimator_cpp/README.md`
- `nodes/web_ui/web_ui/ground_recorder.py`
- `nodes/web_ui/tests`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
