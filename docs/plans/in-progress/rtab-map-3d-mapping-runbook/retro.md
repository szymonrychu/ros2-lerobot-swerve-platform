# RTAB-Map 3D mapping runbook retro

## Review

- Final verify: `2257 passed, 9 skipped in 243.75s`; lint, lint-nodes and test-ansible exit 0 (skips are pre-existing docker-only tests).
- Fix rounds: 0. Findings acted on: none. Findings dropped: none.
- "The D435 has no focus control" instead of "No focus lock": accepted, the plan's own test forbids the phrase and the meaning is kept.

## What went well

- One step, one commit (22f33728), diff limited to the runbook, its pin test and tests/README.md.
- All six pins exist under the planned names and pass; thresholds untouched.

## What went wrong or could improve

- The plan's step text and its own test disagreed on "focus lock"; the implementer had to reword. Plan-writers should check prose against the pins.

## Recommendations

- Discovered and not built: none.
- Out-of-scope ideas: none.
- Impossible at planning time: none. The live items 1.2, 1.3, 2.1, 3.1 remain in "After merge (live)" and need the D435 session (S59, S84, S72).
