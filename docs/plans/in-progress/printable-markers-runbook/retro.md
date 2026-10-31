# Printable markers runbook retro

## Review

- Final verify: pytest "2258 passed, 9 skipped in 280.71s", lint, lint-nodes and test-ansible exit 0 (at 2d463150; the fix commit 29662988 touches one README line, markers tests 7 passed and lint clean).
- Fix round 1: the "and later" wording in tests/README.md was removed (acted on).

## What went well

- Every hunk mapped to S1 or S2; the 7 pinned tests exist, assert literals and would fail on the old docs.
- The Results table kept no invented numbers.

## What went wrong or could improve

- Commit 77d808a7 was committed with a failing test and fixed in 2d463150; commit only green.
- A parking word ("later") slipped into an added README line; grep added lines for debt words before handing over.

## Recommendations

- Discovered and not built: none.
- Out-of-scope ideas: none.
- Impossible at planning time: the live run (steps 1-9, measured ranges, Results rows) needs D435 S59 and S84 done first; the plan-writer could ask for a live-run owner and date at the gate.
