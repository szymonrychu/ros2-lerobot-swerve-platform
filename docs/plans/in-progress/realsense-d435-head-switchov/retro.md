# RealSense D435 head switchover retro

## Review

- Final verify at 5b66248b: root pytest "2175 passed, 9 skipped" (docker-only), head_rgbd_calibration 944 passed, web_ui 1076 passed, frontend 1348 passed, all lints and ansible tests exit 0. Fix round 70a01972 is README-only; 34 README-pinning tests re-run green.
- Fix round 1 (acted on): nodes/web_ui/README.md still documented gripper camera keys, ToF and the zoom route; rewritten on the D435.
- Dropped as out of scope: robot-health.json camera counters (generic), client-cpu-budget.md (dated record).

## What went well

- Every step's named tests exist and ran; no debt markers, no skipped or weakened tests.
- Retirement tests on git ls-files pinned each deletion.
- S65 and S28 reviews found nothing.

## What went wrong or could improve

- The web_ui README was in no step's file list although rung item 1.10 covers all READMEs; the plan should list node READMEs for every behaviour step.

## Recommendations

- Discovered and not built: mcp_server models.py:334-353,459 and tools.py:926 still say "head ToF" in tool description strings; hints.capture_hints and MIN_COMMON_CORNERS are ToF-shaped dead code (only tests use them) and can be removed.
- Out-of-scope ideas: none.
- Impossible at planning time: none.
