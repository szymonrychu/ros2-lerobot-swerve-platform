---
title: Final suite and runbook
status: todo
repo: ros2-lerobot-sverve-platform
value: "Named maps are proven on the robot: a saved map is loaded after a reset and the robot relocalises near the saved pose, with measured timings and thresholds."
files: [docs/runbooks/named-maps.md, ansible/group_vars/client.yml, nodes/map_manager/README.md]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && (cd nodes/poi_store && uv run --frozen pytest -q) && (cd nodes/map_manager && uv run --frozen pytest -q) && (cd nodes/map_guard && uv run --frozen pytest -q) && (cd nodes/web_ui && uv run --frozen pytest -q) && (cd nodes/mcp_server && uv run --frozen pytest -q) && (cd nodes/claude_agent && uv run --frozen pytest -q) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)"
depends_on: [named-maps-docs-and-memory]
owns: []
uses: []
model: sonnet
estimate: 90
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Final suite and runbook

## Requirement

- Full suite green (the verify command, every line read).
- `docs/runbooks/named-maps.md`, written and then executed with results filled in (date, numbers, pass/fail per step):
  1. Preconditions: `apt-cache policy ros-jazzy-slam-toolbox` recorded; map_manager deployed with the other changed
     nodes through the ansible-deploy skill, logs in `.logs/` (`./scripts/deploy-nodes.sh client map_manager poi_store
     web_ui map_guard_cpp`; mcp_server and claude_agent only when no agent session runs, MEMORY).
  2. Save: with a mapped room, "Save map as" `runbook-room` from the web UI; the bundle directory has the four files;
     bundle.json last pose equals the robot pose shown in the Map tab; POI count equals the POI panel.
  3. Clear POIs: clear the agent's POIs with zones kept, then all; counts match.
  4. Reset: "Reset map"; the map is empty; POIs as left by step 3.
  5. Load at the saved pose (robot not moved): state `localised`, ratio recorded, pose within 0.10 m / 5 deg of the
     saved pose, POIs restored, costmap_zones masks back, GPS anchor reset, map_guard_cpp logs hold then re-arm.
  6. Load with the robot moved about 0.5 m and 30 deg (drive slowly, least dramatic motion): state `localised` with
     the refined pose within 0.15 m / 10 deg of the true pose (measured with a tape), search time recorded.
  7. Load with the robot in another room or turned to a symmetric view: state `not_localised` or `ambiguous`, slam
     stays paused (`ros2 param get /slam_toolbox paused_new_measurements` true), the map is not overwritten, nothing
     published on `/map_manager/localised` as true; then Retry with a correct pose from the agent's `load_map(x, y,
     yaw)` localises.
  8. Agent: `list_maps`, `save_map`, `load_map` and `clear_pois` from a session (when allowed), result recorded.
  9. Timings: deserialize + verify durations and the candidate search time on the Pi; map_manager idle CPU from
     Grafana/Mimir; adjust `localised_ratio`, search window or `load_timeout_s` in client.yml when the measurements say
     so, and record the final values in `nodes/map_manager/README.md`.
- The task ends with the runbook committed with real numbers, or with a named blocker (robot offline) per CLAUDE.md.

- Runbook step 7b (U8, executed on the robot, hand-carried, no servo moves): (i) lift and set the robot down about 0.5 m away: map_guard_cpp (dry_run false for the step) requests relocalise, state `localised`, map and POI counts unchanged; (ii) carry it to another room: unrecoverable, the live map and POIs are cleared, the event reaches the web UI, and the saved `runbook-room` bundle is still listed and loads.

## Acceptance conditions

- a1: the whole repository suite, node suites, frontend and C++ tests pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp]
- a2: the runbook records save, reset, load at the saved pose, load after a move and a failed load with numbers [evidence: doc-section: docs/runbooks/named-maps.md#Results]
- a3: a failed relocalisation left slam_toolbox paused and the loaded map intact on the robot [evidence: judgement]
- a4: the runbook records a recoverable move (nothing deleted) and an unrecoverable one (live map and POIs cleared, saved bundle intact) [evidence: doc-section: docs/runbooks/named-maps.md#Moved by hand]

## Files

- `docs/runbooks/named-maps.md`
- `ansible/group_vars/client.yml`
- `nodes/map_manager/README.md`

## Interfaces

- `docs/runbooks/named-maps.md` with a `## Results` section.

## Expected outcome

The epic's goals are verified on the robot with recorded numbers.

## Steps

- Write the test(s) named in the requirement first; run `uv run pytest tests/ -q` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && (cd nodes/poi_store && uv run --frozen pytest -q) && (cd nodes/map_manager && uv run --frozen pytest -q) && (cd nodes/map_guard && uv run --frozen pytest -q) && (cd nodes/web_ui && uv run --frozen pytest -q) && (cd nodes/mcp_server && uv run --frozen pytest -q) && (cd nodes/claude_agent && uv run --frozen pytest -q) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)`; expect every test passed and lint clean.
- Commit: `docs: named maps runbook with measured results`.

## Notes

Use the ansible-deploy skill before deploying (CLAUDE.md); logs in `.logs/`. Driving for step 6 is slow and short; no arm motion. Deploy mcp_server and claude_agent only when no agent session runs (MEMORY).
