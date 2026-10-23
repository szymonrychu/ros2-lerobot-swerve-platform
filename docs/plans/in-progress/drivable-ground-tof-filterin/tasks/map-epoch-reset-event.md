---
title: Map epoch reset event
status: done
repo: ros2-lerobot-sverve-platform
value: Every map reset the agent runs into is explained by an event with its cause
files: [nodes/mcp_server/mcp_server/frontiers.py, nodes/mcp_server/mcp_server/frontier_tools.py, nodes/mcp_server/mcp_server/monitor.py, nodes/mcp_server/tests/test_map_epoch_event.py, nodes/mcp_server/README.md]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_map_epoch_event.py -q && uv run poe lint"
depends_on: [map-reset-requester-event]
owns: [map-epoch-event]
uses: [map-reset-event]
model: ""
estimate: 60
actual: ""
review: ""
branch: main
verified_head: 75a7f4bd194b1dc2da0b415308e513c0adbe7fd0
run: drivable-ground-tof-filterin
provides: [map-epoch-event]
acs_for_audit: []
process_gaps: []
---

# Map epoch reset event

## Requirement

Make every map reset the agent sees explained: when mcp_server's frontier tracker starts a new epoch, a robot event
names the cause (robot agent retrospective 2026-10-22 16:28: the map reset mid go_to_frontier "with no map_guard event";
user: "make every map reset logged/explained"). Design D57; approval R3. Uses part map-reset-event (task
map-reset-requester-event).

- When `FrontierTracker` (`nodes/mcp_server/mcp_server/frontiers.py`) starts a new epoch (known cells below
  `epoch_reset_known_fraction` of the previous map, or a resolution change), mcp_server's monitor publishes one
  `/robot_events` event `type: "map_epoch_reset"`, `source: "mcp_server"`, `severity: "critical"` while a base motion
  or a job runs, else `"warning"`, with `data: {epoch_before, epoch_after, known_before, known_after, resolution_before,
  resolution_after, cause}`; `cause` is the latest `map_reset` (web_ui) or `map_guard_reset` (map_guard) event seen on
  `/robot_events` within 30 s before (its source, requester and message), else `"unexplained"` with the cell counts.
- The event is debounced like the other monitor events, shows in `get_body_state` and the tool-result digest, and is
  logged at warning with the same fields.
- `get_exploration_status` / `list_frontiers` results carry `epoch_changed_at` and `epoch_cause` for the current epoch
  (null in epoch 1).
- Tests first (seen failing): `nodes/mcp_server/tests/test_map_epoch_event.py` (`test_epoch_change_publishes_event`,
  `test_cause_from_recent_map_reset_event`, `test_unexplained_cause_with_counts`, `test_exploration_status_names_epoch_cause`).
- `nodes/mcp_server/README.md` (frontiers and robot events parts) updated.
- Type hints, docstrings with types, constants on top (CLAUDE.md).

## Acceptance conditions

- a1: a new frontier epoch publishes a map_epoch_reset robot event [evidence: test:nodes/mcp_server/tests/test_map_epoch_event.py::test_epoch_change_publishes_event]
- a2: the event names a recent web_ui or map_guard reset as its cause [evidence: test:nodes/mcp_server/tests/test_map_epoch_event.py::test_cause_from_recent_map_reset_event]
- a3: without a known reset the cause is unexplained with the cell counts [evidence: test:nodes/mcp_server/tests/test_map_epoch_event.py::test_unexplained_cause_with_counts]
- a4: exploration status names the epoch cause [evidence: test:nodes/mcp_server/tests/test_map_epoch_event.py::test_exploration_status_names_epoch_cause]
- a5: mcp_server suite and lint pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/mcp_server/mcp_server/frontiers.py`
- `nodes/mcp_server/mcp_server/frontier_tools.py`
- `nodes/mcp_server/mcp_server/monitor.py`
- `nodes/mcp_server/tests/test_map_epoch_event.py`
- `nodes/mcp_server/README.md`

## Interfaces

Uses part map-reset-event; builds part map-epoch-event: `/robot_events` `type: "map_epoch_reset"`, `source:
"mcp_server"`, data `{epoch_before, epoch_after, known_before, known_after, resolution_before, resolution_after,
cause}`; `get_exploration_status` and `list_frontiers` gain `epoch_changed_at`, `epoch_cause`.

## Expected outcome

`cd nodes/mcp_server && uv run --frozen pytest tests/test_map_epoch_event.py -q` passes four tests that failed before.

## Steps

- Write the four tests; they FAIL.
- Implement the event in the monitor and the epoch fields; tests PASS; run the verify command and the mcp_server suite;
  commit `feat(mcp_server): map epoch resets are robot events with their cause`.

## Notes

- Live step: none (code, run 1 of D29).
- map_guard's own event type is `map_guard_reset` (`nodes/map_guard_cpp/include/map_guard_cpp/agent.hpp`).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on map-reset-requester-event
