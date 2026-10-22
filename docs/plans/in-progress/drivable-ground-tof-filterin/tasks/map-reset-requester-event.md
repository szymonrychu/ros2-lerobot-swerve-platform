---
title: Map reset requester event
status: todo
repo: ros2-lerobot-sverve-platform
value: "A map reset from the web UI is no longer a mystery: the agent gets an event naming who asked"
files: [nodes/web_ui/web_ui/server.py, nodes/web_ui/web_ui/bridge.py, nodes/web_ui/tests/test_map_reset_event.py, nodes/web_ui/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests/test_map_reset_event.py -q && uv run poe lint"
depends_on: [web-ui-ground-recorder-api, web-ui-applied-head-urdf]
owns: [map-reset-event]
uses: []
model: ""
estimate: 50
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Map reset requester event

## Requirement

Explain the map reset of the 16:28 "Explore" job and make every web UI map reset a robot event that names who asked
(robot agent retrospective 2026-10-22 16:28: "The map reset (epoch 1->2) during go_to_frontier 52 right after a
collision_stop StopBox; there was no map_guard event in robot_events. Please investigate why."). Design D57; approval R3.

Evidence already pulled (`.logs/agent-retro-2026-10-22/web_ui.log`, absolute path under the main checkout
`/Users/szymonri/Documents/ros2-lerobot-sverve-platform/.logs/`): at 16:24:29 CEST web_ui logged
`reset_map_requested` (service `/slam_toolbox/reset`, logger web_ui.bridge) and at 16:24:44 `map_reset` ok ("removed 3
live POIs and 1 photos"): the reset came through web_ui's `POST /api/map/reset`, not from map_guard (map_guard_cpp runs
in dry run and logged only a lifted/carried episode at 16:21). Nothing records the requester today.

- Investigate first: from Loki (web_ui, claude_agent, mcp_server and the web proxy, if its access log is shipped) find
  which client called `POST /api/map/reset` at 16:24:29 (browser tab, a script, or a tool path); write the finding, or
  that the requester cannot be known from the existing logs, to `nodes/web_ui/README.md` section
  `### Map reset during go_to_frontier`.
- `POST /api/map/reset` logs the requester (client address, user agent, tab) with the request and the result.
- Every reset request publishes one `/robot_events` event (std_msgs/String JSON, the contract in
  `nodes/claude_agent/claude_agent/robot_events.py`: `{seq, ts, type, severity, source, message, data}`) with
  `type: "map_reset"`, `source: "web_ui"`, `severity: "critical"`, a message naming the tab, the requester and the outcome
  (ok, or the failure), and `data: {tab, requester, user_agent, ok, pois_removed, photos_removed}`; published after the
  outcome is known, also when the reset fails or is refused (battery block). Reliable QoS, depth 10, like the other
  publishers of the topic. Nothing is published for a request that never reached the route.
- Tests first (seen failing): `nodes/web_ui/tests/test_map_reset_event.py` (`test_reset_publishes_map_reset_event`,
  `test_failed_reset_publishes_event_with_failure`, `test_reset_logs_requester`) with the bridge faked as the existing
  web_ui tests do.
- `nodes/web_ui/README.md` (map reset part) documents the event; tests/README is not needed (node test).
- Type hints, docstrings with types, constants on top (CLAUDE.md).

## Acceptance conditions

- a1: the requester of the 16:24:29 reset (or why it cannot be known) is in the node README [evidence: doc-section:nodes/web_ui/README.md#Map reset during go_to_frontier]
- a2: a web UI map reset publishes a map_reset robot event with tab, requester and outcome [evidence: test:nodes/web_ui/tests/test_map_reset_event.py::test_reset_publishes_map_reset_event]
- a3: a failed or refused reset still publishes the event with the failure [evidence: test:nodes/web_ui/tests/test_map_reset_event.py::test_failed_reset_publishes_event_with_failure]
- a4: the reset route logs the requester [evidence: test:nodes/web_ui/tests/test_map_reset_event.py::test_reset_logs_requester]
- a5: web_ui suite and lint pass [evidence: command:cd nodes/web_ui && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/tests/test_map_reset_event.py`
- `nodes/web_ui/README.md`

## Interfaces

Builds part map-reset-event: `/robot_events` `type: "map_reset"`, `source: "web_ui"`, data `{tab, requester,
user_agent, ok, pois_removed, photos_removed}` (cross-epic contract robot-event-map-reset), published by web_ui's bridge
from `POST /api/map/reset`.

## Expected outcome

`cd nodes/web_ui && uv run --frozen pytest tests/test_map_reset_event.py -q` passes three tests that failed before;
the README names who reset the map at 16:24:29 or why that cannot be known.

## Steps

- Investigate the 16:24:29 request in Loki; write the section.
- Write the three tests; they FAIL.
- Add the requester log and the event publish; tests PASS; run the verify command and the web_ui suite; commit
  `feat(web_ui): map reset publishes a robot event naming the requester`.

## Notes

- Live step: none (code, run 1 of D29). No live map reset is done (it deletes the live POIs and photos).
- Ordered after web-ui-ground-recorder-api and web-ui-applied-head-urdf (server.py, bridge.py).
