# Manual robot tests: 2026-10-18 deliveries

Run on or against the client (`client.ros2.lan`). Deploys go through the `ansible-deploy` skill and
`scripts/deploy-nodes.sh`, logged to `.logs/`. ROS commands need
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. Web UI API base: `http://client.ros2.lan:8080`.
Servo safety (CLAUDE.md): gripper first, minimal amplitude, stop and ask before anything larger.

| ID | Area | Person at robot | Status |
|---|---|---|---|
| T1 | Named maps: load at saved pose | yes | PENDING |
| T2 | Named maps: load after 0.5 m / 30 deg move | yes | PENDING |
| T3 | Named maps: other room + Retry | yes | PENDING |
| T4 | Named maps: kidnap 0.5 m (map_guard_cpp) | yes | PENDING |
| T5 | Named maps: kidnap to other room | yes | PENDING |
| T6 | Named maps: agent tools | yes | PENDING |
| T7 | Split servo buses: arm teleop tracking | yes | PENDING |
| T8 | Split servo buses: decision on the 98 Hz gate | no | PENDING |
| T9 | Front camera: agent zoom | no | PENDING |
| T10 | Front camera: web UI click-to-zoom | yes (browser) | PENDING |
| T11 | Front camera: autofocus on clicked point | yes | PENDING |
| T12 | Markers: QR marker PDF print | yes | PENDING |
| T13 | Markers: ChArUco board print | yes | PENDING |
| T14 | Markers: two-colour STL/3MF print | yes | PENDING |

## Named maps (source: `docs/runbooks/named-maps.md`, steps 5-8)

### T1 - Load at the saved pose (step 5)

- Purpose: confirm the ambiguity (`search.ambiguity_min_support`) and `min_evaluated_fraction` fixes make the load reach `localised`.
- Preconditions: fixes merged and pushed to `origin/main`; bundle `runbook-room` exists in `/var/lib/ros2/maps/named`; robot still where `runbook-room` was saved, not moved.
- Steps:
  1. `mkdir -p .logs && ./scripts/deploy-nodes.sh client map_manager map_guard_cpp 2>&1 | tee .logs/deploy-client-map-fixes.log` (deploy code and config together).
  2. `curl -s -X POST 'http://client.ros2.lan:8080/api/maps/load?tab=map' -H 'content-type: application/json' -d '{"name":"runbook-room"}'`
  3. Check `/map_manager/localised` and `ros2 param get /slam_toolbox paused_new_measurements`.
- Expected: answer in about 8 s, state `localised`, ratio about 0.99, pose within 0.01 m / 0.1 deg of saved, POIs restored, slam unpaused (`False`), `/map_manager/localised` true, map_guard_cpp re-armed. Also note whether costmap_zones masks and the GPS anchor reset happen (not observed so far).
- Result/notes: ______________________

### T2 - Load after moving 0.5 m / 30 deg (step 6)

- Purpose: search recovery inside the +-1.0 m window.
- Preconditions: T1 passed; tape measure; `runbook-room` saved.
- Steps: push the robot by hand about 0.5 m and 30 deg from the saved pose (measure it), then repeat the T1 load call.
- Expected: `localised` within 0.15 m / 10 deg of the true pose, search about 1.7 s, total about 8 s.
- Result/notes: ______________________

### T3 - Other room, then Retry (step 7)

- Purpose: a wrong room must not overwrite the live map; Retry with a correct pose recovers.
- Preconditions: T1 passed; robot can be carried (wheels off the floor risk: power the drive off or carry gently).
- Steps:
  1. Carry the robot to another room (or turn it to a symmetric view). Load `runbook-room` as in T1.
  2. `ros2 param get /slam_toolbox paused_new_measurements` and watch `/map_manager/localised`.
  3. Carry it back, set it at a known pose, then Retry from the Map tab or `load_map` with x, y, yaw.
- Expected: step 1-2: `not_localised` or `ambiguous`, paused true, live map unchanged, `localised` never true. Step 3: `localised`.
- Result/notes: ______________________

### T4 - Kidnap 0.5 m with map_guard_cpp live (step 7b i)

- Purpose: map_guard_cpp requests relocalise after a small kidnap.
- Preconditions: map loaded and `localised` (T1). Set `dry_run: false` for `map_guard_cpp` in `ansible/group_vars/client.yml` (local edit), then `./scripts/deploy-nodes.sh client map_guard_cpp 2>&1 | tee .logs/deploy-client-guard.log`.
- Steps: lift the robot and set it down about 0.5 m away. Watch map_guard_cpp logs (`ansible client -m ansible.builtin.command -a 'journalctl -u ros2-map_guard_cpp -n 40 --no-pager'`).
- Expected: relocalise requested, state returns to `localised`, map and POI counts unchanged.
- Result/notes: ______________________

### T5 - Kidnap to another room (step 7b ii)

- Purpose: the live map and POIs are cleared, saved bundle intact.
- Preconditions: T4 setup (`dry_run: false` still active).
- Steps: carry the robot to another room. Check the Map tab, `curl -s 'http://client.ros2.lan:8080/api/maps?tab=map'` (or list in the UI), then load `runbook-room` again.
- Expected: live map and POIs cleared, event shows in the web UI, `runbook-room` still listed and loads.
- Cleanup: restore `dry_run` to its original value (`git checkout ansible/group_vars/client.yml`) and redeploy `map_guard_cpp`.
- Result/notes: ______________________

### T6 - Agent tools (step 8)

- Purpose: `list_maps`, `save_map`, `load_map`, `clear_pois` work from the agent.
- Preconditions: idle claude_agent session (`curl -s http://client.ros2.lan:18300/api/state`); never deploy mcp_server/claude_agent while a session runs (exit 143).
- Steps: in the web UI chat ask the agent, one at a time: list the maps; save the current map as `agent-test`; clear the agent POIs; load `runbook-room` at the current pose.
- Expected: each tool call returns ok, `agent-test` appears in the list, POI count drops as asked, load ends `localised`.
- Result/notes: ______________________

## Split servo buses (source: `docs/plans/done/split-servo-buses-arm-alone/tasks/final-suite-and-runbook.md`)

Measured already: arm bus 94.7 Hz idle, 89 Hz while driving (gate 98 Hz; cycle 4.1 ms of 10 ms, CPU/timer-bound); base bus 48.7 Hz idle; base nudges 3.7 cm and 11.3 cm OK; consumers OK.

### T7 - Arm teleop tracking

- Purpose: the follower tracks the leader with the arm alone on its bus.
- Preconditions: leader and follower powered, clear workspace. `topic_scraper_api` enabled (`enabled: true` in `ansible/group_vars/client.yml`, local edit; `./scripts/deploy-nodes.sh client topic_scraper_api 2>&1 | tee .logs/deploy-client-scraper.log`). Gripper only; no other joint without asking.
- Steps:
  1. `./scripts/teleop_diag.sh` - all three teleop topics PRESENT.
  2. In another shell: `python scripts/topic_scraper_collect.py --source client=http://client.ros2.lan:18100 --select /follower/joint_states:.position --interval 0.1 > .logs/teleop-tracking.ndjson`
  3. Open and close the leader gripper slowly by hand a few times.
  4. Optional: `curl -s http://client.ros2.lan:18100/rules/leader_vs_follower`.
- Expected: `/follower/joint_states` gripper position follows the leader smoothly, no oscillation or runaway.
- Cleanup: set `topic_scraper_api` back to `enabled: false` (`git checkout ansible/group_vars/client.yml`) and redeploy it.
- Result/notes: ______________________

### T8 - Decision: 98 Hz arm gate

- Purpose: the arm bus measures 94.7 Hz idle / 89 Hz driving against the 98 Hz gate.
- Preconditions: T7 result in hand.
- Steps: decide one: (a) accept (tracking in T7 is fine; lower the gate in the plan/tests to the measured value with margin), or (b) require 98 Hz and open a follow-up to cut the cycle (CPU/timer-bound, 4.1 ms of 10 ms).
- Expected: a written decision.
- Result/notes: ______________________

## Front camera (source: `docs/plans/done/front-camera-30-fps-stream-w/tasks/final-suite-and-runbook.md`)

Measured already: head_camera_cpp 30.1 fps, web UI zoom API x2 ok in 1.26 s, memory peak 71 MB.

### T9 - Agent zoom

- Purpose: `get_camera_image` zoom works end to end.
- Preconditions: no running agent session you care about; mcp_server and claude_agent deployed with the zoom tool.
- Steps: in the web UI chat ask: "Call get_camera_image with camera front and zoom {u: 0.5, v: 0.5, factor: 4}".
- Expected: status `ok` (or `af_failed` once, then retry ok), image 1024x576, `phase_ms` total under 2.5 s; 30 fps dips under 1 s and recovers.
- Result/notes: ______________________

### T10 - Web UI click-to-zoom

- Purpose: Head tab zoom UI.
- Preconditions: web_ui deployed; browser open on `http://client.ros2.lan:8080`.
- Steps: Head tab, pick factor 4, click a point on the RGB view, then click "Back to live". API check: `curl -s -X POST 'http://client.ros2.lan:8080/api/camera/zoom?tab=head' -H 'content-type: application/json' -d '{"u":0.5,"v":0.5,"zoom":2}'`.
- Expected: crop appears, "Back to live" restores the stream; curl returns 200 with `status` ok.
- Result/notes: ______________________

### T11 - Autofocus on the clicked point

- Purpose: focus lands where clicked.
- Preconditions: T10 works; two objects at clearly different distances (near and far) in view, with text or texture.
- Steps: zoom (factor 4) on the near object, check sharpness; Back to live; zoom on the far object.
- Expected: the clicked object is sharp in each crop (`af_failed` once is acceptable, retry).
- Result/notes: ______________________

## Markers and calibration board (source: `docs/plans/done/marker-and-calibration-board`, `scripts/README.md` section markers_generate.py)

Generate once: `uv run python -m scripts.markers_generate` (outputs in `.logs/markers`, gitignored, never commit).

### T12 - QR marker PDF

- Purpose: printed QR marker has the right size and scans.
- Steps: print one marker PDF (for example `.logs/markers/charger-dock-1.pdf`) at 100 % (actual size, no fit to page); measure the 100 mm ruler on the page with calipers.
- Expected: ruler measures 100 mm, white quiet zone intact, a phone scans the payload `P1/K:<kind>/N:<name>/S:<mm>`.
- Result/notes: ______________________

### T13 - ChArUco board

- Purpose: calibration board is true to scale.
- Steps: print `.logs/markers/charuco-6x5-40mm.pdf` at 100 %; measure several 40 mm squares with a ruler or calipers; mount flat on a rigid plate.
- Expected: squares 40 mm (30 mm markers), within about 0.5 mm; board flat.
- Result/notes: ______________________

### T14 - Two-colour STL/3MF on the Bambu

- Purpose: base and modules print in two colours.
- Steps: open one 3MF (for example `.logs/markers/charger-dock-1.3mf`) in Bambu Studio; check extruders 1 (base, white) and 2 (modules, black) with the AMS; slice and print; scan the result.
- Expected: white base, black raised modules, scans like T12. Module size must be at or above 1.5 mm (warn under 2 mm).
- Result/notes: ______________________

## Note: head servos

Pan ID 20 and tilt ID 21 sit on the base bus (5A7A059004) and are not configured in software yet (the pan/tilt epic is next). Keep them unpowered-safe and unused (no commands, no torque tests) until that epic lands.
