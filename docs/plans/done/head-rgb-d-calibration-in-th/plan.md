---
title: Head RGB-D calibration in the web UI Calibrate page
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-19
owner: Szymon Richert
rung: 1
---

# Head RGB-D calibration in the web UI Calibrate page

## What and why

What: Capture, review, compute and apply the head RGB-D calibration from the web UI Calibrate page, all on the robot
How: A robot-side calibration capture and compute service fed by the live topics, poses stored in a state dir, web_ui routes and a Calibrate page section, an apply path the head nodes and URDF pick up
Why: The terminal flow lost frames and bags on the loaded robot and gave the operator no view of usable poses

Calibrating the head RGB-D pair (Camera Module 3 mapping stream + Arducam ToF) from the Mac terminal failed repeatedly: frames streamed over ssh were lost on the loaded robot, bags came back empty, and the operator could not see which poses were usable. The operator wants the whole flow on the robot, driven from the web UI's hidden Calibrate page (hamburger menu, #/calibrate), next to the IMU calibration section (enabled on the robot in 053adfe3).

From the page the operator starts a capture session, moves the ChArUco board through poses and sees each stored pose appear live (count, RGB and ToF thumbnails with the detected corners, an image-coverage hint, a status line such as "moving" / "move closer"), deletes bad poses, lists and reopens past sessions, launches the calibration computation on a session and reads its report (RMS values, view counts, limits), then applies the result so the head nodes use it, or downloads the YAMLs to commit them.

## Scope ladder

### Rung 1: agreed scope (migrated)

- 1.1: Start and stop a head RGB-D calibration capture from the web UI Calibrate page; the capture runs on the robot and stores only usable still poses (RGB + ToF amplitude + depth + detections) in a robot state dir.
- 1.2: The page shows the session live: pose count, current status, RGB and ToF thumbnails with corners, a coverage hint for both images.
- 1.3: Delete single poses; list past sessions and reopen any of them.
- 1.4: Compute the calibration (RGB intrinsics, ToF intrinsics, stereo extrinsic, limits, report) on the robot from a session, without starving the robot stack or the web UI, and show the report.
- 1.5: Apply a computed result: head_camera_cpp, head_rgbd_cpp and the head URDF (tof_joint) use it without a redeploy, and it survives deploys. Download the result YAMLs.
- 1.6: The IMU section keeps working on the same page.

## Agreed scope

Rung 1, migrated from the epic format: the epic's goals were its agreed scope.

## Non-goals

- No change to the ChArUco board, the marker generator, or the calibration maths (solvers, limits) beyond what running on the robot needs.
- Validation captures (wall distances, pan sweep, time offset) stay in the existing CLI.
- No streaming of frames to the dev machine.

## Design

Migrated from the epic format; the design sections without a plan.md heading are under Migrated notes.

## Decisions

### D1: A dedicated Python node `head_rgbd_calibration`, web_ui proxies it

The node owns capture, the session store, the compute job and apply; it serves a JSON/binary HTTP API on
`127.0.0.1:18110` (port: my call; 18080 and 18100 are taken). web_ui forwards `/api/calibration/rgbd/*` to it.
Alternatives: capture and compute inside web_ui (rejected: 512M cap, a ~500 MB worst case per the brief, GIL and event
loop contention with the UI, and the focus-lock subscription tied to the UI process); a C++ node (rejected: the
calibration core is Python and must be reused). The node has its own cgroup: memory_low 64M, memory_max 768M,
cpu_weight 200, io_weight 200, oom_score_adjust 0 (my call: below the robot tiers 500/1000, above pure background,
because capture is interactive). It runs `enabled: true`; idle it holds no subscriptions. Serves A2, A4.

### D2: The calibration core moves to `shared/ros2_common/head_rgbd_calibration`

Pure modules move with `git mv` and unchanged maths: `config.py` (StrictModel, BoardConfig, SelectionConfig,
LimitsConfig), `charuco.py`, `frames.py` (dataclasses, normalise_amplitude, rgb_difference, tof_diff_fraction,
nearest_index, still_pairs), `intrinsics.py`, `extrinsics.py`, `calibrate.py` (detect_tof, CalibrationRun,
`calibrate_detections` taking precomputed detections, `failed_limits(run, limits)`, `report_markdown(run, limits, ...)`),
`urdf_text.py` (stdlib only: rewrite_tof_joint, format_vec, joint constants), `urdf_handoff.py` (numpy/scipy part),
and `poses.py` (PoseCounter, extended for depth and capture payloads). rosbags, ssh and repo-path code stays in
`scripts/head_rgbd_calibration` (frames bag reading, capture_bag, CLI config models, outputs, lens_position, validation).
The root uv project gains `ros2-common` as an editable dev dependency so root tests import the package.
`live.py`, `live_relay.py`, `capture.live` and their tests are deleted, pinned by a `git ls-files` retirement test.
The 8-file task cap splits the move into a chain (config and charuco; frames and intrinsics; extrinsics and urdf;
calibrate), each green on its own: a moved module leaves a one-line re-export in `scripts/head_rgbd_calibration` for
its remaining importers, and `calib-core-retire-shims` deletes every re-export once the live counter is retired,
pinned by a `git ls-files` retirement test (my call). Serves A5.

### D3: Robot state dir layout, only selected poses stored

`/var/lib/ros2/head_rgbd_calibration` (node config `state_dir`), owned by ansible_user, never touched by deploys:

```
sessions/<sid>/session.json                 id, created, lens_position, rgb_size, tof_size, tof_info (SDK camera_info) or null, next_pose
sessions/<sid>/poses/<pid>/rgb.jpg          mapping JPEG bytes exactly as received
sessions/<sid>/poses/<pid>/amplitude.png    uint16 PNG
sessions/<sid>/poses/<pid>/depth.png        uint16 mm PNG; absent when no depth frame within max_pair_dt_s
sessions/<sid>/poses/<pid>/pose.json        stamps, rgb and tof corner ids and points
sessions/<sid>/poses/<pid>/rgb_thumb.jpg    <= 320 px wide, corners drawn
sessions/<sid>/poses/<pid>/tof_thumb.jpg    normalised amplitude enlarged 2x, corners drawn
sessions/<sid>/results/<rid>/               head_camera_calibration.yaml, tof_calibration.yaml, head_rgbd_extrinsics.yaml,
                                            tof_joint.json, head.urdf (tof_joint rewritten), report.md, result.json
applied/                                    head_camera_calibration.yaml, tof_calibration.yaml, head_rgbd_extrinsics.yaml,
                                            tof_joint.json, applied.json (written last)
```

Ids: session `calib-YYYYmmdd-HHMMSS`, pose zero-padded `001` (stable, never reused after a delete), result
`r-YYYYmmdd-HHMMSS`; every id is regex-checked and the resolved path must stay under `state_dir`. Writes are atomic
(temp file plus `os.replace`). A 30-pose session is about 6 MB. Serves A1, A2, A3.

### D4: Capture subscribes only during a session, with the measured QoS

Start creates the subscriptions, stop (or `max_capture_s`, default 900 s, D12 c) destroys them, so the focus lock
is released. `/tof/amplitude/image_raw`, `/tof/depth/image_raw` RELIABLE depth 10; `/tof/camera_info` RELIABLE;
`/head_camera/mapping/image_raw/compressed` BEST_EFFORT depth 5. Frames feed the core `PoseCounter`; a captured pose
is written with its detections (no re-detection) and thumbnails. The status line is the PoseCounter status, or
`waiting for frames` until both streams delivered a frame. A capture can start on an existing session (reopen and add
poses). Serves A2, A3.

### D5: Compute in a niced child process, one job at a time

`POST compute` starts `python3 -m head_rgbd_calibration.compute` (niceness 10, timeout `compute_timeout_s` 600 s)
that loads the session's poses from disk, calls the core `calibrate_detections`, `failed_limits` and `report_markdown`,
computes the tof_joint origin from the deployed checkout's head.urdf (`head_urdf_file`), and writes the result dir
atomically. A failing result is kept with `passed: false` so its report is visible. Capture and compute are mutually
exclusive (409). Serves A3, A4.

### D6: Apply writes `applied/` atomically; a systemd path unit restarts the consumers

Apply copies a `passed: true` result into `applied/` (a failing result answers 409, D12 b) and writes
`applied.json` last. Ansible installs `ros2-head-rgbd-calibration-apply.path` (`PathChanged=` on `applied.json`) and
a oneshot `.service` running `systemctl try-restart ros2-head_camera_cpp.service ros2-head_rgbd_cpp.service
ros2-head_state_publisher.service`. The node needs no privilege; it reports each unit's restart by reading
`systemctl show -p ActiveEnterTimestamp` (unprivileged) and comparing with the apply time. Alternative: Trigger
reload services in both C++ nodes (rejected: two C++ changes and a launch-file node that cannot reload its URDF).
Serves A1.

### D7: Nodes read the applied files; repo files only seed an empty `applied/`

client.yml sets head_camera_cpp `camera_info_file` and head_rgbd_cpp `tof_calibration_file` to the `applied/` paths;
the head_rgbd_cpp `config_files` copy into `/etc/ros2/head_rgbd_cpp/` is removed. Ansible copies a committed repo
calibration file into `applied/` only when the destination does not exist (`force: false`, `remote_src`), so a
reflashed robot falls back to the committed calibration and a deploy never overwrites an applied one. A missing file
keeps today's behaviour (no camera_info, head_rgbd_cpp publishes nothing). Serves A1, A7.

### D8: head_state_publisher applies `tof_joint.json` at launch

The launch file reads `applied/tof_joint.json` when present and rewrites the tof_joint origin of the repo head.urdf
with `rewrite_tof_joint`, loaded by file path from `shared/ros2_common/head_rgbd_calibration/urdf_text.py`
(importlib, stdlib only, because the launch runs on the system Python without the node venvs). Absent or invalid file:
the repo URDF unchanged, with a log line. Serves A1.

### D9: web_ui proxies; the IMU section is untouched

`CalibrationConfig.rgbd_url: str | None` (absent: no RGB-D section) and `rgbd_timeout_s`. `GET /api/calibration/config`
adds `rgbd: bool`. New module `rgbd_calibration.py` forwards method, path, query and body to the node and forwards
binary bodies (JPEG, zip) back, buffered (the zip is a few MB), plus the request content-type on POST, with their content type and `Content-Disposition`; an unreachable node answers 503
`{ok: false, message}`. IMU routes and their tests stay green. Serves A3, A6.

### D10: Frontend section with poll-on-press

`rgbdApi.ts` (the `call<T>` discriminated-union pattern of `imuApi.ts`), `RgbdCapture.tsx` (start/stop, status line,
pose count, thumbnail grid with delete through `ConfirmDialog`, RGB and ToF coverage grids), `RgbdSessions.tsx` (list,
reopen, compute) and `RgbdResults.tsx` (report table, limits, Apply through `ConfirmDialog`, Download link, applied
status). Polling every 1 s only while a capture or compute runs and the document is visible. Rendered by
`CalibratePage` when `config.rgbd`. Serves A2, A3, A6.

### D11: Docs and the overlap with head-rgb-d-fusion-of-camera

`docs/runbooks/head-rgbd-fusion.md` steps 2-3 describe the web UI flow (warm-up, board, capture, compute, Apply,
optional Download and commit). The in-progress task `head-rgb-d-fusion-of-camera/rgbd-final-suite-and-runbook`
(todo) commits the calibration files and sets `camera_info_file`; with this epic the robot's applied files are the
source of truth, the committed files are the downloaded copies that seed `applied/`, and `camera_info_file` points at
`applied/head_camera_calibration.yaml` (that task's grep AC on `head_camera_calibration.yaml` still holds). Registered
as the cross-epic contract `head-rgbd-applied-calibration`. Serves A7.

### D12: User-approved decisions at the gate

(a) Apply restarts head_camera_cpp, head_rgbd_cpp and head_state_publisher through a systemd path unit (D6) rather than hot reload in the C++ nodes (calib-node-apply-download, calib-node-ansible-deploy). (b) Apply is refused for a result that fails any limit (RGB, ToF or stereo RMS, view counts); the operator deletes poses or recaptures and recomputes (calib-node-apply-download a2, calib-node-http-api a6, rgbd-sessions-and-results-ui). (c) A capture stops by itself after 15 minutes (`max_capture_s` 900) so the head camera focus lock is released (calib-node-live-capture a4).

## Open questions

None. Q1-Q3 (restart through a path unit, refuse failing results, 15-minute auto-stop) were approved at the gate and are D12.

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| research-code-internal | ran (discovery lane 4, brief `discovery-calibrate.md` sections 3-5 and Plan inputs) | reuse map below; `photos.py` thumbnail and path-safety pattern; `agent_proxy.py` httpx proxy; `bno055_state_dir.yml` state dir | a new detector or solver (exists) |
| research-code-external | skipped: in-house glue around existing OpenCV calibration; no open fork that prior art settles | - | - |
| research-papers | skipped: no new algorithm; the maths is unchanged | - | - |
| research-docs | skipped: rclpy QoS, FastAPI, systemd path units and cv2.aruco are used as the repo already uses them; QoS facts were measured on the robot | systemd `PathChanged=` semantics confirmed at deploy (Assumptions) | - |

### Research inputs

| Source | Entry | Lands in |
|---|---|---|
| brief Plan inputs reuse | `imu_calibration.py:register_imu_calibration_routes`, `error`, `get_config` flag dict | web-ui-rgbd-calib-proxy |
| brief Plan inputs reuse | `server.py:call_ros_service`, `await_ros_future` | rejected: the RGB-D section talks HTTP to the node, not ROS services |
| brief Plan inputs reuse | `bridge.py:subscribe_always_on`, `SENSOR_SUB_QOS` | rejected: web_ui subscribes nothing for this section (D1); the node sets RELIABLE for ToF (D4) |
| brief Plan inputs reuse | `photos.py:scaled_jpeg`, `image_path` | calib-node-session-store (thumbnail and path-safety pattern) |
| brief Plan inputs reuse | `config.py:CalibrationConfig` (extend) | web-ui-rgbd-calib-proxy |
| brief Plan inputs reuse | frontend `CalibratePage.tsx` flags, `imuApi.ts:call`, `ImuCalibration.tsx` poll-on-press, `layout/ConfirmDialog` | rgbd-calib-frontend-api, rgbd-capture-section-ui, rgbd-sessions-and-results-ui |
| brief Plan inputs reuse | `charuco.make_board/detect`, `frames.*` | calib-core-config-charuco, calib-core-frames-intrinsics |
| brief Plan inputs reuse | intrinsics/extrinsics writers, `urdf_handoff` | calib-core-frames-intrinsics, calib-core-extrinsics-urdf |
| brief Plan inputs reuse | `pipeline.detect_tof/run_calibration/failed_limits/report_markdown` | calib-core-calibrate-run |
| brief Plan inputs reuse | `live.PoseCounter` | calib-pose-counter-in-core |
| brief Plan inputs reuse | `config.CalibrationConfig` (board, selection, limits) | calib-core-config-charuco, calib-node-scaffold-config |
| brief Plan inputs reuse | `shared/ros2_common` with `[tool.uv.sources]`, `src_shared: true` | calib-core-config-charuco, calib-node-scaffold-config, calib-node-ansible-deploy |
| brief Plan inputs reuse | `ansible/playbooks/tasks/bno055_state_dir.yml`; `config_files` copy | calib-node-ansible-deploy, head-nodes-use-applied-calib |
| brief do-not-build | new ChArUco detector, still-pose logic, solver, report generator | calib-core-config-charuco, calib-core-frames-intrinsics, calib-core-extrinsics-urdf, calib-core-calibrate-run (moved, not rewritten) |
| brief do-not-build | new IMU backend or frontend | rejected as work: IMU enabled in 053adfe3; regression ACs in web-ui-rgbd-calib-proxy and rgbd-capture-section-ui |
| brief do-not-build | ssh/scp capture path for the robot-side job | calib-node-live-capture subscribes with rclpy on the robot |
| brief risk | whole-bag decode, over 512M | calib-node-live-capture, calib-node-compute-job (poses only) |
| brief risk | output paths assume a dev repo; nodes read calibration only at startup | calib-node-apply-download, calib-node-ansible-deploy, head-nodes-use-applied-calib, head-urdf-tof-joint-override |
| brief risk | overlap with rgbd-final-suite-and-runbook | rgbd-calib-runbook-and-docs, contract head-rgbd-applied-calibration |
| brief risk | focus lock fires whenever the view is open | calib-node-live-capture (subscriptions only during capture) |
| brief risk | amplitude RELIABLE at the publisher | calib-node-live-capture |
| brief risk | `scripts.*` imports and `StrictModel` from `scripts.markers` | calib-core-config-charuco, calib-core-retire-shims |
| brief risk | `CalibrationConfig` name clash | calib-node-scaffold-config (node model `NodeConfig`), web-ui-rgbd-calib-proxy |
| brief risk | `scripts/` outside root lint | calib-core-config-charuco (the core moves under `shared/`, which root lint covers) |

## Irreversible operations

- Deleting a pose or (through the store only) a result removes files under `/var/lib/ros2/head_rgbd_calibration`;
  covered by a ConfirmDialog in the UI. Sessions are never deleted by this work.
- Apply replaces the files in `applied/`; the previous applied result stays in its session's `results/` dir and can be
  re-applied.
- `git rm` of `scripts/head_rgbd_calibration/live.py`, `live_relay.py`, `tests/test_head_rgbd_calib_live.py` and the
  transitional re-exports `scripts/head_rgbd_calibration/{charuco,intrinsics,extrinsics,urdf_handoff}.py` (reversible
  through git).
- Removing the head_rgbd_cpp `config_files` entry leaves a stale `/etc/ros2/head_rgbd_cpp/tof_calibration.yaml` only if
  one was ever deployed; nothing reads it after head-nodes-use-applied-calib.

## Migrated notes

### README.md: Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.

Verdict: pass (2026-10-20)

- E2E on the robot 2026-10-20: session calib-20261020-083900, 38 poses, RGB 0.10 px, ToF 0.28 px, stereo 0.28 px RMS; applied r-20261020-092536; head_rgbd publishes pairs (0 drops, valid ratio 0.42); user confirmed the RGB-D overlay is mostly right.
- Finding acted on: plane depth check measured the ToF's own depth error (0-40 mm long at 0.2-0.4 m), 15 mm refused a good calibration; limit 40 mm, background-contaminated pairs rejected (33a73511).
- Known residual: depth overlay shifts right for close objects (parallax from the ToF depth error); depth correction deferred to the obstacle work.

### design.md: Summary

The head RGB-D calibration (Camera Module 3 mapping stream + Arducam ToF, ChArUco board) moves from the Mac terminal
to the robot, driven from the web UI's hidden Calibrate page next to the IMU section. A new Python node,
`head_rgbd_calibration`, subscribes the head topics only while a capture session runs, stores only the usable still
poses in `/var/lib/ros2/head_rgbd_calibration`, computes the calibration in a niced child process, and applies a
result by copying it into `applied/`, which a systemd path unit turns into a restart of head_camera_cpp,
head_rgbd_cpp and head_state_publisher. web_ui proxies the node's localhost HTTP API under
`/api/calibration/rgbd/*`; the frontend shows the live status, thumbnails with corners, a coverage grid, past
sessions, reports, Apply and Download. The calibration maths moves unchanged into `shared/ros2_common`, the CLI keeps
working on top of it, and the ssh live counter is retired.

### design.md: Intent and why

The terminal flow failed repeatedly on the loaded robot (load ~28 on 4 cores): frames streamed over ssh were lost,
bags came back empty, and the operator could not see which poses were usable. The operator wants to stand at the
robot with the board and a phone or laptop browser, see each pose being stored, prune bad ones, compute, and apply
the result without a redeploy, with nothing streamed to the Mac.

### design.md: Goals

- G1: Start and stop a capture from the Calibrate page; it runs on the robot and stores only usable still poses
  (RGB JPEG, ToF amplitude, ToF depth, both detections, two thumbnails) under the state dir.
- G2: While capturing, the page shows the pose count, the current status line (`moving`, `still - no board in the RGB
  image`, `still - board not found by the ToF: move closer`, `still - no ToF frame close in time`,
  `pose N captured - move to the next pose`), and the thumbnails as poses are stored.
- G3: Delete single poses; see a coverage grid for the RGB and the ToF image; list past sessions, reopen any (view,
  delete, add poses) and compute on any.
- G4: Compute RGB intrinsics, ToF intrinsics, the stereo extrinsic, limits and the report on the robot from a session's
  stored poses, without blocking web_ui or starving the robot nodes; the page shows the report.
- G5: Apply a passing result: head_camera_cpp (camera_info), head_rgbd_cpp (ToF intrinsics) and head_state_publisher
  (tof_joint) use it after an automatic restart, no redeploy; the applied files survive Ansible deploys. Download a
  result as a zip of its YAMLs, the tof_joint origin, the rewritten head.urdf and the report.
- G6: The IMU section keeps working on the same page.
- G7: The CLI (`scripts/head_rgbd_calibrate.py` capture, capture-validation, calibrate, urdf, validate) keeps working on
  the shared core; `live.py`, `live_relay.py` and `capture.live` are gone.

### design.md: Non-goals

- No change to the board, the marker generator, the solvers or the limits.
- Validation captures (distances, wall, sweep, time offset) stay in the CLI.
- No frame streaming to the dev machine; no video stream in the page (thumbnails of stored poses only).
- The web UI's URDF viewer (`/api/urdf/head.urdf`) keeps serving the repo head.urdf; the applied tof_joint reaches
  TF consumers through head_state_publisher (my call).
- No hot-reload services in the C++ nodes (apply restarts them, D6).

### design.md: Anti-goals

- web_ui OOM or event-loop stalls caused by calibration work (web_ui MemoryMax 512M stays untouched by this work).
- The focus lock engaged while nobody captures (the mapping subscription must not outlive a capture).
- Placeholder data: no fabricated detections, thumbnails, coverage or camera_info; a missing value is absent.
- A deploy that overwrites an applied calibration.
- Decoding whole bags or holding a whole session in memory.

### design.md: Constraints

- Facts measured on the robot this session: CPU saturated by the normal stack; `/tof/amplitude/image_raw` and
  `/tof/depth/image_raw` are RELIABLE at the publisher and a BEST_EFFORT subscriber loses about half the frames, so the
  node subscribes them RELIABLE; `/head_camera/mapping/image_raw/compressed` is BEST_EFFORT 2 Hz 576x324 and any
  subscriber engages the focus lock (`head_camera_node.cpp:280-282`); RGB stillness uses `frames.rgb_difference`
  (mains flicker); ToF detection needs `detect_tof` (2x upscale) and the board within about 0.45 m; ROS processes need
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4` (node units already set it).
- Memory: web_ui MemoryMax 512M; store only selected poses; compute from them.
- Reuse, do not rebuild: `charuco.detect/make_board`, `frames.rgb_difference/tof_diff_fraction/normalise_amplitude/
  still_pairs`, `live.PoseCounter`, `pipeline.detect_tof/run_calibration/failed_limits/report_markdown`, the
  intrinsics/extrinsics writers, `urdf_handoff`.
- CLAUDE.md: type hints, pydantic config, constants at the top, no placeholder topic data, tests per function, README
  per node, `tests/README.md` updated, Ansible `ros2_nodes` schema in sync, `poe lint-ansible` / `test-ansible` green.
  web_ui stays Python; a new Python node is allowed because it reuses the Python core.
- Verification: root `uv run pytest tests/ -v`, `uv run poe lint`, `uv run poe lint-nodes`, `uv run poe lint-ansible`,
  `uv run poe test-ansible`; web_ui `cd nodes/web_ui && uv sync && uv run pytest tests/ -v`, `uv run poe lint`;
  frontend `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint`. Deploy is the orchestrator's,
  after merge, through `scripts/deploy-nodes.sh`.

### design.md: Decision coverage

| Decision | Tasks |
|---|---|
| D1 dedicated node, web_ui proxies | calib-node-scaffold-config, calib-node-http-api, calib-node-ansible-deploy, web-ui-rgbd-calib-proxy |
| D2 core in shared/ros2_common | calib-core-config-charuco, calib-core-frames-intrinsics, calib-core-extrinsics-urdf, calib-core-calibrate-run, calib-pose-counter-in-core, retire-ssh-live-counter, calib-core-retire-shims |
| D3 state dir layout | calib-node-session-store, calib-node-live-capture, calib-node-compute-job, calib-node-apply-download |
| D4 capture subscriptions and QoS | calib-node-live-capture, calib-pose-counter-in-core |
| D5 compute child process | calib-node-compute-job, calib-node-http-api |
| D6 apply and path unit restart | calib-node-apply-download, calib-node-ansible-deploy |
| D7 nodes read applied files | head-nodes-use-applied-calib, calib-node-ansible-deploy |
| D8 head_state_publisher tof_joint override | head-urdf-tof-joint-override, calib-core-extrinsics-urdf |
| D9 web_ui proxy, IMU untouched | web-ui-rgbd-calib-proxy, head-nodes-use-applied-calib |
| D10 frontend section | rgbd-calib-frontend-api, rgbd-capture-section-ui, rgbd-sessions-and-results-ui |
| D11 docs and overlap | rgbd-calib-runbook-and-docs |
| D12 user-approved decisions | calib-node-apply-download, calib-node-ansible-deploy, calib-node-http-api, calib-node-live-capture, rgbd-sessions-and-results-ui |

### design.md: Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `rgbd-calib-core` | calib-core-config-charuco | calib-core-frames-intrinsics, calib-core-extrinsics-urdf, calib-core-calibrate-run, calib-pose-counter-in-core, calib-core-retire-shims, calib-node-scaffold-config, calib-node-session-store, calib-node-compute-job, head-urdf-tof-joint-override | `shared/ros2_common/head_rgbd_calibration/config.py` |
| `rgbd-pose-counter` | calib-pose-counter-in-core | calib-node-live-capture | `shared/ros2_common/head_rgbd_calibration/poses.py` |
| `rgbd-calib-node-config` | calib-node-scaffold-config | calib-node-session-store, calib-node-live-capture, calib-node-compute-job, calib-node-apply-download, calib-node-http-api | `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py` |
| `rgbd-calib-session-store` | calib-node-session-store | calib-node-live-capture, calib-node-compute-job, calib-node-apply-download, calib-node-http-api | `nodes/head_rgbd_calibration/head_rgbd_calibration/store.py` |
| `rgbd-calib-result-files` | calib-node-compute-job | calib-node-apply-download, calib-node-http-api | `nodes/head_rgbd_calibration/head_rgbd_calibration/compute.py` |
| `rgbd-calib-applied-files` | calib-node-apply-download | head-urdf-tof-joint-override, calib-node-ansible-deploy, head-nodes-use-applied-calib | `nodes/head_rgbd_calibration/head_rgbd_calibration/apply.py` |
| `rgbd-calib-node-http-api` | calib-node-http-api | calib-node-ansible-deploy, web-ui-rgbd-calib-proxy | `nodes/head_rgbd_calibration/head_rgbd_calibration/app.py` |
| `rgbd-calib-web-api` | web-ui-rgbd-calib-proxy | head-nodes-use-applied-calib, rgbd-calib-frontend-api | `nodes/web_ui/web_ui/rgbd_calibration.py` |
| `rgbd-calib-frontend-api` | rgbd-calib-frontend-api | rgbd-capture-section-ui, rgbd-sessions-and-results-ui | `nodes/web_ui/frontend/src/calibrate/rgbdApi.ts` |

Cross-epic: `head-rgbd-applied-calibration` (registered in `docs/plans/contracts.json`), owner task head-nodes-use-applied-calib, defined in `ansible/group_vars/client.yml`; consumer epic head-rgb-d-fusion-of-camera (its task rgbd-final-suite-and-runbook).

### design.md: Node HTTP API

Contract `rgbd-calib-node-http-api` (all JSON errors `{ok: false, message}`):

| Method and path | Answer |
|---|---|
| GET `/api/state` | `{capture: {running, session_id, status, pose_count, started_at, stops_at}, compute: {running, session_id, result_id, started_at, error}, applied}` |
| POST `/api/capture/start` `{session_id?}` | 200 state; 404 unknown session; 409 capture or compute running |
| POST `/api/capture/stop` | 200 state |
| GET `/api/sessions` | `[{id, created, pose_count, result_count}]`, newest first |
| GET `/api/sessions/{sid}` | `{id, created, lens_position, poses: [{id, rgb_corners, tof_corners, has_depth}], coverage: {rgb, tof}, results: [result.json]}`; coverage `{cols, rows, counts}` |
| DELETE `/api/sessions/{sid}/poses/{pid}` | 200; 404 |
| GET `/api/sessions/{sid}/poses/{pid}/rgb.jpg`, `/tof.jpg` | image/jpeg thumbnail; 404 |
| POST `/api/sessions/{sid}/compute` | 202 `{result_id}`; 409 busy or fewer poses than `limits.min_views` |
| GET `/api/sessions/{sid}/results/{rid}` | result.json plus `report` (markdown) |
| GET `/api/sessions/{sid}/results/{rid}/download` | application/zip `<sid>-<rid>.zip` |
| POST `/api/sessions/{sid}/results/{rid}/apply` | 200 applied state; 409 not passed or busy |
| GET `/api/applied` | applied.json plus `units: [{name, restarted}]`, or 404 when nothing is applied |

web_ui maps `/api/calibration/rgbd/<rest>` to node `/api/<rest>`.

### design.md: Review focus

- `calib-node-live-capture`: subscription lifetime (nothing subscribed after stop, timeout or a start failure), RELIABLE
  QoS on ToF, thread safety between the executor thread, the HTTP handlers and the store.
- Path safety of every id in the store and the proxy (no traversal outside `state_dir`).
- Atomic writes of results and `applied/` (a crash mid-apply never leaves a mixed calibration).
- The core move is behaviour-neutral: existing calibration tests pass with only import changes.
- No placeholder data: a pose without a ToF detection is never stored, a missing depth stays absent.

### design.md: Left to the builder

- Internal module split inside the node beyond the files named in the tasks, provided the contracts hold.
- Thumbnail JPEG quality and the exact corner drawing style.
- Coverage grid size within 3x2 to 6x4 (default 4x3).
- MUI layout of the section, provided touch targets use `TOUCH_TARGET_PX` and the design-rules tests pass.

### design.md: Assumptions and unknowns

- systemd `PathChanged=` fires on an `os.replace` of `applied.json` (inotify `IN_MOVED_TO` is in its mask); confirmed on
  the robot at deploy by applying a result and reading the units' `ActiveEnterTimestamp` (the node reports it).
- `/etc/ros2/head_camera_cpp/config.yaml` is readable by ansible_user (node config files are 0644); the node falls back
  to the head_camera_cpp default `lens_position` 1.0 only when the key is absent, never when the file is unreadable
  (then capture start fails with the reason).
- The node venv uses `--system-site-packages` for rclpy like every native Python node (role main.yml:201).

### approvals.json

- A1: Results live on the robot: the computed calibration is written to a robot state dir (/var/lib/ros2/head_rgbd_calibration); an Apply button makes head_camera_cpp (camera_info), head_rgbd_cpp (ToF calibration) and the head URDF tof_joint use it without a redeploy (restart triggered on the robot), it survives Ansible deploys, and a Download button returns the result YAMLs so they can be committed. [decisions: D3, D6, D7, D8, D12] [tasks: calib-node-session-store, calib-node-compute-job, calib-node-apply-download, calib-node-http-api, head-urdf-tof-joint-override, calib-node-ansible-deploy, head-nodes-use-applied-calib, rgbd-sessions-and-results-ui] [status: approved]
- A2: Everything runs on the robot and nothing is streamed to the Mac; the operator sees live the pose count, the current status (moving / no board in RGB / board not found by the ToF: move closer / pose N captured) and the thumbnails as poses are stored. [decisions: D1, D3, D4, D10] [tasks: calib-pose-counter-in-core, calib-node-live-capture, calib-node-http-api, web-ui-rgbd-calib-proxy, rgbd-calib-frontend-api, rgbd-capture-section-ui] [status: approved]
- A3: Pose list features, all required: RGB and ToF thumbnails with the detected corners drawn; delete single poses; a coverage hint showing which image areas of RGB and ToF the stored poses cover; past sessions kept (list them, reopen any, recompute on any). [decisions: D3, D4, D5, D9, D10] [tasks: calib-node-session-store, calib-node-live-capture, calib-node-compute-job, calib-node-http-api, web-ui-rgbd-calib-proxy, rgbd-calib-frontend-api, rgbd-capture-section-ui, rgbd-sessions-and-results-ui] [status: approved]
- A4: The calibration is computed on the robot from the stored poses only, without decoding bags, without running in the web_ui event loop or its 512M budget, and without starving the robot nodes (own cgroup, niced child process). [decisions: D1, D5] [tasks: calib-node-scaffold-config, calib-node-compute-job, calib-node-ansible-deploy] [status: approved]
- A5: The calibration core is reused, not rebuilt: it moves into shared/ros2_common, importable from node venvs; the existing CLI keeps working on top of it; live.py, live_relay.py and capture.live are removed with a retirement test. [decisions: D2] [tasks: calib-core-config-charuco, calib-core-frames-intrinsics, calib-core-extrinsics-urdf, calib-core-calibrate-run, calib-pose-counter-in-core, retire-ssh-live-counter, calib-core-retire-shims] [status: approved]
- A6: The IMU section, enabled on the robot in 053adfe3, keeps working on the same Calibrate page. [decisions: D9, D10] [tasks: web-ui-rgbd-calib-proxy, rgbd-capture-section-ui, head-nodes-use-applied-calib] [status: approved]
- A7: camera_info_file and tof_calibration_file point at the applied state files, with committed repo files only seeding an empty applied dir; docs/runbooks/head-rgbd-fusion.md steps 2-3 describe the web UI flow; the overlap with head-rgb-d-fusion-of-camera (rgbd-final-suite-and-runbook) is noted in design.md. [decisions: D7, D11] [tasks: head-nodes-use-applied-calib, calib-node-ansible-deploy, rgbd-calib-runbook-and-docs] [status: approved]
