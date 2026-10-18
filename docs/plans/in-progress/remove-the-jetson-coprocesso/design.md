# Remove the Jetson coprocessor design

## Summary

The Jetson Nano leaves the robot, so everything that exists only for it is deleted: the `jetson/` tree, the
`jetson_base`, `jetson_depthd` and `jetson_link` roles, the `jetson` deploy target, `nodes/jetson_bridge_cpp`, the
retired Pi-side `stereo_depth` / `stereo_depth_cpp` sources, `depth_wire` in `ros2_common_cpp`, and every `/stereo/*`
and `/overview_camera/*` consumer. The web_ui Head tab shows the Arducam ToF (depth + amplitude, no RGB); the Arm tab
and mcp_server use `/front_camera/image_raw/compressed`; the Overview tab and the stereo calibration proxy go. On the
client, `jetson_bridge_cpp` becomes a `present: false` entry whose uninstall also purges its colcon output, and the
client side of the Jetson cable is undone by an idempotent removal under the existing `jetson_link` tag, with chrony
kept on plain internet pools.

## Intent and why

The user is physically removing the Jetson; the ToF camera (arducam_tof, CSI cam1) replaces stereo depth. Dead code,
dead deploy paths and a Head tab pointed at topics nobody publishes would mislead every later change and leave
NAT/forwarding/static-IP state on the Pi that no longer serves anything. Doing it now, while the inventory is fresh
(discovery brief), keeps the removal complete and the deployed client clean.

## Goals

- No Jetson code, roles, playbooks, deploy target, scripts, CI jobs, tests or live docs remain (docs/plans archive
  untouched); a suite test greps for live references against a categorised allowlist.
- `nodes/stereo_depth`, `nodes/stereo_depth_cpp` (and their node_cpu_compare entry) and `depth_wire` are deleted.
- Head tab: `/tof/depth/image_raw` + `/tof/amplitude/image_raw`, no RGB; `fused_depth_topic` and the raw/fused toggle
  removed from backend and frontend code.
- Arm tab and mcp_server `front` camera on `/front_camera/image_raw/compressed`; Overview tab removed.
- Calibrate page IMU-only: no `depthd_url`, no stereo proxy, no `stereoApi`.
- `jetson_bridge_cpp` stays as a `present: false` entry; its uninstall purges `/opt/ros2-ws/{src,build,install}/jetson_bridge_cpp`
  and the `.built-jetson_bridge_cpp-*` stamps.
- Client jetson_link state removed idempotently (eth0 static stanza, hosts line, ip_forward drop-in, nftables
  table/file/include); chrony kept with plain pools.
- `network_link_ethernets` / `network_netplan_superseded_files` and monitoring `host_targets` / `target.host` removed.
- All suites green, docs/MEMORY/skills updated, client deployed.

## Non-goals

- Fixing Arducam ToF frame capture on the Pi 5 kernel (tracked separately).
- Rewriting `docs/plans` or `docs/superpowers` archive files.
- Removing the retired `overview_camera` entry or `pi_cameras`; the retired `stereo_depth` / `stereo_depth_cpp`
  `present: false` entries and playbook blocks also stay (uninstall path, D7).
- Purging `ros2_common_cpp` from the client workspace: `map_guard_cpp` (present, enabled) still builds against it.

## Anti-goals

- Leaving any suite red between tasks or at the end (every task verifies its own targeted tests).
- Breaking the IMU calibration routes while deleting the stereo proxy they import helpers from.
- A removal that is not idempotent, or that touches host state the Jetson link did not create (e.g. disabling the
  nftables service, or forcing `ip_forward=0` while docker runs).
- Rebuilding libcamera / camera_ros on the client: the `nodes/pi_cameras/patches` content (which still names
  `overview_camera` as maintainer) is part of the pi_cameras build stamp and is not edited.
- Scope growth: no new features on the Head tab beyond what the goals name.

## Constraints

- Repo conventions (CLAUDE.md): type hints, pydantic config, tests for each change, tests/README.md updated with every
  test change, READMEs per area, semantic commits, push to main, deploy only via `scripts/deploy-nodes.sh` logged to
  `.logs/` after the ansible-deploy skill; never run `ansible-playbook` against a host directly.
- Ansible must keep `name`/`node_type`/`present`/`enabled`/`config` for every node; `uv run poe test-ansible` green.
- Verification commands: root `uv run pytest tests/ -q`, `uv run poe lint`, `uv run poe lint-nodes`,
  `uv run poe test-ansible`, `uv run poe test-cpp` (docker, available); web_ui
  `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint`; frontend
  `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build` (package.json scripts:
  `test` = `vitest run`, `build` = `tsc && vite build`, `lint` = eslint); mcp_server
  `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint`.
- Deploying mcp_server/claude_agent kills a running agent session (user memory): check before deploying.
- Baseline today: root 1185 passed / 9 skipped, web_ui 765, mcp_server 2464, frontend 898 tests.

## Decisions

### D1: Delete the stereo_depth and stereo_depth_cpp sources, keep their retired entries

`nodes/stereo_depth` and `nodes/stereo_depth_cpp` are deleted with their lint/CI/node_cpu_compare entries. Their
`present: false` ros2_nodes entries, uninstall-only node types and playbook blocks stay (the D7 uninstall path, pinned
by `tests/test_stereo_depth_config.py`). Alternative: keep sources as retired (rejected by the user, decision 1).
Serves: U1, G2.

### D2: Remove fused_depth_topic and the raw/fused toggle from code

`TabConfig.fused_depth_topic`, its role and tof range, the frontend type field, the HeadTab toggle state and its tests
are deleted, not just unset in config. Alternative: config-only (rejected, decision 2). Serves: U2, G3.

### D3: Client time sync is a new `chrony` role with plain internet pools

A small `ansible/roles/chrony` (pools from the old `jetson_link_ntp_pools`, timesyncd stopped/disabled when present,
restart only on change) is applied by client provisioning (`playbooks/client.yml`) and by the link-removal block, so
the deployed client gets the plain-pool config now and a reprovisioned one later (my call: role placement). Its
config has no `allow`, `local stratum` or Jetson server line. Alternative: back to systemd-timesyncd (rejected,
decision 3). Serves: U3, G7.

### D4: Remove the generic link and host-target features

The network role loses `network_link_ethernets`, `network_netplan_superseded_files`, the link address probe and the
superseded-file removal; monitoring_agent loses `monitoring_agent_host_targets`, the host scrape block and the
per-target `host` override. Alternative: keep them generic and empty (rejected, decision 4). Serves: U4, G8.

### D5: Head tab is ToF depth + amplitude, no RGB, no camera_info subscription

Both `default.yaml` and `client.yml` head tabs set `tof_depth_topic: /tof/depth/image_raw`,
`tof_amplitude_topic: /tof/amplitude/image_raw`, keep `tof_range_m: 4.0`, and have no `rgb_camera_topic`. The ToF's
`/tof/camera_info` is documented as published by arducam_tof, but the Head tab does not subscribe it (my call): the
tab has no camera_info consumer, and `camera_info_topic` would only stream unused intrinsics over the websocket.
Serves: U5, G3.

### D6: Arm tab field renamed to front_camera_topic; Overview tab removed; claude_agent unchanged

`TabConfig.overview_camera_topic` becomes `front_camera_topic` (my call: needed so no live `overview_camera` name
remains) defaulting to `/front_camera/image_raw/compressed`, in backend, both configs and the frontend. The
`overview_camera` camera tab is deleted from both configs and the `/overview_camera` type hint from the bridge.
mcp_server `topics.front_camera` (default and client.yml) moves to `/front_camera/image_raw/compressed`; its docs
describe the pi_cameras front Camera Module 3 (640x480 at 10 fps) instead of the left IMX219 (480x360 at 5 Hz); the
`front` role wording ("fixed overhead camera") is kept (my call: mount unchanged, `cameras.front` mount is null).
claude_agent needs no code change: it reaches cameras through mcp_server and its `camera_note` already describes a
640x480 autofocus front camera. Serves: U5, G4.

### D7: Calibrate is IMU-only; /api/calibration/config moves into imu_calibration.py

`calibration_proxy.py` and its tests are deleted; `GET /api/calibration/config` is served by `imu_calibration.py`
and returns `{"imu": true, "arm": "deferred"}` (my call: the existing `arm` key stays); `NOT_CONFIGURED_MESSAGE` and
`error()` move there. `CalibrationConfig` drops `depthd_url` and the three proxy-only timeouts
(`connect_timeout_s`, `read_timeout_s`, `solve_read_timeout_s`, my call: they configured only the proxy), so
`calibration: {}` turns the page on. Frontend: `stereoApi` and `StereoCalibration` deleted; `CalibratePage` reads
only `imu`. Serves: U6, G5.

### D8: jetson_bridge_cpp becomes a present:false entry; its sources are deleted in this epic

The node type keeps only `deploy_mode`, `node_launch_command` and `colcon_purge`; the entry is `present: false` with no
config; the playbook block stays so the next deploy runs the uninstall. The uninstall path never reads the node's
source (no `colcon_source`, `node_src_dir` or `src_paths` on the type), so `nodes/jetson_bridge_cpp` is deleted in the
same epic. Alternative: delete the entry after the uninstall deploy (rejected, decision 7). Serves: U7, G6, G1.

### D9: colcon purge is an explicit per-type `colcon_purge` list handled by the uninstall block

New node type key `colcon_purge: [{package, workspace}]` (my call: explicit, so shared packages such as
`ros2_common_cpp` are never purged by accident), passed as `node_colcon_purge` by `resolve_and_deploy.yml`, and a new
`ros2_node_deploy/tasks/colcon_purge.yml` included (with `become` applied) from the existing uninstall block. It
removes `<ws>/src/<pkg>`, `<ws>/build/<pkg>`, `<ws>/install/<pkg>` and `<ws>/.built-<pkg>-*`, nothing else.
Alternative: a new role (rejected by the reuse brief: extend the uninstall block). Serves: U8, G6.

### D10: The client link removal is a task file under the existing `jetson_link` tag

`playbooks/tasks/jetson_link_remove.yml` (state-based, pattern of `monitoring_agent/tasks/remove_local_stack.yml`)
removes the `jetson.ros2.lan` hosts line, the forwarding sysctl drop-in (live `ip_forward=0` only when the drop-in
was removed and docker is not running), the `ip jetson_link` nftables table (only when listed), its file, the include
line and `/etc/nftables.d` when empty; the nftables package and service are left as they are (my call: their state
before the role ran is unknown and an empty ruleset is harmless). The block in `deploy_nodes_client.yml` keeps the
tag `jetson_link` (my call: `deploy-nodes.sh client jetson_link`, `select_run.yml` and the deploy-speed test stay
unchanged) and also applies the network role (netplan re-render without the eth0 static stanza) and the chrony role.
Serves: G7, U3.

### D11: Retirement is pinned by data-driven tests plus a live-reference grep test

`tests/test_jetson_retired.py` holds `REMOVED_PATHS` and `ABSENT_TOKENS` with two parametrized tests; later tasks
extend the lists (no new tests/README rows needed). The jetson_bridge_cpp retirement adds shape tests copied from
`tests/test_stereo_depth_config.py`. The final task adds `test_no_live_jetson_stereo_or_overview_reference`: a
`git grep` whose every hit must match a narrow, categorised allowlist entry. Serves: G1, G9.

### D12: The jetson deploy target goes first; generic tests move to the deploy-speed test

`deploy-nodes.sh` accepts only `client` and `server`; `deploy_nodes_jetson.yml`, `group_vars/jetson.yml`, the
inventory example group and the pyproject syntax check go. `test_deploy_script_uses_no_associative_arrays` and
`test_provision_plays_target_server_and_client_only` move from `test_jetson_ansible.py` into
`test_ansible_deploy_speed.py`. Serves: G1.

### D13: depth_wire and zstd leave ros2_common_cpp together with the map_guard_cpp apt entry

Deleting `depth_wire` drops the zstd dependency from `CMakeLists.txt` and `package.xml`, and `libzstd-dev` from the
`map_guard_cpp` apt list; the changed tree changes the stamp, so `map_guard_cpp` rebuilds on the next deploy
(expected). Serves: G2.

### D14: Deploy is the orchestrator's post-merge step, from a runbook in the final task

Tasks run in worktrees before merge, so the deploy (and the local untracked edits: `[jetson]` out of
`ansible/inventory`, removing the untracked `nodes/overview_camera/` pycache directory, the out-of-repo auto-memory
`project_jetson_vision_coprocessor.md`) is run by the orchestrator after merge, from the exact runbook in
`final-suite-and-grep`'s Notes. Serves: G9.

### D15: One linear task chain

`tests/README.md`, `ansible/README.md`, `client.yml` and `nodes/web_ui/README.md` are shared by nearly every task, so
tasks run in one dependency chain (my call). Serves: G9.

## Research

| Lens | Ran / skipped | Adopted | Rejected |
|---|---|---|---|
| discovery (codebase map) | ran: `jetson-removal-brief.md` (git grep; graphify broken, LSP unused) | full inventory with file:line; client-side link state list (section 2); Head tab contract (section 3); verification commands (section 4) | moving the Calibrate page off entirely (user chose IMU-only) |
| reuse (research-code-internal) | ran: `jetson-removal-reuse.md` | `ros2_node_deploy/tasks/main.yml` uninstall block extended for colcon purge; retired-type shape `client.yml` stereo entries; retirement tests from `tests/test_stereo_depth_config.py` and `tests/test_pi_cameras_retired.py`; host cleanup pattern `monitoring_agent/tasks/remove_local_stack.yml`; lineinfile absent `playbooks/tasks/pi_camera_boot_config.yml:8`; HeadTab and `msg_serializer.py` ToF rendering unchanged | a new uninstall role; a camera_info consumer on the Head tab (D5) |
| research-code-external | skipped: pure removal, no new external API or algorithm | - | - |
| research-papers | skipped: no algorithmic core | - | - |
| research-docs | skipped: no library used in a new way; the chrony role reuses the existing chrony.conf directives and the purge uses core `file`/`find` modules already used in the repo | - | - |

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| reuse brief, reuse | node uninstall path `ros2_node_deploy/tasks/main.yml:65` | purge-colcon-on-uninstall, retire-jetson-bridge-entry |
| reuse brief, reuse | retired-type shape `client.yml:364-372`, `:1151-1160` | retire-jetson-bridge-entry |
| reuse brief, reuse | retirement test template `tests/test_stereo_depth_config.py` | retire-jetson-bridge-entry |
| reuse brief, reuse | files-gone assertion `tests/test_pi_cameras_retired.py:test_old_boot_file_and_old_tests_are_deleted` | delete-jetson-tree |
| reuse brief, reuse | state-based host cleanup `monitoring_agent/tasks/remove_local_stack.yml` | remove-jetson-link-on-client |
| reuse brief, reuse | lineinfile `state: absent` `playbooks/tasks/pi_camera_boot_config.yml:8` | remove-jetson-link-on-client |
| reuse brief, reuse | HeadTab ToF rendering `HeadTab.tsx:HeadTab` | head-and-arm-tabs-frontend |
| reuse brief, reuse | 16UC1 greyscale depth `msg_serializer.py:_serialize_depth_image` | head-tab-tof-topics |
| reuse brief, reuse | mono16 amplitude `msg_serializer.py:_serialize_image` | head-tab-tof-topics |
| reuse brief, reuse | head roles/throttle `bridge.py:head_spec`, `config.py:tof_ranges` | drop-fused-depth-topic |
| reuse brief, reuse | HeadTab tests already on `/tof/depth/image_raw` | head-and-arm-tabs-frontend |
| reuse brief, do-not-build | ToF depth/amplitude renderer | rejected: exists (HeadTab + msg_serializer); no task builds one |
| reuse brief, do-not-build | C++ colcon uninstall role | rejected: no new role; purge-colcon-on-uninstall extends the uninstall block |
| reuse brief, do-not-build | amplitude topic config field | rejected: exists (`TabConfig.tof_amplitude_topic`); head-tab-tof-topics only sets it |
| reuse brief, fix-as-ac | C++ node colcon leftovers | purge-colcon-on-uninstall, retire-jetson-bridge-entry |
| reuse brief, fix-as-ac | jetson_link client state | client-chrony-role, remove-jetson-link-on-client, drop-network-link-support |
| reuse brief, fix-as-ac | IMU calibration coupling to the proxy | move-calibrate-config-to-imu, calibration-config-no-depthd |
| reuse brief, fix-as-ac | `/overview_camera` consumers | mcp-front-camera-topic, remove-overview-tab, arm-tab-front-camera-field, head-and-arm-tabs-frontend |
| reuse brief, fix-as-ac | zstd dependency | drop-depth-wire-and-zstd |
| reuse brief, ideas | adopted: copy the test_stereo_depth_config.py layout | retire-jetson-bridge-entry |
| reuse brief, ideas | open: Head tab has no camera_info consumer | rejected: not subscribed (D5); documented in head-tab-tof-topics |
| reuse brief, ideas | open: Head rgb_camera_topic replacement | head-tab-tof-topics (no RGB, decision 5) |
| reuse brief, ideas | noted: stereo_depth / stereo_depth_cpp / overview_camera sources still in tree | delete-stereo-depth-node, delete-stereo-depth-cpp-node; the untracked `nodes/overview_camera` pycache is a runbook step in final-suite-and-grep |
| discovery brief, 1 | jetson tree, scripts, roles, playbook, group vars, inventory, deploy script, pyproject, CI | remove-jetson-deploy-target, delete-jetson-tree, delete-jetson-host-roles, delete-jetson-bridge-sources |
| discovery brief, 1 | depth_wire users only depthd and jetson_bridge_cpp | drop-depth-wire-and-zstd |
| discovery brief, 1 | web_ui backend/frontend and mcp_server edits | move-calibrate-config-to-imu, calibration-config-no-depthd, calibrate-page-imu-only, mcp-front-camera-topic, drop-fused-depth-topic, head-tab-tof-topics, remove-overview-tab, arm-tab-front-camera-field, head-and-arm-tabs-frontend |
| discovery brief, 1 | monitoring host targets and target.host (commit 12d44b48) | drop-monitoring-host-targets |
| discovery brief, 1 | READMEs, SKILL.md, MEMORY.md stale sections | docs-and-memory-sweep, plus each owning task |
| discovery brief, 2 | client link state 1-6 and their undo | remove-jetson-link-on-client, client-chrony-role, drop-network-link-support, drop-monitoring-host-targets |
| discovery brief, 2 | colcon output and stamp not removed by the uninstall | purge-colcon-on-uninstall |
| discovery brief, 3 | Head tab schema and arducam_tof topics | drop-fused-depth-topic, head-tab-tof-topics, head-and-arm-tabs-frontend |
| discovery brief, 4 | verification commands and baselines | final-suite-and-grep |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 Delete stereo sources, keep retired entries | delete-stereo-depth-node, delete-stereo-depth-cpp-node |
| D2 Remove fused_depth_topic and toggle | drop-fused-depth-topic, head-and-arm-tabs-frontend |
| D3 chrony role with plain pools | client-chrony-role, remove-jetson-link-on-client |
| D4 Remove link and host-target features | drop-network-link-support, drop-monitoring-host-targets |
| D5 Head tab ToF, no RGB | head-tab-tof-topics, head-and-arm-tabs-frontend |
| D6 Arm front_camera_topic, Overview removed, mcp front camera | mcp-front-camera-topic, remove-overview-tab, arm-tab-front-camera-field, head-and-arm-tabs-frontend |
| D7 Calibrate IMU-only | move-calibrate-config-to-imu, calibration-config-no-depthd, calibrate-page-imu-only |
| D8 jetson_bridge_cpp present:false, sources deleted | retire-jetson-bridge-entry, delete-jetson-bridge-sources |
| D9 colcon_purge key | purge-colcon-on-uninstall, retire-jetson-bridge-entry |
| D10 Link removal under jetson_link tag | remove-jetson-link-on-client |
| D11 Retirement tests and live-reference grep | delete-jetson-tree, retire-jetson-bridge-entry, final-suite-and-grep |
| D12 Jetson deploy target removed | remove-jetson-deploy-target, delete-jetson-host-roles |
| D13 depth_wire and zstd removed | drop-depth-wire-and-zstd |
| D14 Deploy runbook after merge | final-suite-and-grep, docs-and-memory-sweep |
| D15 Linear chain | final-suite-and-grep |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `jetson-retired-test` | delete-jetson-tree | delete-jetson-host-roles, retire-jetson-bridge-entry, delete-jetson-bridge-sources, drop-depth-wire-and-zstd, final-suite-and-grep | `tests/test_jetson_retired.py` |
| `colcon-purge-key` | purge-colcon-on-uninstall | retire-jetson-bridge-entry | `ansible/roles/ros2_node_deploy/tasks/colcon_purge.yml` |
| `chrony-role` | client-chrony-role | remove-jetson-link-on-client | `ansible/roles/chrony/tasks/main.yml` |
| `calibration-config-endpoint` | move-calibrate-config-to-imu | calibrate-page-imu-only | `nodes/web_ui/web_ui/imu_calibration.py` |
| `arm-front-camera-field` | arm-tab-front-camera-field | head-and-arm-tabs-frontend | `nodes/web_ui/web_ui/config.py` |

## Review focus

- `colcon_purge.yml`: removes only the named package's src/build/install and its own stamps; never `install/setup.bash`,
  the workspace root or `ros2_common_cpp`.
- `jetson_link_remove.yml`: every task idempotent and guarded; `ip_forward=0` only when the drop-in was removed and
  docker is not running; nftables table deleted only when present; package/service untouched.
- The netplan re-render for the client after `network_link_ethernets` goes: WiFi stanza byte-identical to before.
- The IMU calibration routes keep working after `calibration_proxy.py` is deleted (helpers moved, not lost).
- The live-reference allowlist: every entry narrow (file + line regex) and in one of the named categories.
- `test_mcp_server_config.py` couples client.yml, both node READMEs and mcp_server sources: each task that edits one
  of those keeps it green.

## Left to the builder

- Exact wording of README/MEMORY/SKILL text, within the facts in this design.
- Whether `STANDALONE_CPP_TYPES` in `test_cpp_node_deploy.py` becomes empty or is removed with its parametrized tests.
- How the purge behaviour test drives the task file (a local `ansible-playbook -c local` run against a temp workspace
  is preferred; a static YAML check is the minimum, and both the include and the mapping must be asserted).
- The allowlist entries of the final grep test, within the categories in final-suite-and-grep.

## Assumptions and unknowns

- After the network role re-renders without the link, eth0 comes up as a DHCP `optional` secondary and wlan0 stays
  primary (not verified): confirmed after deploy with a read-only `ansible client -m command -a "ip -4 addr show eth0"`.
- Installing chrony probably removed the systemd-timesyncd package; the chrony role guards on the unit's presence, so
  either state works.
- No docker or other forwarder needs `ip_forward=1` on the client (discovery brief); the removal still guards on
  docker.service.
- nftables state before the role ran is unknown; the removal leaves the package and service as they are.
- The front Camera Module 3 is where mcp_server's "fixed overhead" wording says (mount `null`, no calibration to
  invalidate); a user look at `/front_camera` in the Front tab after deploy confirms it.

## Irreversible operations in scope

- Deleting tracked trees (`jetson/`, three roles, `nodes/jetson_bridge_cpp`, `nodes/stereo_depth`,
  `nodes/stereo_depth_cpp`, depth_wire files, Jetson tests and scripts): recoverable from git history; approved by the
  README goals and decision 1.
- On the client (post-merge deploy): uninstalling `jetson_bridge_cpp` and purging its colcon output (rebuildable from
  history), removing the jetson_link hosts/sysctl/nftables/netplan state and rewriting chrony.conf: approved by
  decisions 3, 7, 8 and the README goals.
- Local untracked edits: the `[jetson]` group in `ansible/inventory` and the untracked `nodes/overview_camera/`
  pycache directory (not in git; regenerable).
