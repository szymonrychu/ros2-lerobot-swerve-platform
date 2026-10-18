---
title: RGBD final suite and runbook
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The fusion is proven on the robot: calibrated cameras, measured time offset, depth accuracy at known distances, alignment, CPU, and no placeholder output."
files: [docs/runbooks/head-rgbd-fusion.md, nodes/head_camera_cpp/config/head_camera_calibration.yaml, nodes/head_rgbd_cpp/config/tof_calibration.yaml, nodes/head_rgbd_cpp/config/head_rgbd_extrinsics.yaml, nodes/web_ui/urdf/head.urdf, ansible/group_vars/client.yml, docs/head-rgbd-fusion-methodology.md]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && uv run poe lint-cpp && (cd nodes/bridges/arducam_tof && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint)"
depends_on: [rgbd-methodology-doc]
owns: []
uses: []
model: sonnet
estimate: 90
actual: ""
review: ""
branch: head-rgb-d-fusion-of-camera--rgbd-final-suite-and-runbook
verified_head: ""
run: ""
provides: []
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-fusion-of-camera--rgbd-final-suite-and-runbook
---

# RGBD final suite and runbook

## Requirement

- Full suite green (the verify command, every line read).
- `docs/runbooks/head-rgbd-fusion.md` (steps plus a `## Results` section), written and then executed with results filled in (date, numbers, pass/fail per step), logs in `.logs/`, deploys only through `scripts/deploy-nodes.sh` after invoking the ansible-deploy skill:
  1. Deploy `./scripts/deploy-nodes.sh client arducam_tof head_camera_cpp head_rgbd_cpp` (head_rgbd_cpp publishes nothing yet: no ToF calibration; confirm with `ros2 topic hz` that `/head_rgbd/*` is silent: placeholder check part 1).
  2. Warm up the ToF for 20 minutes. Print the ChArUco board (6x5, 40 mm squares) with the marker-and-calibration-board generator (`uv run python -m scripts.markers_generate`): the PDF at 100 % on matte laser paper mounted on a rigid plate (or the two-colour STL/3MF); check one square with a caliper (40 +- 0.2 mm). `python scripts/head_rgbd_calibrate.py capture` with 20-30 still poses (0.5-1.5 m, tilts, corners); `calibrate`; limits met (RGB <= 0.5 px, ToF <= 0.3 px, stereo <= 1.0 px, plane bias <= 15 mm) or iterate the capture; commit the three files.
  3. `urdf` mode rewrites `tof_joint`; commit head.urdf; set `camera_info_file` (head_camera_cpp) and keep `tof_calibration_file` in client.yml; redeploy head_state_publisher, head_camera_cpp and head_rgbd_cpp; confirm `/head_rgbd/depth/image_raw` publishes at the mapping rate with a subscriber and that the head camera's focus is continuous again when no subscriber is left (U4).
  4. Time offset: `capture-validation sweep` while panning the head +-20 deg at about 20 deg/s (head pan only, inside the verified range); `validate` gives the ToF-RGB offset; write it into `arducam_tof.stamp_latency_ms` in client.yml, redeploy arducam_tof, repeat the sweep: |offset| <= 10 ms.
  5. Depth accuracy: matte target perpendicular at 0.5, 1, 2, 3 m (tape reference), `capture-validation <d>m` each; `validate`: raw ToF bias/RMSE and registration bias against the targets; if a residual bias > 1 cm remains, record it as a finding with numbers (the design's correction rule) rather than hiding it.
  6. Alignment and coverage: board 1 m in front of a wall >= 2.5 m (`capture-validation wall`): median edge alignment <= 1.0 px at 288x162; valid ratio >= 0.85 on a flat wall at 1.5 m; motion: the sweep with compensation on gives median alignment <= 1.5 px and the dropped-pair counter rises only above the rate limit.
  7. CPU: with `mapping.rate_hz` temporarily 10 and a subscriber on `/head_rgbd/depth/image_raw`, 10 minutes in Grafana (cAdvisor `container_cpu_usage_seconds_total` for `ros2-head_rgbd_cpp.service`, and `rate(process_cpu_seconds_total)` of the node): p95 <= 0.2 core with upsampling off; repeat with upsampling on and record; record head_camera_cpp's CPU delta; set memory_max to >= 2x the measured peak; restore `mapping.rate_hz` 2.
  8. Placeholder check part 2: stop arducam_tof, then block TF (stop head_state_publisher): `/head_rgbd/*` publishes nothing and the drop counters name the reason; restart both.
- Fill the methodology document's `### Results` table from steps 2-7.
- The task ends with the runbook committed with real numbers, or with a named blocker (robots offline) per CLAUDE.md (deploy skipped when RPi targets are offline).

## Acceptance conditions

- a1: the whole repository suite and linters pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && uv run poe lint-cpp]
- a2: the runbook records calibration RMS values, the measured time offset, depth accuracy at four distances, alignment, valid ratio and CPU [evidence: doc-section: docs/runbooks/head-rgbd-fusion.md#Results]
- a3: the calibration files are committed and the head URDF carries the calibrated tof_joint [evidence: command: test -f nodes/head_camera_cpp/config/head_camera_calibration.yaml && test -f nodes/head_rgbd_cpp/config/tof_calibration.yaml && test -f nodes/head_rgbd_cpp/config/head_rgbd_extrinsics.yaml && grep -q 'calibrated' nodes/web_ui/urdf/head.urdf]
- a4: client.yml carries the measured stamp latency and the camera calibration file [evidence: command: grep -q head_camera_calibration.yaml ansible/group_vars/client.yml && grep -q stamp_latency_ms ansible/group_vars/client.yml]
- a5: the methodology results table is filled with measured values [evidence: judgement]
- a6: the deployed node publishes nothing without ToF data or TF [evidence: judgement]

## Files

- `docs/runbooks/head-rgbd-fusion.md`
- `nodes/head_camera_cpp/config/head_camera_calibration.yaml`
- `nodes/head_rgbd_cpp/config/tof_calibration.yaml`
- `nodes/head_rgbd_cpp/config/head_rgbd_extrinsics.yaml`
- `nodes/web_ui/urdf/head.urdf`
- `ansible/group_vars/client.yml`
- `docs/head-rgbd-fusion-methodology.md`

## Interfaces

- `docs/runbooks/head-rgbd-fusion.md` with a `## Results` section.

## Expected outcome

The epic's goals are verified on the robot with recorded numbers.

## Steps

- Run the full verify command; fix or report any failure with its output.
- Write the runbook; execute it step by step with `.logs/` logs; fill `## Results` and the methodology results table.
- Commit the calibration files, URDF, client.yml values and docs: `docs: head RGB-D fusion runbook and measured calibration`.

## Notes

- Servo safety: the head pan sweep is the only motion; stay inside the pan/tilt epic's verified pan range and speed caps.
- Use the ansible-deploy skill before deploying (CLAUDE.md); check no claude_agent session runs before any mcp_server/claude_agent deploy (none planned here).
- Use the grafana MCP or Grafana Explore for CPU; do not SSH to read top.
