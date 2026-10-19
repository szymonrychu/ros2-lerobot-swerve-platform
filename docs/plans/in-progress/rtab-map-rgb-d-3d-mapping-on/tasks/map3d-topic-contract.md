---
title: Map3d topic contract
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "One file names every stream that crosses the WiFi link, its rate, size and QoS, so the uplink, RTAB-Map and the budget can never disagree."
files: [nodes/rtabmap_mapping/config/map3d_topics.yaml, nodes/rtabmap_mapping/map3d_topics.py, tests/test_map3d_topics.py, nodes/rtabmap_mapping/README.md]
verify: "uv run pytest tests/test_map3d_topics.py -q && uv run poe lint"
depends_on: []
owns: [map3d-topics]
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/rtab-map-rgb-d-3d-mapping-on--map3d-topic-contract
branch: rtab-map-rgb-d-3d-mapping-on--map3d-topic-contract
verified_head: ""
run: ""
provides: []
---

# Map3d topic contract

## Requirement

- New directory `nodes/rtabmap_mapping` (launch/config-only node, no Python package of its own). `nodes/rtabmap_mapping/config/map3d_topics.yaml` is the single definition (D2, D3) of the cross-host streams, with three sections:
  - `uplink` (client -> server), one entry per stream with `source`, `dest`, `type`, `rate_hz` (null = pass every message), `qos` (`reliability`, `durability`, `depth`), `transport` (`raw` or `compressedDepth`), `lazy` (bool) and `budget_bytes` (expected bytes per message, documented with its source): the head RGB-D pair from head_rgbd_cpp (epic head-rgb-d-fusion-of-camera, exact-stamp pairs at the 2 Hz mapping rate, relayed unthrottled so a pair is never split, D5): RGB `/head_rgbd/rgb/image_raw/compressed` -> `/uplink/head_rgbd/rgb/image_raw/compressed` (rate null, expected_rate_hz 2.0, budget 40000), `/head_rgbd/rgb/camera_info` -> `/uplink/head_rgbd/rgb/camera_info` (rate null, expected 2.0), registered depth `/head_rgbd/depth/image_raw` -> `/uplink/head_rgbd/depth/image_raw/compressedDepth` (rate null, expected 2.0, transport compressedDepth, budget 45000), `/head_rgbd/depth/camera_info` -> `/uplink/head_rgbd/depth/camera_info` (rate null, expected 2.0); no raw ToF stream is uplinked; `/scan_filtered` -> `/uplink/scan_filtered` (rate null, budget 12000), `/tf` -> `/uplink/tf` (rate null, budget 200, expected 100 msg/s), `/tf_static` -> `/uplink/tf_static` (transient_local, depth 100), `/map_guard/map_reset` -> `/uplink/map_guard/map_reset`, `/map3d/command` -> `/uplink/map3d/command`.
  - `downlink` (server -> client): `/rtabmap/cloud_map` -> `/map3d/cloud` (rate 0.1, lazy true, budget 1500000), `/map3d/status_server` -> `/map3d/status` (rate null, budget 600), `/map3d/result_server` -> `/map3d/result` (reliable, depth 10).
  - `frames`: `map_frame: map` (slam_toolbox, used by RTAB-Map as its odometry frame, D7), `map3d_frame: rtabmap_map` (RTAB-Map's map frame and the frame of `/map3d/cloud`, identical to `map` by D7), `base_frame: base_link`, `rgb_frame: head_camera_optical_frame`, `depth_frame: tof_optical_frame`; and `budget`: `uplink_mbit_s_max: 8.0`, `downlink_mbit_s_max: 2.0` (my call, D3).
- Every uplink entry lists `server_readers` (the server processes that subscribe it: `rtabmap_mapping` and/or `map3d_manager`); validation allows more than one only for entries flagged `small: true` (budget_bytes x rate < 1 KB/s), because each extra remote reader process is one more copy over WiFi (D2). Readers: RGB, depth, scan, tf, tf_static -> `rtabmap_mapping`; the two camera_infos -> both (small); map_reset and map3d command -> `map3d_manager`.
- `nodes/rtabmap_mapping/map3d_topics.py` (stdlib + pyyaml + pydantic, importable by tests and by both launch files with `sys.path` insertion): pydantic models `Stream`, `Frames`, `Budget`, `Map3dTopics` (unknown keys forbidden), `load_map3d_topics(path: Path | None = None) -> Map3dTopics` (default: the yaml next to it) and `estimated_mbit_s(streams: list[Stream], expected_rates: dict[str, float]) -> float` (sum of budget_bytes x rate x 8 / 1e6; a null rate takes the stream's `expected_rate_hz` field, required when `rate_hz` is null).
- Validation in the models: every uplink `dest` starts with `/uplink/`, every downlink `dest` with `/map3d/`; no `dest` equals any `source` (relay loop and duplicate publishers on the client graph, D2); dests are unique; `rate_hz` > 0 when set; `budget_bytes` > 0; `/tf_static` is transient_local.
- Tests first in `tests/test_map3d_topics.py`, seen failing before the module exists: the shipped yaml loads; each rule above rejects a crafted bad document with a message naming the field; the computed uplink estimate is <= `uplink_mbit_s_max` and the downlink estimate <= `downlink_mbit_s_max`; a large stream with two server readers is rejected; the head RGB uplink is head_rgbd_cpp's pass-through of the mapping stream, never `/head_camera/image_raw` or the 30 fps compressed topic; the depth uplink is `/head_rgbd/depth/image_raw`, never `/tof/depth/image_raw`; the four head_rgbd entries have no rate (unthrottled pairs).
- `nodes/rtabmap_mapping/README.md` (first version): purpose of the node, a `## Topics` section with the uplink and downlink tables generated by hand from the yaml (source, dest, rate, budget) and the bandwidth budget arithmetic of D3.

## Acceptance conditions

- a1: the shipped contract loads and every uplink dest is under /uplink/ and every downlink dest under /map3d/ [evidence: test: tests/test_map3d_topics.py]
- a2: relay loops, duplicate dests, a non-latched /tf_static and a second server reader of a large stream are rejected [evidence: test: tests/test_map3d_topics.py]
- a3: the estimated uplink and downlink bandwidth stay within the budget in the file [evidence: test: tests/test_map3d_topics.py]
- a4: only the head_rgbd RGB-D pair (2 Hz mapping RGB plus registered depth, unthrottled) crosses the link, never raw or 30 fps RGB or raw ToF depth [evidence: test: tests/test_map3d_topics.py]
- a5: /scan_filtered is uplinked to /uplink/scan_filtered, decided by the user on 2026-10-18 [evidence: test: tests/test_map3d_topics.py]
- a6: the README documents the streams and the budget [evidence: doc-section: nodes/rtabmap_mapping/README.md#Topics]

## Files

- `nodes/rtabmap_mapping/config/map3d_topics.yaml`
- `nodes/rtabmap_mapping/map3d_topics.py`
- `tests/test_map3d_topics.py`
- `nodes/rtabmap_mapping/README.md`

## Interfaces

- `nodes/rtabmap_mapping/config/map3d_topics.yaml` and `nodes/rtabmap_mapping/map3d_topics.py:load_map3d_topics` / `Map3dTopics` / `estimated_mbit_s`: the `map3d-topics` contract read by the uplink launch, the RTAB-Map launch, web_ui, mcp_server and map3d_manager_cpp config tests.

## Expected outcome

`uv run pytest tests/test_map3d_topics.py -q` passes; the contract states every cross-host stream with its budget.

## Steps

- Write tests/test_map3d_topics.py; run `uv run pytest tests/test_map3d_topics.py -q`; see it fail on the missing module.
- Write the yaml and map3d_topics.py; rerun until green; `uv run poe lint`.
- Write the README; commit `feat(rtabmap_mapping): cross-host map3d topic contract`.

## Notes

- Root ruff excludes `nodes/`; the module is still linted by the test importing it. Keep it dependency-light: launch files import it on the Pis with the system python (pyyaml and pydantic are available from apt `python3-yaml`, `python3-pydantic`; the node-type tasks add them to apt_packages).
- Budget numbers come from design D3; the runbook replaces them with measured values only through a later commit.
