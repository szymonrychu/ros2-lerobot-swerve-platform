---
title: Web UI applied head URDF
status: todo
repo: ros2-lerobot-sverve-platform
value: The web UI 3D view and head tab draw the head where it really is after a calibration.
files: [nodes/web_ui/web_ui/server.py, nodes/web_ui/web_ui/config.py, nodes/web_ui/config/default.yaml, nodes/web_ui/tests/test_head_urdf.py, ansible/group_vars/client.yml]
verify: "cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py tests/test_server.py -q && uv run poe lint"
depends_on: [mcp-front-camera-applied]
owns: []
uses: [head-chain-override]
model: ""
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Web UI applied head URDF

## Requirement

The web UI head URDF (3D view, head tab) shows the applied head chain (design D10; approval H4; the RGB-D epic left `/api/urdf/head.urdf` raw).

- `nodes/web_ui/web_ui/config.py`: top-level `head_applied_calibration_dir: str | None = None` (None: URDFs served as files).
- `nodes/web_ui/config/default.yaml`: `head_applied_calibration_dir: null`; `ansible/group_vars/client.yml` web_ui config: `/var/lib/ros2/head_rgbd_calibration/applied`.
- `nodes/web_ui/web_ui/server.py` `/api/urdf/{path}`: when `path` is a configured head URDF name (any tab's `head_urdf`, default `head.urdf`) and the dir is set, answer `apply_overrides(text, dir)` from `ros2_common.head_rgbd_calibration.urdf_text` (`application/xml`, `Cache-Control: no-store`), logging each warning; every other file and `/api/urdf/status` unchanged; the path safety checks unchanged.
- `nodes/web_ui/tests/test_head_urdf.py`: override applied to the served head URDF; no dir -> the file bytes; invalid override -> the URDF chain and a logged warning; other URDF files untouched; `Cache-Control: no-store` on the head URDF.

## Acceptance conditions

- a1: the served head URDF carries the applied chain [evidence: test:nodes/web_ui/tests/test_head_urdf.py::test_served_head_urdf_has_applied_chain]
- a2: without the dir the file is served unchanged [evidence: test:nodes/web_ui/tests/test_head_urdf.py::test_head_urdf_raw_without_applied_dir]
- a3: other URDFs are untouched [evidence: test:nodes/web_ui/tests/test_head_urdf.py::test_other_urdfs_untouched]
- a4: web_ui URDF and server tests pass [evidence: command:cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py tests/test_server.py -q]
- a5: web_ui lint passes [evidence: command:cd nodes/web_ui && uv run poe lint]

## Files

- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/config/default.yaml`
- `nodes/web_ui/tests/test_head_urdf.py`
- `ansible/group_vars/client.yml`

## Interfaces

- web_ui config `head_applied_calibration_dir`; `/api/urdf/<head_urdf>` answers the overridden text with `Cache-Control: no-store`.

## Expected outcome

The web UI 3D view and head tab draw the head where it really is after a calibration.

## Steps

- Tests first (red), implement, verify.
- Run `cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py tests/test_server.py -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D16, review focus).

- 2026-10-22 re-plan: the floor fit epic delivered the override, the FK core and head_state_publisher; dependencies and design references updated to the head-from-arm plan.
