---
title: Calib node scaffold config
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The robot-side calibration node exists as a linted, tested uv project with a validated config."
files: [nodes/head_rgbd_calibration/pyproject.toml, nodes/head_rgbd_calibration/uv.lock, nodes/head_rgbd_calibration/head_rgbd_calibration/__init__.py, nodes/head_rgbd_calibration/head_rgbd_calibration/config.py, nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml, nodes/head_rgbd_calibration/tests/test_config.py, pyproject.toml, .github/workflows/ci.yml]
verify: "(cd nodes/head_rgbd_calibration && uv sync && uv run --frozen pytest tests/test_config.py -q && uv run --frozen poe lint) && uv run --frozen pytest tests/test_ci_workflow.py -q"
depends_on: [calib-core-calibrate-run]
owns: [rgbd-calib-node-config]
uses: [rgbd-calib-core]
model: ""
estimate: 50
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-calibration-in-th--calib-node-scaffold-config
branch: head-rgb-d-calibration-in-th--calib-node-scaffold-config
verified_head: ""
run: ""
provides: []
---

# Calib node scaffold config

## Requirement

Create the robot-side calibration node `head_rgbd_calibration` as a uv project with its validated config (design D1).

- `nodes/head_rgbd_calibration/pyproject.toml` (project `head-rgbd-calibration`, package `head_rgbd_calibration`, Python `>=3.12,<3.15`): dependencies fastapi, uvicorn[standard], pydantic>=2, pyyaml, numpy>=1.26, opencv-python-headless>=4.9,<5, ros2-common (`[tool.uv.sources] ros2-common = { path = "../../shared", editable = true }`); dev group ruff==0.15.20, vulture, poethepoet, pytest, httpx; poe `lint` (ruff check, ruff format --check, vulture) and `lint-fix` like `nodes/web_ui/pyproject.toml`; `uv.lock` generated.
- `head_rgbd_calibration/__init__.py` docstring; `tests/__init__.py` if the repo's node test layout needs it.
- `head_rgbd_calibration/config.py`: `NodeConfig` (the core `StrictModel`; not named CalibrationConfig) with constants at the top: `state_dir` (`/var/lib/ros2/head_rgbd_calibration`), `http_host` (`127.0.0.1`), `http_port` (18110), `topics` (`rgb` `/head_camera/mapping/image_raw/compressed`, `amplitude` `/tof/amplitude/image_raw`, `depth` `/tof/depth/image_raw`, `tof_info` `/tof/camera_info`), `qos` (`tof_depth` 10, `rgb_depth` 5, both >= 1), `max_capture_s` 900 (> 0), `compute_timeout_s` 600 (> 0), `compute_nice` 10 (0..19), `head_camera_config_file` (`/etc/ros2/head_camera_cpp/config.yaml`), `default_lens_position` 1.0, `head_urdf_file` (`/opt/ros2-lerobot-swerve-platform/nodes/web_ui/urdf/head.urdf`), `restart_units` (`ros2-head_camera_cpp.service`, `ros2-head_rgbd_cpp.service`, `ros2-head_state_publisher.service`), `restart_check_s` 60, `thumbnail_width_px` 320, `coverage` (`cols` 4 in 3..6, `rows` 3 in 2..4), and the core `board`, `selection`, `limits`; `CONFIG_ENV = "HEAD_RGBD_CALIBRATION_CONFIG"`; `load_config(path) -> NodeConfig` (empty file gives defaults, unknown keys fail naming the key). Type hints and docstrings with input and output types.
- `config/head_rgbd_calibration.yaml`: every default, each key commented.
- `tests/test_config.py`: defaults equal the shipped yaml; unknown keys rejected naming the key; out-of-range values rejected; `board`, `selection`, `limits` equal those of `scripts/head_rgbd_calibrate.yaml` (read relative to the repo root).
- Root `pyproject.toml` `lint-nodes` runs `(cd nodes/head_rgbd_calibration && uv run --frozen poe lint)`; `.github/workflows/ci.yml` node matrix includes `head_rgbd_calibration` so `tests/test_ci_workflow.py` stays green.

## Acceptance conditions

- a1: the config defaults equal the shipped yaml [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_config.py::test_defaults_match_shipped_yaml -q]
- a2: unknown keys and out-of-range values are rejected naming the key [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_config.py::test_rejects_unknown_and_out_of_range -q]
- a3: board, selection and limits equal the CLI config [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_config.py::test_board_selection_limits_match_cli -q]
- a4: the node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen poe lint]
- a5: CI and root lint-nodes cover the node [evidence: command:uv run --frozen pytest tests/test_ci_workflow.py -q && grep -q "nodes/head_rgbd_calibration" pyproject.toml]

## Files

- `nodes/head_rgbd_calibration/pyproject.toml`
- `nodes/head_rgbd_calibration/uv.lock`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/__init__.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `nodes/head_rgbd_calibration/tests/test_config.py`
- `pyproject.toml`
- `.github/workflows/ci.yml`

## Interfaces

- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py::NodeConfig`, `load_config(path: Path) -> NodeConfig`, `CONFIG_ENV`.
- Contract `rgbd-calib-node-config` (owned here).

## Expected outcome

`cd nodes/head_rgbd_calibration && uv sync && uv run pytest tests -q` passes and `uv run poe lint-nodes` at the root covers the node; CI runs its tests.

## Steps

- Write `tests/test_config.py`; `uv run --frozen pytest tests/test_config.py -q`: red.
- Write `pyproject.toml`, `uv lock`, `config.py`, the shipped yaml; add the node to root `lint-nodes` and the CI matrix.
- Run the verify command: all pass, lint clean.

## Notes

- Pattern: `nodes/web_ui/pyproject.toml` (ros2-common editable path source, poe lint) and `nodes/marker_detector`.
- README is created by calib-node-session-store.
