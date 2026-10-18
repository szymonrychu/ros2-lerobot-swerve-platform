---
title: Calibration config no depthd
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Turning the Calibrate page on needs no Jetson URL: `calibration: {}` is enough."
files: [nodes/web_ui/web_ui/config.py, nodes/web_ui/tests/test_config.py, nodes/web_ui/tests/test_imu_calibration.py, nodes/web_ui/README.md, tests/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [move-calibrate-config-to-imu]
owns: []
uses: []
model: sonnet
estimate: 35
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Calibration config no depthd

## Requirement

- `CalibrationConfig` has no `depthd_url`, `connect_timeout_s`, `read_timeout_s` or `solve_read_timeout_s` field and no URL validator; its docstring says it holds the IMU calibration ROS names only. `CalibrationConfig()` and `AppConfig.model_validate({"calibration": {}})` are valid with the IMU defaults (`/imu/calibration`, `/imu/calibration/save`, `/imu/calibration/reset`, `imu_stale_s` 3.0, `imu_service_timeout_s` 5.0).
- Tests first in `tests/test_config.py`, seen failing: the depthd_url tests (~391-424) are replaced by `test_calibration_config_needs_no_depthd_url` (empty mapping valid, defaults as above, `depthd_url` not a model field) and the invalid-value cases for `imu_stale_s` and `imu_service_timeout_s` (still rejected). `tests/test_imu_calibration.py` builds `CalibrationConfig` without `depthd_url` (~54, ~136).
- `nodes/web_ui/README.md`: the Calibrate config example has no `depthd_url` or proxy timeouts. `tests/README.md` rows updated.

## Acceptance conditions

- a1: an empty calibration mapping is valid with the IMU defaults and depthd_url is not a field [evidence: test: nodes/web_ui/tests/test_config.py::test_calibration_config_needs_no_depthd_url]
- a2: invalid IMU timeouts are still rejected [evidence: test: nodes/web_ui/tests/test_config.py::test_calibration_config_rejects_invalid_imu_values]
- a3: no web_ui code, test or README mentions depthd [evidence: command: ! git grep -n -i -w depthd -- nodes/web_ui]
- a4: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/tests/test_imu_calibration.py`
- `nodes/web_ui/README.md`
- `tests/README.md`

## Interfaces


- `CalibrationConfig(imu_status_topic: str, imu_save_service: str, imu_reset_service: str, imu_stale_s: float > 0, imu_service_timeout_s: float > 0)`.

## Expected outcome


CalibrationConfig is IMU-only; README and tests agree.

## Steps


- Replace the depthd_url tests; run `cd nodes/web_ui && uv run --frozen pytest tests/test_config.py -q` and see the new test fail.
- Edit `CalibrationConfig`; fix the constructions in test_imu_calibration.py.
- Run the verify command; update READMEs; commit `refactor(web_ui): CalibrationConfig without depthd_url`.

## Notes


- `web_ui/config.py:429-447`. Pydantic models here ignore unknown keys by default; keep that behaviour (no `extra='forbid'`).
