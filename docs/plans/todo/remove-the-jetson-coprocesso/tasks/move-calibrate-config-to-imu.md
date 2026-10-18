---
title: Move calibrate config to IMU
status: todo
repo: ros2-lerobot-sverve-platform
value: The Calibrate page keeps a working config endpoint and IMU routes after the stereo proxy is deleted.
files: [nodes/web_ui/web_ui/imu_calibration.py, nodes/web_ui/tests/test_imu_calibration.py, nodes/web_ui/web_ui/calibration_proxy.py, nodes/web_ui/tests/test_calibration_proxy.py, nodes/web_ui/web_ui/server.py, nodes/web_ui/README.md, tests/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [drop-network-link-support]
owns: [calibration-config-endpoint]
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Move calibrate config to IMU

## Requirement

- `GET /api/calibration/config` is registered by `register_imu_calibration_routes` in `imu_calibration.py` and returns exactly `{"imu": true, "arm": "deferred"}` when `AppConfig.calibration` is set, and 404 with the not-configured message otherwise. `NOT_CONFIGURED_MESSAGE` and `error()` move into `imu_calibration.py` (type hints, docstrings).
- `web_ui/calibration_proxy.py` and `tests/test_calibration_proxy.py` are deleted. `server.py` drops the `register_calibration_routes` import and call, the `calibration_transport` parameter and its docstring line. No `/api/calibration/stereo/*` route exists.
- Tests first in `tests/test_imu_calibration.py`, seen failing: the config route returns `{"imu": true, "arm": "deferred"}` with calibration set; 404 without; `GET /api/calibration/stereo/preview` is not served by any API route (404). The IMU status, save and reset tests keep passing unchanged.
- `nodes/web_ui/README.md` Calibrate backend section: the stereo proxy bullets, the depthd error text and the `{stereo, imu, arm}` response are replaced by the IMU-only response. `tests/README.md` web_ui calibration rows updated.

## Acceptance conditions

- a1: the config route returns imu true and arm deferred when calibration is configured [evidence: test: nodes/web_ui/tests/test_imu_calibration.py::test_calibration_config_route_is_imu_only]
- a2: the config route answers 404 without calibration [evidence: test: nodes/web_ui/tests/test_imu_calibration.py::test_calibration_config_route_404_when_not_configured]
- a3: no stereo calibration route is served [evidence: test: nodes/web_ui/tests/test_imu_calibration.py::test_stereo_calibration_routes_are_gone]
- a4: the proxy module and its tests are deleted and nothing imports calibration_proxy [evidence: command: test ! -e nodes/web_ui/web_ui/calibration_proxy.py && test ! -e nodes/web_ui/tests/test_calibration_proxy.py && ! git grep -n calibration_proxy -- nodes/web_ui]
- a5: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/web_ui/web_ui/imu_calibration.py`
- `nodes/web_ui/tests/test_imu_calibration.py`
- `nodes/web_ui/web_ui/calibration_proxy.py`
- `nodes/web_ui/tests/test_calibration_proxy.py`
- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/README.md`
- `tests/README.md`

## Interfaces


- `GET /api/calibration/config` -> `{"imu": true, "arm": "deferred"}` (the `calibration-config-endpoint` contract, consumed by calibrate-page-imu-only); `register_imu_calibration_routes(app: FastAPI, config: AppConfig, bridge_node: Any) -> None` keeps its signature.

## Expected outcome


The web_ui backend has no stereo calibration proxy; the Calibrate config endpoint and IMU routes work from imu_calibration.py.

## Steps


- Add the three tests; run `cd nodes/web_ui && uv run --frozen pytest tests/test_imu_calibration.py -q` and see them fail.
- Move the route and helpers; delete the proxy and its tests; edit server.py.
- Run the verify command; update READMEs; commit `refactor(web_ui): IMU calibration owns the calibrate config route; drop the stereo proxy`.

## Notes


- Coupling (discovery brief): `imu_calibration.py:16` imports `NOT_CONFIGURED_MESSAGE, error` from `calibration_proxy`; `/config` lived at `calibration_proxy.py:118`. httpx stays a dependency (agent_proxy, gps_status, tiles use it).
