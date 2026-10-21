"""HTTP routes of the head floor fit (design D6, D7); included into the node app by __main__.

Errors use the common {ok: false, message} body: InvalidId and NotFound (handlers of build_app) give 404, Busy and
NotApplicable give 409, a sweep that cannot start gives 400.
"""

import json
from datetime import datetime
from pathlib import Path
from typing import Any

from fastapi import APIRouter

from head_rgbd_calibration.app import HTTP_BAD_REQUEST, HTTP_CONFLICT, RunFn, error_response
from head_rgbd_calibration.capture import CaptureController
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.floor_results import (
    FLOOR_SESSION_ID_RE,
    applied_head_chain,
    apply_floor,
    compute_floor,
    existing_floor_session_dir,
    read_result,
    result_ids,
)
from head_rgbd_calibration.floor_sweep import FLOOR_DIR, OBSERVATIONS_FILE, Busy, FloorSweep, SweepError
from head_rgbd_calibration.jobs import ComputeJob

MSG_SWEEP_RUNNING = "a floor sweep is running; stop it first"
MSG_COMPUTE_RUNNING = "a compute is running; wait for it to finish"
MSG_CAPTURE_RUNNING = "a capture is running; stop it first"


def list_floor_sessions(cfg: NodeConfig) -> list[dict[str, Any]]:
    """Summaries of the stored floor sessions, newest first.

    Args:
        cfg: Node config (state_dir).

    Returns:
        [{id, created, observations, skipped, results}]: counts of observations and skipped poses, result ids.
    """
    root = Path(cfg.state_dir) / FLOOR_DIR
    if not root.is_dir():
        return []
    found = []
    for path in root.iterdir():
        if not FLOOR_SESSION_ID_RE.fullmatch(path.name) or not (path / OBSERVATIONS_FILE).is_file():
            continue
        doc = json.loads((path / OBSERVATIONS_FILE).read_text())
        found.append(
            {
                "id": path.name,
                "created": doc["created"],
                "observations": len(doc["observations"]),
                "skipped": len(doc["skipped"]),
                "results": result_ids(cfg, path.name),
            }
        )
    return sorted(found, key=lambda s: (s["created"], s["id"]), reverse=True)


def floor_router(
    sweep: FloorSweep, capture: CaptureController, job: ComputeJob, cfg: NodeConfig, run: RunFn
) -> APIRouter:
    """Routes of the head floor fit.

    Args:
        sweep: The floor sweep (state, start, stop).
        capture: RGB-D capture controller (blocks a sweep while it runs).
        job: RGB-D compute job (blocks a sweep while it runs).
        cfg: Node config (state_dir, restart_units, floor.fit).
        run: subprocess.run-compatible callable for the unit state after an apply.

    Returns:
        An APIRouter with the /api/floor/* routes.
    """
    router = APIRouter(prefix="/api/floor")

    @router.get("/state")
    def state() -> dict[str, Any]:
        return {"sweep": sweep.state(), "applied": applied_head_chain(cfg)}

    @router.post("/sweep/start")
    def sweep_start() -> Any:
        if sweep.state()["running"]:
            return error_response(HTTP_CONFLICT, MSG_SWEEP_RUNNING)
        if capture.state()["running"]:
            return error_response(HTTP_CONFLICT, MSG_CAPTURE_RUNNING)
        if job.state()["running"]:
            return error_response(HTTP_CONFLICT, MSG_COMPUTE_RUNNING)
        try:
            sweep.start()
        except SweepError as exc:
            return error_response(HTTP_CONFLICT if isinstance(exc, Busy) else HTTP_BAD_REQUEST, str(exc))
        return sweep.state()

    @router.post("/sweep/stop")
    def sweep_stop() -> dict[str, Any]:
        return sweep.stop()

    @router.get("/sessions")
    def sessions() -> list[dict[str, Any]]:
        return list_floor_sessions(cfg)

    @router.post("/sessions/{sid}/compute")
    def compute(sid: str) -> Any:
        existing_floor_session_dir(cfg, sid)
        if sweep.state()["running"]:
            return error_response(HTTP_CONFLICT, MSG_SWEEP_RUNNING)
        return compute_floor(cfg, sid, datetime.now().astimezone())

    @router.get("/sessions/{sid}/results/{rid}")
    def result(sid: str, rid: str) -> dict[str, Any]:
        return read_result(cfg, sid, rid)

    @router.post("/sessions/{sid}/results/{rid}/apply")
    def apply(sid: str, rid: str) -> dict[str, Any]:
        read_result(cfg, sid, rid)
        return apply_floor(cfg, sid, rid, datetime.now().astimezone(), run)

    return router
