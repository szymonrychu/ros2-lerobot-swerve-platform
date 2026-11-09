"""HTTP routes of the arm to camera calibration (/api/arm-camera/*); included into the node app by __main__.

Capture reuses the head-arm sequence (/api/head-arm/sequence/*): a session here is a head_arm session. The compute runs
in the arm to camera ComputeJob (a niced child) and answers 202 {result_id}; the result is polled. Errors use the
common {ok: false, message} body: InvalidId and NotFound give 404 (unknown ids and results of another kind), a session
that is not head_arm gives 400, a running sequence or compute, a node not ready (no applied head chain), and an accept
or decline of a result that is not done, not plausible, declined or accepted give 409.
"""

from datetime import datetime
from typing import Any

from fastapi import APIRouter
from fastapi.responses import JSONResponse

from head_rgbd_calibration.app import HTTP_ACCEPTED, HTTP_BAD_REQUEST, HTTP_CONFLICT, RunFn, error_response
from head_rgbd_calibration.apply import NotApplicable
from head_rgbd_calibration.arm_camera_results import (
    NOT_DONE_ERROR,
    STATE_DONE,
    accept_arm_camera,
    arm_camera_state,
    decline_arm_camera,
    read_arm_camera_result,
    readiness,
)
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.jobs import ComputeJob
from head_rgbd_calibration.sequencer import Sequencer
from head_rgbd_calibration.store import KIND_HEAD_ARM, SessionStore

MSG_SEQUENCE_RUNNING = "a head-arm sequence is running; stop it first"


def arm_camera_router(
    sequencer: Sequencer, job: ComputeJob, store: SessionStore, cfg: NodeConfig, run: RunFn
) -> APIRouter:
    """Routes of the arm to camera calibration.

    Args:
        sequencer: The head-arm capture sequence (a compute is refused while it runs).
        job: The arm to camera ComputeJob (module head_rgbd_calibration.arm_camera_compute).
        store: Session store (head_arm sessions and their results).
        cfg: Node config (state_dir, restart_units, fit).
        run: subprocess.run-compatible callable for the unit states after an accept.

    Returns:
        APIRouter: POST /sessions/{sid}/compute, GET /sessions/{sid}/results/{rid}, POST .../accept, POST .../decline
        and GET /state under /api/arm-camera.
    """
    router = APIRouter(prefix="/api/arm-camera")

    def done_result(sid: str, rid: str) -> None:
        """Raise NotApplicable (409) unless the result is done; NotFound (404) when there is none."""
        body = read_arm_camera_result(store, sid, rid, job.state())
        if body["state"] != STATE_DONE:
            raise NotApplicable(NOT_DONE_ERROR.format(rid=rid, state=body["state"]))

    @router.post("/sessions/{sid}/compute")
    def compute(sid: str) -> Any:
        store.existing_session_dir(sid)
        try:
            store.require_kind(sid, KIND_HEAD_ARM, "the arm to camera fit")
        except ValueError as exc:
            return error_response(HTTP_BAD_REQUEST, str(exc))
        if sequencer.state()["running"]:
            return error_response(HTTP_CONFLICT, MSG_SEQUENCE_RUNNING)
        ready, reason = readiness(cfg)
        if not ready:
            return error_response(HTTP_CONFLICT, str(reason))
        return JSONResponse({"result_id": job.start(sid)}, status_code=HTTP_ACCEPTED)

    @router.get("/sessions/{sid}/results/{rid}")
    def result(sid: str, rid: str) -> dict[str, Any]:
        return read_arm_camera_result(store, sid, rid, job.state())

    @router.post("/sessions/{sid}/results/{rid}/accept")
    def accept(sid: str, rid: str) -> dict[str, Any]:
        done_result(sid, rid)
        return accept_arm_camera(cfg, store, sid, rid, datetime.now().astimezone(), run)

    @router.post("/sessions/{sid}/results/{rid}/decline")
    def decline(sid: str, rid: str) -> dict[str, Any]:
        done_result(sid, rid)
        return decline_arm_camera(store, sid, rid, datetime.now().astimezone())

    @router.get("/state")
    def state() -> dict[str, Any]:
        return arm_camera_state(cfg)

    return router
