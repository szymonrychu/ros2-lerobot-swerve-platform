"""HTTP routes of the ground board calibration (/api/ground-board/*); included into the node app by __main__.

Capture is a ground session (GroundSweep: start, next placement, stop); the compute runs in the ground ComputeJob (a
niced child) and answers 202 {result_id}; the result is polled; Accept applies its head chain, Decline records; the
verification (GroundVerify) runs after an accept. Errors use the common {ok: false, message} body: InvalidId and
NotFound give 404 (unknown ids and results of another kind), a session or verification that cannot start gives 400,
and a running session, verification, sweep, sequence or compute, a next placement that is not awaited, and an accept
or decline of a result that is not done, not plausible, declined, accepted or fitted against another head chain give
409; so does an accept while a ground verification or ground session runs (the verification would judge, and the
session capture against, a chain replaced under it).
"""

from collections.abc import Callable
from datetime import datetime
from typing import Any

from fastapi import APIRouter
from fastapi.responses import JSONResponse

from head_rgbd_calibration.app import HTTP_ACCEPTED, HTTP_BAD_REQUEST, HTTP_CONFLICT, RunFn, error_response
from head_rgbd_calibration.apply import NotApplicable
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.floor_sweep import Busy, SweepError
from head_rgbd_calibration.ground_results import (
    NOT_DONE_ERROR,
    STATE_DONE,
    GroundStore,
    accept_ground,
    decline_ground,
    ground_state,
    read_ground_result,
)
from head_rgbd_calibration.ground_sweep import GroundSweep
from head_rgbd_calibration.ground_verify import GroundVerify
from head_rgbd_calibration.jobs import ComputeJob

ACCEPT_DURING_VERIFY = "a ground verification is running: stop it or let it finish before accepting another head chain"
ACCEPT_DURING_SWEEP = "a ground board session is running: stop it before changing the head chain it captures against"


def started(action: Callable[[], Any], state: Callable[[], dict[str, Any]]) -> Any:
    """Run a start-like action: its state, or the error response (409 busy, 400 cannot start).

    Args:
        action: The start call.
        state: The state call answered on success.

    Returns:
        The state dict or an error JSONResponse.
    """
    try:
        action()
    except SweepError as exc:
        return error_response(HTTP_CONFLICT if isinstance(exc, Busy) else HTTP_BAD_REQUEST, str(exc))
    return state()


def ground_router(
    sweep: GroundSweep,
    verify: GroundVerify,
    job: ComputeJob,
    store: GroundStore,
    cfg: NodeConfig,
    run: RunFn,
    compute_blocked: Callable[[], str | None],
) -> APIRouter:
    """Routes of the ground board calibration.

    Args:
        sweep: The ground session (capture).
        verify: The verification after an accept.
        job: The ground ComputeJob (module head_rgbd_calibration.ground_compute, on `store`).
        store: Ground store (sessions and results).
        cfg: Node config (state_dir, restart_units, ground_board).
        run: subprocess.run-compatible callable for the unit states after an accept.
        compute_blocked: Returns why a compute may not start now (a head motion or the arm to camera compute runs),
            else None.

    Returns:
        APIRouter: GET /state, POST /sweep/start, /sweep/next-placement, /sweep/stop, GET /sessions, POST
        /sessions/{sid}/compute, GET /sessions/{sid}/results/{rid}, POST .../accept, POST .../decline, POST
        /verify/start, /verify/stop and GET /verify/state under /api/ground-board.
    """
    router = APIRouter(prefix="/api/ground-board")

    def done_result(sid: str, rid: str) -> None:
        """Raise NotApplicable (409) unless the result is done; NotFound (404) when there is none."""
        body = read_ground_result(store, sid, rid, job.state())
        if body["state"] != STATE_DONE:
            raise NotApplicable(NOT_DONE_ERROR.format(rid=rid, state=body["state"]))

    @router.get("/state")
    def state() -> dict[str, Any]:
        return {"sweep": sweep.state(), "verify": verify.state(), "compute": job.state(), **ground_state(cfg, store)}

    @router.post("/sweep/start")
    def sweep_start() -> Any:
        return started(sweep.start, sweep.state)

    @router.post("/sweep/next-placement")
    def sweep_next() -> Any:
        return started(sweep.next_placement, sweep.state)

    @router.post("/sweep/stop")
    def sweep_stop() -> dict[str, Any]:
        return sweep.stop()

    @router.get("/sessions")
    def sessions() -> list[dict[str, Any]]:
        return store.list_sessions()

    @router.post("/sessions/{sid}/compute")
    def compute(sid: str) -> Any:
        store.existing_session_dir(sid)
        reason = compute_blocked()
        if reason:
            return error_response(HTTP_CONFLICT, reason)
        return JSONResponse({"result_id": job.start(sid)}, status_code=HTTP_ACCEPTED)

    @router.get("/sessions/{sid}/results/{rid}")
    def result(sid: str, rid: str) -> dict[str, Any]:
        return read_ground_result(store, sid, rid, job.state())

    @router.post("/sessions/{sid}/results/{rid}/accept")
    def accept(sid: str, rid: str) -> Any:
        done_result(sid, rid)
        for part, reason in ((verify, ACCEPT_DURING_VERIFY), (sweep, ACCEPT_DURING_SWEEP)):
            if part.state()["running"]:
                return error_response(HTTP_CONFLICT, reason)
        return accept_ground(cfg, store, sid, rid, datetime.now().astimezone(), run)

    @router.post("/sessions/{sid}/results/{rid}/decline")
    def decline(sid: str, rid: str) -> dict[str, Any]:
        done_result(sid, rid)
        return decline_ground(store, sid, rid, datetime.now().astimezone())

    @router.post("/verify/start")
    def verify_start() -> Any:
        return started(verify.start, verify.state)

    @router.post("/verify/stop")
    def verify_stop() -> dict[str, Any]:
        return verify.stop()

    @router.get("/verify/state")
    def verify_state() -> dict[str, Any]:
        return verify.state()

    return router
