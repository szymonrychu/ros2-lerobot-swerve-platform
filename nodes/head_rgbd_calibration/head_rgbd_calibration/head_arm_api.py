"""HTTP routes of the guided head-from-arm calibration (design D36, D38); included into the node app by __main__.

Errors use the common {ok: false, message} body, as floor_api.py: InvalidId and NotFound (handlers of build_app) give
404, Busy and NotApplicable give 409, a refused step (no confirm, SequenceError, wrong session kind) gives 400. The
confirms are enforced here and again by the Sequencer, so the browser is not the only guard.
"""

import json
from datetime import datetime
from typing import Any

from fastapi import APIRouter
from pydantic import BaseModel

from head_rgbd_calibration.app import HTTP_BAD_REQUEST, HTTP_CONFLICT, RunFn, error_response
from head_rgbd_calibration.capture import CaptureController
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.floor_results import applied_head_chain
from head_rgbd_calibration.floor_sweep import FloorSweep
from head_rgbd_calibration.head_arm_results import apply_head_arm, compute_head_arm
from head_rgbd_calibration.jobs import ComputeJob
from head_rgbd_calibration.sequencer import Busy, SequenceError, Sequencer
from head_rgbd_calibration.store import KIND_HEAD_ARM, RESULT_FILE, NotFound, SessionStore

MSG_SEQUENCE_RUNNING = "a head-arm sequence is running; stop it first"
MSG_SWEEP_RUNNING = "a floor sweep is running; stop it first"
MSG_COMPUTE_RUNNING = "a compute is running; wait for it to finish"
MSG_CAPTURE_RUNNING = "a capture is running; stop it first"
MSG_NEEDS_CONFIRM = "the base move needs confirm=true: it moves the arm joints (no board attached)"
MSG_NEEDS_BOARD_REMOVED = "park needs board_removed=true: remove the board from the gripper first"


class ConfirmRequest(BaseModel):
    """Body of the confirmed sequence steps."""

    confirm: bool = False


class ParkRequest(BaseModel):
    """Body of POST /api/head-arm/park."""

    board_removed: bool = False


def list_head_arm_sessions(store: SessionStore) -> list[dict[str, Any]]:
    """Summaries of the stored head_arm sessions, newest first.

    Args:
        store: Session store.

    Returns:
        [{id, created, views, arm_poses, results}]: stored view count, captured arm pose count, result ids.
    """
    return [
        {
            "id": session["id"],
            "created": session["created"],
            "views": session["pose_count"],
            "arm_poses": len(store.captured_arm_poses(session["id"])),
            "results": store.result_ids(session["id"]),
        }
        for session in store.list_sessions()
        if session["kind"] == KIND_HEAD_ARM
    ]


def head_arm_router(
    sequencer: Sequencer,
    capture: CaptureController,
    sweep: FloorSweep,
    job: ComputeJob,
    store: SessionStore,
    cfg: NodeConfig,
    run: RunFn,
) -> APIRouter:
    """Routes of the guided head-from-arm calibration.

    Args:
        sequencer: The capture sequence (state, move_to_base, board_attached, allow_big_joints, park, stop).
        capture: RGB-D capture controller (blocks a sequence start while it runs).
        sweep: Floor sweep (blocks a sequence start while it runs).
        job: RGB-D compute job (blocks a sequence start while it runs).
        store: Session store (head_arm sessions and results).
        cfg: Node config (state_dir, restart_units, fit).
        run: subprocess.run-compatible callable for the unit state after an apply.

    Returns:
        An APIRouter with the /api/head-arm/* routes.
    """
    router = APIRouter(prefix="/api/head-arm")

    def step(action: Any, *args: Any) -> Any:
        try:
            return action(*args)
        except Busy as exc:
            return error_response(HTTP_CONFLICT, str(exc))
        except (SequenceError, ValueError) as exc:
            return error_response(HTTP_BAD_REQUEST, str(exc))

    def head_arm_session(sid: str) -> None:
        store.require_kind(sid, KIND_HEAD_ARM, "head-arm calibration")

    @router.get("/state")
    def state() -> dict[str, Any]:
        return {"sequence": sequencer.state(), "applied": applied_head_chain(cfg)}

    @router.post("/sequence/base")
    def sequence_base(body: ConfirmRequest | None = None) -> Any:
        if not (body and body.confirm):
            return error_response(HTTP_BAD_REQUEST, MSG_NEEDS_CONFIRM)
        for running, message in (
            (sequencer.state()["running"], MSG_SEQUENCE_RUNNING),
            (capture.state()["running"], MSG_CAPTURE_RUNNING),
            (sweep.state()["running"], MSG_SWEEP_RUNNING),
            (job.state()["running"], MSG_COMPUTE_RUNNING),
        ):
            if running:
                return error_response(HTTP_CONFLICT, message)
        return step(sequencer.move_to_base, True)

    @router.post("/sequence/board-attached")
    def sequence_board_attached() -> Any:
        return step(sequencer.board_attached)

    @router.post("/sequence/big-joints")
    def sequence_big_joints(body: ConfirmRequest | None = None) -> Any:
        return step(sequencer.allow_big_joints, bool(body and body.confirm))

    @router.post("/sequence/stop")
    def sequence_stop() -> dict[str, Any]:
        return sequencer.stop()

    @router.post("/park")
    def park(body: ParkRequest | None = None) -> Any:
        if not (body and body.board_removed):
            return error_response(HTTP_BAD_REQUEST, MSG_NEEDS_BOARD_REMOVED)
        return step(sequencer.park, True)

    @router.get("/sessions")
    def sessions() -> list[dict[str, Any]]:
        return list_head_arm_sessions(store)

    @router.post("/sessions/{sid}/compute")
    def compute(sid: str) -> Any:
        store.existing_session_dir(sid)
        if sequencer.state()["running"]:
            return error_response(HTTP_CONFLICT, MSG_SEQUENCE_RUNNING)
        return step(lambda: compute_head_arm(cfg, store, sid, datetime.now().astimezone()))

    @router.get("/sessions/{sid}/results/{rid}")
    def result(sid: str, rid: str) -> Any:

        def read() -> Any:
            head_arm_session(sid)
            path = store.result_dir(sid, rid) / RESULT_FILE
            if not path.is_file():
                raise NotFound(f"no such result: {rid}")
            return json.loads(path.read_text())

        return step(read)

    @router.post("/sessions/{sid}/results/{rid}/apply")
    def apply(sid: str, rid: str) -> Any:
        def do_apply() -> Any:
            head_arm_session(sid)
            return apply_head_arm(cfg, store, sid, rid, datetime.now().astimezone(), run)

        return step(do_apply)

    return router
