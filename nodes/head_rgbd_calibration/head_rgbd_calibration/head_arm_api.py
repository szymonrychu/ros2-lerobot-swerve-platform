"""HTTP routes of the guided head-from-arm calibration (design D36, D38); included into the node app by __main__.

Errors use the common {ok: false, message} body, as floor_api.py: InvalidId and NotFound (handlers of build_app) give
404, Busy and NotApplicable give 409, a refused step (no confirm, SequenceError, wrong session kind) gives 400. The
confirms (including the grip confirm and reset) are enforced here and again by the Sequencer, so the browser is not
the only guard.
"""

import json
from datetime import datetime
from typing import Any

from fastapi import APIRouter
from pydantic import BaseModel

from head_rgbd_calibration.app import HTTP_BAD_REQUEST, HTTP_CONFLICT, RunFn, error_response
from head_rgbd_calibration.arm_camera_results import KIND as ARM_CAMERA_KIND
from head_rgbd_calibration.arm_results import KIND as ARM_KIND
from head_rgbd_calibration.arm_results import apply_arm, compute_arm
from head_rgbd_calibration.capture import CaptureController
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.floor_results import applied_head_chain
from head_rgbd_calibration.floor_sweep import FloorSweep
from head_rgbd_calibration.head_arm_results import apply_head_arm, compute_head_arm
from head_rgbd_calibration.jobs import ComputeJob
from head_rgbd_calibration.sequencer import IDLE, Busy, SequenceError, Sequencer
from head_rgbd_calibration.store import KIND_HEAD_ARM, RESULT_FILE, NotFound, SessionStore

MSG_SEQUENCE_RUNNING = "a head-arm sequence is running; stop it first"
MSG_SWEEP_RUNNING = "a floor sweep is running; stop it first"
MSG_COMPUTE_RUNNING = "a compute is running; wait for it to finish"
MSG_CAPTURE_RUNNING = "a capture is running; stop it first"
MSG_NEEDS_CONFIRM = "the base move needs confirm=true: it moves the arm joints (no board attached)"
MSG_NEEDS_RESUME_CONFIRM = (
    "resume with the board on needs confirm=true: it moves the arm with the board to the base pose (the gripper keeps"
    " its measured angle)"
)
MSG_NEEDS_BOARD_GRIPPED = (
    "resume from idle needs board_gripped=true: state that the calibration board is gripped in the jaws (taped or"
    " seated); the jaws will not open"
)
MSG_NEEDS_PARK_CONFIRM = (
    "park needs confirm=true: the arm moves to its park pose (the board may stay on when the path is clear)"
)
MSG_NEEDS_GRIP_CONFIRM = (
    "grip confirm needs confirm=true: state that the board handle is seated in the closed jaws; the measured gripper"
    " angle becomes the handle contact (nothing moves)"
)
MSG_NEEDS_GRIP_RESET_CONFIRM = (
    "grip reset needs confirm=true: it deletes the recorded handle contact and returns to the configured one"
)


class ParkRequest(BaseModel):
    """Body of POST /api/head-arm/park: `board_on` states the calibration board is on the gripper (recorded)."""

    confirm: bool = False
    board_on: bool = False


class ResumeRequest(BaseModel):
    """Body of POST /api/head-arm/sequence/resume-board-on: `board_gripped` states the calibration board is gripped in
    the jaws (needed from idle, e.g. after a node restart)."""

    confirm: bool = False
    board_gripped: bool = False


class ComputeArmRequest(BaseModel):
    """Body of POST /api/head-arm/sessions/{sid}/compute-arm: the floor session of the floor views, None to use the
    floor reference of the applied head chain."""

    floor_session: str | None = None


class ConfirmRequest(BaseModel):
    """Body of the confirmed sequence steps."""

    confirm: bool = False


def head_arm_result_ids(store: SessionStore, sid: str) -> list[str]:
    """Result ids of a session for the head-arm routes, newest first: arm to camera results (/api/arm-camera) left out.

    Args:
        store: Session store.
        sid: Session id.

    Returns:
        list[str]: The ids of the head-arm and arm offsets results.
    """
    return [result["id"] for result in store.list_results(sid) if result.get("kind") != ARM_CAMERA_KIND]


def list_head_arm_sessions(store: SessionStore) -> list[dict[str, Any]]:
    """Summaries of the stored head_arm sessions, newest first.

    Args:
        store: Session store.

    Returns:
        [{id, created, views, arm_poses, results}]: stored view count, captured arm pose count, result ids (no arm to
        camera results).
    """
    return [
        {
            "id": session["id"],
            "created": session["created"],
            "views": session["pose_count"],
            "arm_poses": len(store.captured_arm_poses(session["id"])),
            "results": head_arm_result_ids(store, session["id"]),
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
        sequencer: The capture sequence (state, move_to_base, resume_with_board, board_attached, allow_big_joints,
            park, stop, confirm_grip, reset_grip).
        capture: RGB-D capture controller (blocks a sequence start while it runs).
        sweep: Floor sweep (blocks a sequence start while it runs).
        job: RGB-D compute job (blocks a sequence start while it runs).
        store: Session store (head_arm sessions and results).
        cfg: Node config (state_dir, restart_units, fit).
        run: subprocess.run-compatible callable for the unit state after an apply.

    Returns:
        An APIRouter with the /api/head-arm/* routes: the sequence steps, the grip confirm and reset (body
        {confirm}, answer the sequencer state), the head-from-arm compute (`/compute`), the
        arm offsets compute (`/compute-arm`, body {floor_session}), the result of either kind and its apply (dispatched
        on the result's `kind`).
    """
    router = APIRouter(prefix="/api/head-arm")

    def step(action: Any, *args: Any) -> Any:
        try:
            return action(*args)
        except Busy as exc:
            return error_response(HTTP_CONFLICT, str(exc))
        except (SequenceError, ValueError) as exc:
            return error_response(HTTP_BAD_REQUEST, str(exc))

    def wrong_kind(sid: str) -> Any:
        """The 400 response when the session is not a head_arm one (ids that do not exist raise 404/InvalidId)."""
        store.existing_session_dir(sid)
        try:
            store.require_kind(sid, KIND_HEAD_ARM, "head-arm calibration")
        except ValueError as exc:
            return error_response(HTTP_BAD_REQUEST, str(exc))
        return None

    def others_running() -> Any:
        """The 409 response when a sequence, capture, floor sweep or compute runs, else None."""
        for running, message in (
            (sequencer.state()["running"], MSG_SEQUENCE_RUNNING),
            (capture.state()["running"], MSG_CAPTURE_RUNNING),
            (sweep.state()["running"], MSG_SWEEP_RUNNING),
            (job.state()["running"], MSG_COMPUTE_RUNNING),
        ):
            if running:
                return error_response(HTTP_CONFLICT, message)
        return None

    @router.get("/state")
    def state() -> dict[str, Any]:
        return {"sequence": sequencer.state(), "applied": applied_head_chain(cfg)}

    @router.post("/sequence/base")
    def sequence_base(body: ConfirmRequest | None = None) -> Any:
        if not (body and body.confirm):
            return error_response(HTTP_BAD_REQUEST, MSG_NEEDS_CONFIRM)
        if (refused := others_running()) is not None:
            return refused
        return step(sequencer.move_to_base, True)

    @router.post("/sequence/resume-board-on")
    def sequence_resume_board_on(body: ResumeRequest | None = None) -> Any:
        if not (body and body.confirm):
            return error_response(HTTP_BAD_REQUEST, MSG_NEEDS_RESUME_CONFIRM)
        if (refused := others_running()) is not None:
            return refused
        if sequencer.state()["phase"] == IDLE and not body.board_gripped:
            return error_response(HTTP_BAD_REQUEST, MSG_NEEDS_BOARD_GRIPPED)
        return step(sequencer.resume_with_board, True, body.board_gripped)

    @router.post("/grip/confirm")
    def grip_confirm(body: ConfirmRequest | None = None) -> Any:
        if not (body and body.confirm):
            return error_response(HTTP_BAD_REQUEST, MSG_NEEDS_GRIP_CONFIRM)
        if (refused := others_running()) is not None:
            return refused
        return step(sequencer.confirm_grip, True)

    @router.post("/grip/reset")
    def grip_reset(body: ConfirmRequest | None = None) -> Any:
        if not (body and body.confirm):
            return error_response(HTTP_BAD_REQUEST, MSG_NEEDS_GRIP_RESET_CONFIRM)
        if (refused := others_running()) is not None:
            return refused
        return step(sequencer.reset_grip, True)

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
        if not (body and body.confirm):
            return error_response(HTTP_BAD_REQUEST, MSG_NEEDS_PARK_CONFIRM)
        return step(sequencer.park, True, body.board_on)

    @router.get("/sessions")
    def sessions() -> list[dict[str, Any]]:
        return list_head_arm_sessions(store)

    @router.post("/sessions/{sid}/compute")
    def compute(sid: str) -> Any:
        store.existing_session_dir(sid)
        if sequencer.state()["running"]:
            return error_response(HTTP_CONFLICT, MSG_SEQUENCE_RUNNING)
        return step(lambda: compute_head_arm(cfg, store, sid, datetime.now().astimezone()))

    @router.post("/sessions/{sid}/compute-arm")
    def compute_arm_route(sid: str, body: ComputeArmRequest | None = None) -> Any:
        if (refused := wrong_kind(sid)) is not None:
            return refused
        if sequencer.state()["running"]:
            return error_response(HTTP_CONFLICT, MSG_SEQUENCE_RUNNING)
        floor_session = body.floor_session if body else None
        return step(lambda: compute_arm(cfg, store, sid, datetime.now().astimezone(), floor_session))

    @router.get("/sessions/{sid}/results/{rid}")
    def result(sid: str, rid: str) -> Any:
        if (refused := wrong_kind(sid)) is not None:
            return refused
        path = store.result_dir(sid, rid) / RESULT_FILE
        if not path.is_file():
            raise NotFound(f"no such result: {rid}")
        body = json.loads(path.read_text())
        if body.get("kind") == ARM_CAMERA_KIND:
            raise NotFound(f"no such result: {rid} (an arm to camera result: /api/arm-camera)")
        return body

    @router.post("/sessions/{sid}/results/{rid}/apply")
    def apply(sid: str, rid: str) -> Any:
        if (refused := wrong_kind(sid)) is not None:
            return refused
        path = store.result_dir(sid, rid) / RESULT_FILE
        if not path.is_file():
            raise NotFound(f"no such result: {rid}")
        apply = apply_arm if json.loads(path.read_text()).get("kind") == ARM_KIND else apply_head_arm
        return apply(cfg, store, sid, rid, datetime.now().astimezone(), run)

    return router
