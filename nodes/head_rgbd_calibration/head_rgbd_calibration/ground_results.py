"""Compute, accept and decline the ground board fit of a ground session (user decisions 312, 324, 326).

The fit (`ground_board_fit.fit_ground_board`) starts from the applied head chain (`floor_results.base_urdf`) and
keeps its head mount x, y and yaw (the gauge); the session's depth floor planes are its cross-check. A result lives in
ground_board/<sid>/results/<rid>/: result.json (`kind: "ground_board"`, `state` done or failed, the quality report,
the calibration) and, for a done result, head_chain.json (the applied origins merged with the fitted ones) and
head.urdf (the applied URDF with the fitted origins). The compute runs as a niced child (ground_compute.py, through a
ComputeJob on a `GroundStore`); while it runs the result has no directory and `read_ground_result` reports `running`
from the job.

Accept (only a done, plausible, not declined result fitted against the head chain still applied) copies
head_chain.json into applied/ through `floor_results.apply_head_chain_file`, the one writer of the applied head chain
(applied.json `head_chain` record with source "ground_board" and floor null; the apply path unit restarts the head
units). Decline records `declined: true` in result.json and changes nothing applied.

Telemetry (calib_event): flow `ground_compute` (run id: the result id): `inputs` (views, poses, placements, planes,
skipped poses, depth stats, camera_info, the head chain, the fit config), `solver` (status, evaluations, start and end
RMS, free parameters, largest correlation, condition number), one `refusal` per refusal and one `warning` per warning
and `result` (plausible, implausible or error, with the score, the parameters, the placements, the cross-check and the
accuracy examples); flow `ground_accept`: `accept` (accepted or refused with the reason) and `decline`.
"""

import json
import logging
import os
import re
import shutil
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
from ros2_common.head_rgbd_calibration.charuco import Detection
from ros2_common.head_rgbd_calibration.ground_board_fit import (
    GroundBoardView,
    fit_ground_board,
    fitted_origins_json,
    ground_board_report,
)
from ros2_common.head_rgbd_calibration.urdf_text import HEAD_CHAIN_FILE, head_chain_json

from head_rgbd_calibration import floor_results
from head_rgbd_calibration.apply import NotApplicable
from head_rgbd_calibration.arm_camera_results import head_chain_source
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.ground_sweep import GROUND_DIR, OBSERVATIONS_FILE
from head_rgbd_calibration.store import (
    RESULT_FILE,
    RESULT_ID_RE,
    TEMP_PREFIX,
    InvalidId,
    NotFound,
    SessionStore,
    write_bytes_atomic,
    write_json_atomic,
)
from head_rgbd_calibration.telemetry import calib_event

LOGGER = logging.getLogger(__name__)

GROUND_SESSION_ID_RE = re.compile(r"^ground-\d{8}-\d{6}(?:-\d+)?$", re.ASCII)
RESULTS_DIR = "results"
HEAD_URDF_FILE = "head.urdf"
VERIFICATION_FILE = "verification.json"
KIND = "ground_board"
SOURCE = "ground_board"
FLOW = "ground_compute"
FLOW_ACCEPT = "ground_accept"
STATE_RUNNING = "running"
STATE_DONE = "done"
STATE_FAILED = "failed"
DATE_TIMESPEC = "seconds"
NO_RESULT_ERROR = "no such ground board result: {rid}"
NOT_DONE_ERROR = "result {rid} is not done (state {state})"
NOT_PLAUSIBLE_ERROR = "result {rid} is not plausible and cannot be accepted: {reasons}"
DECLINED_ERROR = "result {rid} was declined and cannot be accepted"
ACCEPTED_ERROR = "result {rid} was accepted and cannot be declined"
ALREADY_ACCEPTED_ERROR = "result {rid} was already accepted"
CRASHED_ERROR = "the compute ended without a result"
NO_CAMERA_INFO_ERROR = "the session has no colour camera_info: capture again"
HEAD_CHAIN_CHANGED_ERROR = (
    "result {rid} was fitted against another head chain ({fitted}); the applied one is {applied}: compute again"
)
# Serialises the read-check-write of accept and decline (two tabs or a double click).
DECISION_LOCK = threading.Lock()


class GroundStore:
    """Paths and ids of the ground sessions (ground_board/<sid>/) and their results; the store a ComputeJob runs the
    ground compute on."""

    def __init__(self, cfg: NodeConfig) -> None:
        """Bind to the state directory.

        Args:
            cfg: Node config (state_dir).
        """
        self.cfg = cfg
        self.root = Path(cfg.state_dir) / GROUND_DIR

    @staticmethod
    def new_result_id(now: datetime) -> str:
        """Result id of a time (r-YYYYmmdd-HHMMSS), as SessionStore."""
        return SessionStore.new_result_id(now)

    def session_dir(self, sid: str) -> Path:
        """Directory of a ground session (it need not exist).

        Raises:
            InvalidId: The id does not match ground-YYYYmmdd-HHMMSS[-N].
        """
        if not GROUND_SESSION_ID_RE.fullmatch(sid):
            raise InvalidId(f"invalid ground session id: {sid!r}")
        return self.root / sid

    def existing_session_dir(self, sid: str) -> Path:
        """Directory of a ground session that holds an observations.json.

        Raises:
            InvalidId: Malformed id.
            NotFound: No such session.
        """
        path = self.session_dir(sid)
        if not (path / OBSERVATIONS_FILE).is_file():
            raise NotFound(f"no such ground session: {sid}")
        return path

    def result_dir(self, sid: str, rid: str) -> Path:
        """Directory of a result of an existing session (it need not exist).

        Raises:
            InvalidId: Malformed session or result id.
            NotFound: No such session.
        """
        if not RESULT_ID_RE.fullmatch(rid):
            raise InvalidId(f"invalid result id: {rid!r}")
        return self.existing_session_dir(sid) / RESULTS_DIR / rid

    def read(self, sid: str) -> dict[str, Any]:
        """The session's observations.json."""
        return json.loads((self.existing_session_dir(sid) / OBSERVATIONS_FILE).read_text())

    def pose_ids(self, sid: str) -> list[str]:
        """The view ids of a session (ComputeJob logs their count)."""
        return [str(v["view_id"]) for v in self.read(sid)["views"]]

    def result_ids(self, sid: str) -> list[str]:
        """Ids of the session's results that hold a result.json, newest first."""
        results = self.existing_session_dir(sid) / RESULTS_DIR
        if not results.is_dir():
            return []
        found = [p.name for p in results.iterdir() if RESULT_ID_RE.fullmatch(p.name) and (p / RESULT_FILE).is_file()]
        return sorted(found, reverse=True)

    def list_sessions(self) -> list[dict[str, Any]]:
        """Summaries of the stored sessions, newest first.

        Returns:
            [{id, created, placements, views, planes, skipped, results}]: counts and result ids.
        """
        if not self.root.is_dir():
            return []
        found = []
        for path in self.root.iterdir():
            if not GROUND_SESSION_ID_RE.fullmatch(path.name) or not (path / OBSERVATIONS_FILE).is_file():
                continue
            doc = json.loads((path / OBSERVATIONS_FILE).read_text())
            found.append(
                {
                    "id": path.name,
                    "created": doc["created"],
                    "placements": len(doc["placements"]),
                    "views": len(doc["views"]),
                    "planes": len(doc["observations"]),
                    "skipped": len(doc["skipped"]),
                    "results": self.result_ids(path.name),
                }
            )
        return sorted(found, key=lambda s: (s["created"], s["id"]), reverse=True)


def session_views(doc: dict[str, Any]) -> list[GroundBoardView]:
    """The board views of an observations.json."""
    return [
        GroundBoardView(
            view_id=str(v["view_id"]),
            placement=int(v["placement"]),
            pose=int(v["pose"]),
            pan_rad=float(v["pan_rad"]),
            tilt_rad=float(v["tilt_rad"]),
            detection=Detection(
                ids=np.asarray(v["ids"], dtype=np.int32), points=np.asarray(v["points"], dtype=np.float32)
            ),
        )
        for v in doc["views"]
    ]


def depth_summary(doc: dict[str, Any]) -> dict[str, Any]:
    """Median and spread of the views' aligned depth bias (mm) and how many views have one."""
    biases = [float(v["depth_bias_mm"]) for v in doc["views"] if v.get("depth_bias_mm") is not None]
    return {
        "views_with_depth": len(biases),
        "median_bias_mm": float(np.median(biases)) if biases else None,
        "max_abs_bias_mm": float(np.max(np.abs(biases))) if biases else None,
    }


def base_body(rid: str, sid: str, now: datetime, head_chain: dict[str, str | None] | None) -> dict[str, Any]:
    """The result.json fields every result has."""
    return {
        "id": rid,
        "session": sid,
        "kind": KIND,
        "state": STATE_FAILED,
        "error": None,
        "created_at": now.isoformat(timespec=DATE_TIMESPEC),
        "declined": False,
        "declined_at": None,
        "accepted_at": None,
        "head_chain": head_chain,
        "report": None,
        "calibration": None,
        "verification": None,
    }


def log_checks(rid: str, sid: str, report: dict[str, Any]) -> None:
    """One calib_event per refusal (`refused`, WARNING) and per warning (`warning`, INFO)."""
    for entry in report["refusals"]:
        calib_event(LOGGER, FLOW, rid, "refusal", "refused", logging.WARNING, entry["message"], session_id=sid, **entry)
    for entry in report["warnings"]:
        calib_event(LOGGER, FLOW, rid, "warning", "warning", logging.INFO, entry["message"], session_id=sid, **entry)


def compute_ground(cfg: NodeConfig, store: GroundStore, sid: str, rid: str, now: datetime) -> dict[str, Any]:
    """Fit the head chain to a ground session and store the result as results/<rid>/.

    Args:
        cfg: Node config (state_dir, head_urdf_file, camera_frame, floor.sensor_frame, ground_board.fit).
        store: Ground store.
        sid: Ground session id.
        rid: Result id (given by the compute job).
        now: Result time.

    Returns:
        The result.json content {id, session, kind: "ground_board", state, error, created_at, declined, declined_at,
        accepted_at, head_chain {source, session, result}, report, calibration {joints {name: {xyz, rpy}}, note},
        verification}. An input or fit error gives state "failed" with `error` and only result.json.

    Raises:
        InvalidId, NotFound: unknown or malformed session or result id.
    """
    final = store.result_dir(sid, rid)
    tmp = final.with_name(f"{TEMP_PREFIX}{rid}")
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.mkdir(parents=True)
    head_chain = head_chain_source(cfg)
    result = base_body(rid, sid, now, head_chain)
    began = time.monotonic()
    try:
        doc = store.read(sid)
        views = session_views(doc)
        observations, _ = floor_results.read_observations(store.existing_session_dir(sid) / OBSERVATIONS_FILE)
        info = doc.get("camera_info")
        fit_cfg = cfg.ground_board.fit
        calib_event(
            LOGGER,
            FLOW,
            rid,
            "inputs",
            "ok",
            session_id=sid,
            views=len(views),
            poses=len({v.pose for v in views}),
            placements=len({v.placement for v in views}),
            planes=len(observations),
            skipped=doc.get("skipped", []),
            placement_failures=doc.get("placement_failures", []),
            depth=depth_summary(doc),
            camera_info=info,
            head_chain=head_chain,
            fit=fit_cfg,
            camera_frame=cfg.camera_frame,
            sensor_frame=cfg.floor.sensor_frame,
        )
        if info is None:
            raise ValueError(NO_CAMERA_INFO_ERROR)
        urdf = floor_results.base_urdf(cfg)
        fit = fit_ground_board(
            views,
            observations,
            np.asarray(info["k"], dtype=float).reshape(3, 3),
            np.asarray(info["d"], dtype=float),
            urdf,
            fit_cfg,
            cfg.camera_frame,
            cfg.floor.sensor_frame,
        )
        calib_event(
            LOGGER,
            FLOW,
            rid,
            "solver",
            "ok",
            duration_s=time.monotonic() - began,
            session_id=sid,
            **fit.solver,
            rms_px=fit.rms_px,
            max_correlation={"pair": list(fit.max_correlation.pair), "value": fit.max_correlation.value},
            condition_number=fit.condition_number,
        )
        report = ground_board_report(fit, fit_cfg, None if head_chain["source"] == "urdf" else head_chain)
        log_checks(rid, sid, report)
        note = (
            f"ground board fit {now.strftime(floor_results.NOTE_DATE_FORMAT)}, rms {report['score']['rms_px']:.2f} px, "
            f"{report['score']['placements']} placements"
        )
        origins = {
            **floor_results.applied_chain_origins(Path(cfg.state_dir) / floor_results.APPLIED_DIR),
            **fit.origins,
        }
        result.update(
            state=STATE_DONE,
            report=report,
            calibration={"joints": fitted_origins_json(fit), "note": note},
        )
        (tmp / HEAD_CHAIN_FILE).write_text(head_chain_json(origins, note, sid, rid))
        write_bytes_atomic(tmp / HEAD_URDF_FILE, floor_results.apply_head_chain(urdf, fit.origins, note).encode())
        calib_event(
            LOGGER,
            FLOW,
            rid,
            "result",
            "plausible" if report["plausible"] else "implausible",
            logging.INFO if report["plausible"] else logging.WARNING,
            "; ".join(r["message"] for r in report["refusals"]) or None,
            time.monotonic() - began,
            session_id=sid,
            score=report["score"],
            params=report["params"],
            placements=report["placements"],
            cross_check=report["cross_check"],
            accuracy_examples=report["accuracy_examples"],
            current_score=report["comparison"]["current_score"],
            refusals=[r["name"] for r in report["refusals"]],
            warnings=[w["name"] for w in report["warnings"]],
        )
    except Exception as exc:  # every failure becomes a failed result.json with its telemetry, never a lost child
        calib_event(
            LOGGER,
            FLOW,
            rid,
            "result",
            "error",
            logging.WARNING,
            str(exc),
            time.monotonic() - began,
            session_id=sid,
            head_chain=head_chain,
            error_type=type(exc).__name__,
        )
        for path in tmp.iterdir():
            path.unlink()
        result = {**base_body(rid, sid, now, head_chain), "error": str(exc)}
    write_json_atomic(tmp / RESULT_FILE, result)
    shutil.rmtree(final, ignore_errors=True)
    os.replace(tmp, final)
    return result


def stored_result(store: GroundStore, sid: str, rid: str) -> dict[str, Any] | None:
    """The result.json of a ground result, None when the result directory holds none.

    Raises:
        InvalidId, NotFound: unknown or malformed ids, or a result of another kind.
    """
    path = store.result_dir(sid, rid) / RESULT_FILE
    if not path.is_file():
        return None
    result = json.loads(path.read_text())
    if result.get("kind") != KIND:
        raise NotFound(NO_RESULT_ERROR.format(rid=rid))
    return result


def read_ground_result(store: GroundStore, sid: str, rid: str, job_state: dict[str, Any] | None) -> dict[str, Any]:
    """The result body of GET /api/ground-board/sessions/{sid}/results/{rid}.

    Args:
        store: Ground store.
        sid: Ground session id.
        rid: Result id.
        job_state: The ground ComputeJob state ({running, session_id, result_id, started_at, error}), None when
            unknown.

    Returns:
        result.json once written; while the job computes this result the base body with state "running"; after a
        job that ended without result.json the same with state "failed" and the job error.

    Raises:
        InvalidId, NotFound: unknown or malformed ids, a result of another kind, or no such result.
    """
    result = stored_result(store, sid, rid)
    if result is not None:
        return result
    if job_state is None or (job_state.get("session_id"), job_state.get("result_id")) != (sid, rid):
        raise NotFound(NO_RESULT_ERROR.format(rid=rid))
    created = job_state.get("started_at")
    body = base_body(rid, sid, datetime.fromisoformat(created) if created else datetime.now().astimezone(), None)
    if job_state.get("running"):
        return {**body, "state": STATE_RUNNING}
    return {**body, "state": STATE_FAILED, "error": job_state.get("error") or CRASHED_ERROR}


def accept_ground(
    cfg: NodeConfig, store: GroundStore, sid: str, rid: str, now: datetime, run: floor_results.RunFn
) -> dict[str, Any]:
    """Accept a done, plausible result: apply its head chain (floor_results.apply_head_chain_file).

    Args:
        cfg: Node config (state_dir, restart_units).
        store: Ground store.
        sid: Ground session id.
        rid: Result id.
        now: Accept time.
        run: subprocess.run-compatible callable for the unit states.

    Returns:
        {head_chain: {session, result, applied_at, source: "ground_board", floor: None}, units: [{name, active_state,
        restarted}]}.

    Raises:
        NotApplicable: the result is not done, not plausible, was declined, was fitted against a head chain that is
            no longer applied, or no D435 intrinsics check is accepted.
        InvalidId, NotFound: unknown or malformed ids.
    """
    with DECISION_LOCK:
        return accept_locked(cfg, store, sid, rid, now, run)


def accept_locked(
    cfg: NodeConfig, store: GroundStore, sid: str, rid: str, now: datetime, run: floor_results.RunFn
) -> dict[str, Any]:
    """`accept_ground` under DECISION_LOCK (same arguments, return value and errors)."""
    result = stored_result(store, sid, rid)
    if result is None:
        raise NotFound(NO_RESULT_ERROR.format(rid=rid))
    refusal = None
    if result["state"] != STATE_DONE:
        refusal = NOT_DONE_ERROR.format(rid=rid, state=result["state"])
    elif result["declined"]:
        refusal = DECLINED_ERROR.format(rid=rid)
    elif result["accepted_at"]:
        refusal = ALREADY_ACCEPTED_ERROR.format(rid=rid)
    elif not result["report"]["plausible"]:
        reasons = "; ".join(r["message"] for r in result["report"]["refusals"])
        refusal = NOT_PLAUSIBLE_ERROR.format(rid=rid, reasons=reasons)
    elif result["head_chain"] != (applied_chain := head_chain_source(cfg)):
        refusal = HEAD_CHAIN_CHANGED_ERROR.format(rid=rid, fitted=result["head_chain"], applied=applied_chain)
    if refusal is not None:
        calib_event(LOGGER, FLOW_ACCEPT, rid, "accept", "refused", logging.WARNING, refusal, session_id=sid)
        raise NotApplicable(refusal)
    record = {"session": sid, "result": rid, "applied_at": now.isoformat(), "source": SOURCE, "floor": None}
    directory = store.result_dir(sid, rid)
    try:
        body = floor_results.apply_head_chain_file(cfg, directory / HEAD_CHAIN_FILE, record, now, run)
    except NotApplicable as exc:
        calib_event(LOGGER, FLOW_ACCEPT, rid, "accept", "refused", logging.WARNING, str(exc), session_id=sid)
        raise
    write_json_atomic(directory / RESULT_FILE, {**result, "accepted_at": record["applied_at"]})
    calib_event(
        LOGGER,
        FLOW_ACCEPT,
        rid,
        "accept",
        "accepted",
        session_id=sid,
        record=record,
        score=result["report"]["score"],
        joints=result["calibration"]["joints"],
        units=body["units"],
    )
    return body


def decline_ground(store: GroundStore, sid: str, rid: str, now: datetime) -> dict[str, Any]:
    """Record a decline in result.json (declined: true, declined_at); nothing applied changes.

    Args:
        store: Ground store.
        sid: Ground session id.
        rid: Result id.
        now: Decline time.

    Returns:
        The updated result.json content.

    Raises:
        NotApplicable: the result is not done or was accepted.
        InvalidId, NotFound: unknown or malformed ids.
    """
    with DECISION_LOCK:
        result = stored_result(store, sid, rid)
        if result is None:
            raise NotFound(NO_RESULT_ERROR.format(rid=rid))
        refusal = None
        if result["state"] != STATE_DONE:
            refusal = NOT_DONE_ERROR.format(rid=rid, state=result["state"])
        elif result["accepted_at"] is not None:
            refusal = ACCEPTED_ERROR.format(rid=rid)
        if refusal is not None:
            calib_event(LOGGER, FLOW_ACCEPT, rid, "decline", "refused", logging.WARNING, refusal, session_id=sid)
            raise NotApplicable(refusal)
        updated = {**result, "declined": True, "declined_at": now.isoformat()}
        write_json_atomic(store.result_dir(sid, rid) / RESULT_FILE, updated)
        calib_event(
            LOGGER,
            FLOW_ACCEPT,
            rid,
            "decline",
            "declined",
            session_id=sid,
            plausible=result["report"]["plausible"],
            score=result["report"]["score"],
        )
        return updated


def accepted_summary(cfg: NodeConfig, store: GroundStore) -> dict[str, Any] | None:
    """The ground board result whose head chain is applied: {session, result, accepted_at, score,
    accuracy_examples, verification}; None when the applied head chain is not a ground board result.

    Args:
        cfg: Node config (state_dir).
        store: Ground store.

    Returns:
        dict | None: The summary.
    """
    chain = head_chain_source(cfg)
    if chain["source"] != SOURCE or not chain["session"] or not chain["result"]:
        return None
    try:
        result = stored_result(store, str(chain["session"]), str(chain["result"]))
    except (InvalidId, NotFound):
        return None
    if result is None or result.get("report") is None:
        return None
    return {
        "session": result["session"],
        "result": result["id"],
        "accepted_at": result["accepted_at"],
        "score": result["report"]["score"],
        "accuracy_examples": result["report"]["accuracy_examples"],
        "verification": result.get("verification"),
    }


def record_verification(store: GroundStore, sid: str, rid: str, summary: dict[str, Any]) -> None:
    """Store a verification summary with its ground result: verification.json and result.json `verification`.

    Args:
        store: Ground store.
        sid: Ground session id.
        rid: Result id.
        summary: The verification state ({passed, poses, ...}).
    """
    with DECISION_LOCK:
        result = stored_result(store, sid, rid)
        if result is None:
            raise NotFound(NO_RESULT_ERROR.format(rid=rid))
        directory = store.result_dir(sid, rid)
        write_json_atomic(directory / VERIFICATION_FILE, summary)
        write_json_atomic(directory / RESULT_FILE, {**result, "verification": summary})


def ground_state(cfg: NodeConfig, store: GroundStore) -> dict[str, Any]:
    """The applied part of GET /api/ground-board/state.

    Args:
        cfg: Node config (state_dir).
        store: Ground store.

    Returns:
        {head_chain {source, session, result}, accepted}: the head chain in effect and the accepted ground board
        result summary (`accepted_summary`).
    """
    return {"head_chain": head_chain_source(cfg), "accepted": accepted_summary(cfg, store)}
