"""Compute, accept and decline the arm to camera fit of a head_arm session (user decisions 315, 324-326, m577, m578).

The fit (`arm_camera_fit.fit_arm_camera`) holds the applied head chain fixed (`floor_results.base_urdf`: the floor fit
or a head-from-arm result, decision 325) and solves the arm joint offsets, the arm base, the board seat and the sag
from the session's board views; it needs no floor session. The applied arm calibration it starts from and compares
with is the configured `fit.joint_offsets_rad` / `fit.base_in_base_link` (pinned equal to mcp_server's `arm:` block).

A result lives in the session's results/<rid>/ next to the head-arm results: result.json (`kind: "arm_camera"`,
`state` done or failed, the quality report, the calibration) and, for a done result, joint_offsets_snippet.yaml (the
reviewed client.yml mcp_server `arm:` snippet). The compute runs as a niced child (arm_camera_compute.py, through a
ComputeJob); while it runs the result has no directory and `read_arm_camera_result` reports `running` from the job.

Accept (only a done, plausible, not declined result) writes applied/arm_calibration.json and the `arm_calibration`
record of applied/applied.json through `apply.update_applied`, the writer every apply uses (the applied.json path unit
fires as for any apply); the head chain and its record stay as they are. mcp_server reads the arm calibration from
client.yml, so the accepted snippet still goes into client.yml by review. Decline records `declined: true` in
result.json and changes nothing applied.

Telemetry (calib_event): flow `arm_camera_compute` (run id: the result id): `inputs` (views, intrinsics, the head
chain source, the applied offsets and base, the fit config), `solver` (status, evaluations, start and end RMS, free
parameters, largest correlation, condition number), one `refusal` per refusal and one `warning` per warning (name,
value, limit, unit, message) and `result` (plausible, implausible or error, with the score, the parameters and the
accuracy examples); flow `arm_camera_accept`: `accept` (accepted or refused with the reason) and `decline`.
"""

import json
import logging
import os
import shutil
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
from ros2_common.camera_geometry import PLUMB_BOB_COEFFS
from ros2_common.head_rgbd_calibration.arm_camera_fit import (
    arm_camera_report,
    calibration_snippet_yaml,
    fit_arm_camera,
)
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES

from head_rgbd_calibration import floor_results
from head_rgbd_calibration.apply import APPLIED_DIR, SNIPPET_FILE, NotApplicable, unit_state, update_applied
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.head_arm_results import session_intrinsics, views_summary
from head_rgbd_calibration.store import (
    KIND_HEAD_ARM,
    RESULT_FILE,
    TEMP_PREFIX,
    NotFound,
    SessionStore,
    write_json_atomic,
)
from head_rgbd_calibration.telemetry import calib_event

LOGGER = logging.getLogger(__name__)

KIND = "arm_camera"
FLOW = "arm_camera_compute"
FLOW_ACCEPT = "arm_camera_accept"
STATE_RUNNING = "running"
STATE_DONE = "done"
STATE_FAILED = "failed"
DATE_TIMESPEC = "seconds"
ARM_CALIBRATION_FILE = "arm_calibration.json"
ARM_CALIBRATION_KEY = "arm_calibration"
URDF_SOURCE = "urdf"
# applied.json head_chain record `source` -> the head chain source the arm to camera API names.
HEAD_CHAIN_SOURCES = {"floor": "floor_fit", "head_arm": "head_arm", "arm": "arm_offsets"}
NO_HEAD_CHAIN_REASON = (
    "apply a head calibration first (floor fit or head-from-arm): the arm to camera fit holds the applied head chain "
    "fixed"
)
NO_RESULT_ERROR = "no such arm to camera result: {rid}"
NOT_DONE_ERROR = "result {rid} is not done (state {state})"
NOT_PLAUSIBLE_ERROR = "result {rid} is not plausible and cannot be accepted: {reasons}"
DECLINED_ERROR = "result {rid} was declined and cannot be accepted"
ACCEPTED_ERROR = "result {rid} was accepted and cannot be declined"
CRASHED_ERROR = "the compute ended without a result"


def head_chain_source(cfg: NodeConfig) -> dict[str, str | None]:
    """The applied head chain the fit holds fixed.

    Args:
        cfg: Node config (state_dir).

    Returns:
        {source, session, result}: source "floor_fit", "head_arm" or "arm_offsets" from the applied.json head_chain
        record (None when the record does not name the applied head_chain.json), "urdf" without an applied chain.
    """
    applied = floor_results.applied_head_chain(cfg)
    if applied is None:
        return {"source": URDF_SOURCE, "session": None, "result": None}
    source = applied["source"]
    return {
        "source": HEAD_CHAIN_SOURCES.get(source, source) if source is not None else None,
        "session": applied["session"],
        "result": applied["result"],
    }


def readiness(cfg: NodeConfig) -> tuple[bool, str | None]:
    """Whether the fit can run: a head chain must be applied (it is held fixed).

    Args:
        cfg: Node config.

    Returns:
        tuple[bool, str | None]: (ready, the reason when not).
    """
    if head_chain_source(cfg)["source"] == URDF_SOURCE:
        return False, NO_HEAD_CHAIN_REASON
    return True, None


def applied_arm_calibration(cfg: NodeConfig) -> dict[str, Any] | None:
    """applied/arm_calibration.json, None when absent or unreadable."""
    try:
        data = json.loads((Path(cfg.state_dir) / APPLIED_DIR / ARM_CALIBRATION_FILE).read_text())
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def applied_summary(cfg: NodeConfig) -> dict[str, Any] | None:
    """The accepted arm to camera calibration in effect: {session, result, accepted_at, score, accuracy_examples}.

    Args:
        cfg: Node config (state_dir).

    Returns:
        dict | None: None when nothing is accepted.
    """
    data = applied_arm_calibration(cfg)
    if data is None:
        return None
    keys = ("session", "result", "accepted_at", "score", "accuracy_examples")
    return {key: data.get(key) for key in keys}


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
    }


def log_checks(rid: str, sid: str, report: dict[str, Any]) -> None:
    """One calib_event per refusal (`refused`, WARNING) and per warning (`warning`, INFO)."""
    for entry in report["refusals"]:
        calib_event(LOGGER, FLOW, rid, "refusal", "refused", logging.WARNING, entry["message"], session_id=sid, **entry)
    for entry in report["warnings"]:
        calib_event(LOGGER, FLOW, rid, "warning", "warning", logging.INFO, entry["message"], session_id=sid, **entry)


def compute_arm_camera(cfg: NodeConfig, store: SessionStore, sid: str, rid: str, now: datetime) -> dict[str, Any]:
    """Fit the arm to camera calibration of a head_arm session and store the result as results/<rid>/.

    Args:
        cfg: Node config (state_dir, head_urdf_file, arm_urdf_file, camera_frame, fit).
        store: Session store.
        sid: head_arm session id.
        rid: Result id (given by the compute job).
        now: Result time.

    Returns:
        The result.json content {id, session, kind: "arm_camera", state, error, created_at, declined, declined_at,
        accepted_at, head_chain {source, session, result}, report, calibration {joint_offsets_rad,
        base_in_base_link, tool_offset_m, snippet}}. An input error (no applied head chain, not a head_arm session,
        a still without camera_info, too few views) gives state "failed" with `error` and only result.json.

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
        store.require_kind(sid, KIND_HEAD_ARM, "the arm to camera fit")
        ready, reason = readiness(cfg)
        if not ready:
            raise ValueError(reason)
        views = store.load_arm_board_views(sid)
        intrinsics = session_intrinsics(store, sid)
        calib_event(
            LOGGER,
            FLOW,
            rid,
            "inputs",
            "ok",
            session_id=sid,
            **views_summary(views),
            intrinsics=intrinsics,
            head_chain=head_chain,
            applied_offsets_rad=cfg.fit.joint_offsets_rad,
            applied_base=cfg.fit.base_in_base_link,
            tool_offset_m=cfg.fit.tool_offset_m,
            arm_camera=cfg.fit.arm_camera,
            camera_frame=cfg.camera_frame,
        )
        fit = fit_arm_camera(
            views,
            intrinsics.matrix(),
            np.asarray(intrinsics.distortion or np.zeros(PLUMB_BOB_COEFFS), dtype=float),
            floor_results.base_urdf(cfg),
            Path(cfg.arm_urdf_file).read_text(),
            cfg.fit,
            cfg.camera_frame,
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
        report = arm_camera_report(fit, cfg.fit, applied_summary(cfg))
        log_checks(rid, sid, report)
        snippet = calibration_snippet_yaml(fit, sid, rid, now.strftime(floor_results.NOTE_DATE_FORMAT))
        result.update(
            state=STATE_DONE,
            report=report,
            calibration={
                "joint_offsets_rad": {name: float(fit.offsets[name]) for name in ARM_JOINT_NAMES},
                "base_in_base_link": {key: float(value) for key, value in fit.base.items()},
                "tool_offset_m": [float(v) for v in cfg.fit.tool_offset_m],
                "snippet": snippet,
            },
        )
        (tmp / SNIPPET_FILE).write_text(snippet)
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
            skew=report["skew"],
            sag=report["sag"],
            accuracy_examples=report["accuracy_examples"],
            current_score=report["comparison"]["current_score"],
            refusals=[r["name"] for r in report["refusals"]],
            warnings=[w["name"] for w in report["warnings"]],
        )
    except (ValueError, OSError, NotFound) as exc:
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


def stored_result(store: SessionStore, sid: str, rid: str) -> dict[str, Any] | None:
    """The result.json of an arm to camera result, None when the result directory holds none.

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


def read_arm_camera_result(store: SessionStore, sid: str, rid: str, job_state: dict[str, Any] | None) -> dict[str, Any]:
    """The result body of GET /api/arm-camera/sessions/{sid}/results/{rid}.

    Args:
        store: Session store.
        sid: head_arm session id.
        rid: Result id.
        job_state: The arm to camera ComputeJob state ({running, session_id, result_id, started_at, error}), None
            when unknown.

    Returns:
        result.json once written; while the job computes this result {id, session, kind, state: "running", error:
        None, created_at (the start), declined: False, declined_at, accepted_at, head_chain: None, report: None,
        calibration: None}; after a job that ended without result.json the same with state "failed" and the job error.

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


def accept_arm_camera(
    cfg: NodeConfig, store: SessionStore, sid: str, rid: str, now: datetime, run: floor_results.RunFn
) -> dict[str, Any]:
    """Accept a done, plausible result: write applied/arm_calibration.json and the applied.json record.

    Args:
        cfg: Node config (state_dir, restart_units).
        store: Session store.
        sid: head_arm session id.
        rid: Result id.
        now: Accept time.
        run: subprocess.run-compatible callable for the unit states.

    Returns:
        {arm_calibration: {session, result, accepted_at, score}, units: [{name, active_state, restarted}]}.

    Raises:
        NotApplicable: the result is not done, not plausible or was declined.
        InvalidId, NotFound: unknown or malformed ids.
    """
    result = stored_result(store, sid, rid)
    if result is None:
        raise NotFound(NO_RESULT_ERROR.format(rid=rid))
    refusal = None
    if result["state"] != STATE_DONE:
        refusal = NOT_DONE_ERROR.format(rid=rid, state=result["state"])
    elif result["declined"]:
        refusal = DECLINED_ERROR.format(rid=rid)
    elif not result["report"]["plausible"]:
        reasons = "; ".join(r["message"] for r in result["report"]["refusals"])
        refusal = NOT_PLAUSIBLE_ERROR.format(rid=rid, reasons=reasons)
    if refusal is not None:
        calib_event(LOGGER, FLOW_ACCEPT, rid, "accept", "refused", logging.WARNING, refusal, session_id=sid)
        raise NotApplicable(refusal)
    report = result["report"]
    record = {"session": sid, "result": rid, "accepted_at": now.isoformat(), "score": report["score"]}
    calibration = {
        **record,
        **result["calibration"],
        "head_chain": result["head_chain"],
        "accuracy_examples": report["accuracy_examples"],
    }
    text = json.dumps(calibration, indent=2).encode()
    update_applied(
        Path(cfg.state_dir) / APPLIED_DIR,
        lambda info: {**info, ARM_CALIBRATION_KEY: record},
        [(ARM_CALIBRATION_FILE, text)],
    )
    write_json_atomic(store.result_dir(sid, rid) / RESULT_FILE, {**result, "accepted_at": record["accepted_at"]})
    units = [unit_state(unit, now.timestamp(), run) for unit in cfg.restart_units]
    calib_event(
        LOGGER,
        FLOW_ACCEPT,
        rid,
        "accept",
        "accepted",
        session_id=sid,
        record=record,
        joint_offsets_rad=calibration["joint_offsets_rad"],
        base_in_base_link=calibration["base_in_base_link"],
        units=units,
    )
    return {ARM_CALIBRATION_KEY: record, "units": units}


def decline_arm_camera(store: SessionStore, sid: str, rid: str, now: datetime) -> dict[str, Any]:
    """Record a decline in result.json (declined: true, declined_at); nothing applied changes.

    Args:
        store: Session store.
        sid: head_arm session id.
        rid: Result id.
        now: Decline time.

    Returns:
        The updated result.json content.

    Raises:
        NotApplicable: the result is not done or was accepted.
        InvalidId, NotFound: unknown or malformed ids.
    """
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


def arm_camera_state(cfg: NodeConfig) -> dict[str, Any]:
    """Body of GET /api/arm-camera/state.

    Args:
        cfg: Node config (state_dir).

    Returns:
        {applied: {session, result, accepted_at, score, accuracy_examples} | None, head_chain_source, ready, reason}.
    """
    ready, reason = readiness(cfg)
    return {
        "applied": applied_summary(cfg),
        "head_chain_source": head_chain_source(cfg)["source"],
        "ready": ready,
        "reason": reason,
    }
