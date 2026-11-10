"""Compute, accept and decline the arm to camera fit of a head_arm session (user decisions 315, 324-326, m577, m578).

The fit (`arm_camera_fit.fit_arm_camera`) holds the applied head chain fixed (`floor_results.base_urdf`: the floor fit
or a head-from-arm result, decision 325) and solves the arm joint offsets, the arm base, the board seat and the sag
from the session's board views; it needs no floor session. The applied arm calibration it starts from and compares
with is the one mcp_server runs with (`effective_fit`): the accepted applied/arm_calibration.json when there is one,
else the configured `fit.joint_offsets_rad` / `fit.base_in_base_link` (pinned equal to mcp_server's `arm:` block).

A result lives in the session's results/<rid>/ next to the head-arm results: result.json (`kind: "arm_camera"`,
`state` done or failed, the quality report, the calibration) and, for a done result, joint_offsets_snippet.yaml (the
reviewed client.yml mcp_server `arm:` snippet). The compute runs as a niced child (arm_camera_compute.py, through a
ComputeJob); while it runs the result has no directory and `read_arm_camera_result` reports `running` from the job.

Accept (only a done, plausible, not declined result of the applied head chain, captured after the last servo zero change)
writes applied/arm_calibration.json (format: ros2_common `arm_calibration_file.ArmCalibrationFile`, validated before
the write) and then the `arm_calibration` record of applied/applied.json through `apply.update_applied`, the writer
every apply uses; the head chain and its record stay as they are. mcp_server sees the applied.json change and applies
the offsets and the arm base at its next idle point (no restart, no client.yml edit); it also clears joint_offsets from
the arm re-zero stale marker then (the one writer). The accept itself only reads the follower bridge's stale marker and
its zero change record (ros2_common `zero_change_refusal`): it refuses when the zero may have changed at or after the
session's first still (`captured_from`: a re-zero, a zero restore, an unknown time or an unreadable file; the offsets
were measured in another zero) and when the session has no still. Decline records `declined: true` in result.json and changes nothing
applied.

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
import threading
import time
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np
from pydantic import ValidationError
from ros2_common.camera_geometry import PLUMB_BOB_COEFFS
from ros2_common.head_rgbd_calibration.arm_calibration_file import (
    ARM_CALIBRATION_FILE,
    ARM_CALIBRATION_KEY,
    ArmCalibrationError,
    ArmCalibrationFile,
    read_arm_calibration,
    zero_change_refusal,
)
from ros2_common.head_rgbd_calibration.arm_camera_fit import (
    arm_camera_report,
    calibration_snippet_yaml,
    fit_arm_camera,
)
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES, BaseInBaseLink, HeadArmFitConfig
from ros2_common.head_rgbd_calibration.floor_fit import json_float

from head_rgbd_calibration import floor_results
from head_rgbd_calibration.apply import APPLIED_DIR, SNIPPET_FILE, NotApplicable, unit_state, update_applied
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.head_arm_results import session_intrinsics, views_summary
from head_rgbd_calibration.store import (
    KIND_HEAD_ARM,
    POSE_FILE,
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
NS_PER_S = 1e9
URDF_SOURCE = "urdf"
# applied.json head_chain record `source` -> the head chain source the arm to camera API names.
HEAD_CHAIN_SOURCES = {
    "floor": "floor_fit",
    "head_arm": "head_arm",
    "arm": "arm_offsets",
    "ground_board": "ground_board",
}
NO_HEAD_CHAIN_REASON = (
    "apply a head calibration first (floor fit, head-from-arm or ground board): the arm to camera fit holds the applied head chain "
    "fixed"
)
NO_RESULT_ERROR = "no such arm to camera result: {rid}"
NOT_DONE_ERROR = "result {rid} is not done (state {state})"
NOT_PLAUSIBLE_ERROR = "result {rid} is not plausible and cannot be accepted: {reasons}"
DECLINED_ERROR = "result {rid} was declined and cannot be accepted"
ACCEPTED_ERROR = "result {rid} was accepted and cannot be declined"
CRASHED_ERROR = "the compute ended without a result"
ACCEPTED_UNREADABLE_ERROR = "the accepted arm calibration cannot be read ({error}): accept a result again"
ZERO_CHANGED_ERROR = "result {rid} cannot be accepted: {reason}; capture a new session in the current arm zero"
NO_STILLS_ERROR = "session {sid} has no still poses"
HEAD_CHAIN_CHANGED_ERROR = (
    "result {rid} was fitted against another head chain ({fitted}); the applied one is {applied}: compute again"
)
# Serialises the read-check-write of accept and decline (two tabs or a double click).
DECISION_LOCK = threading.Lock()


def head_chain_source(cfg: NodeConfig) -> dict[str, str | None]:
    """The applied head chain the fit holds fixed.

    Args:
        cfg: Node config (state_dir).

    Returns:
        {source, session, result}: source "floor_fit", "head_arm", "arm_offsets" or "ground_board" from the
        applied.json head_chain record (None when the record does not name the applied head_chain.json), "urdf"
        without an applied chain.
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
    """The accepted arm to camera calibration in effect: {session, result, accepted_at, score, accuracy_examples,
    head_chain_changed} (`head_chain_changed`: the applied head chain is no longer the one it was fitted against).

    Args:
        cfg: Node config (state_dir).

    Returns:
        dict | None: None when nothing is accepted.
    """
    data = applied_arm_calibration(cfg)
    if data is None:
        return None
    keys = ("session", "result", "accepted_at", "score", "accuracy_examples")
    return {
        **{key: data.get(key) for key in keys},
        "head_chain_changed": data.get("head_chain") != head_chain_source(cfg),
    }


def effective_fit(cfg: NodeConfig) -> tuple[HeadArmFitConfig, dict[str, str] | None]:
    """The fit config with the arm calibration mcp_server runs with: the accepted one overrides the configured joint
    offsets and arm base.

    Args:
        cfg: Node config (state_dir, fit).

    Returns:
        (fit config, {session, result} of the accepted calibration or None when the configured values are used).

    Raises:
        ValueError: The applied.json record names an arm calibration that cannot be read (ArmCalibrationError text).
    """
    try:
        accepted = read_arm_calibration(Path(cfg.state_dir) / APPLIED_DIR)
    except ArmCalibrationError as exc:
        raise ValueError(ACCEPTED_UNREADABLE_ERROR.format(error=exc)) from exc
    if accepted is None:
        return cfg.fit, None
    update = {
        "joint_offsets_rad": dict(accepted.joint_offsets_rad),
        "base_in_base_link": BaseInBaseLink(**accepted.base_in_base_link.model_dump()),
    }
    return cfg.fit.model_copy(update=update), {"session": accepted.session, "result": accepted.result}


def captured_from(store: SessionStore, sid: str) -> datetime:
    """Capture time of the session's earliest still: the earliest stamp of any source (colour, head and arm joints) of
    any still pose, so a zero change just before the first arm joint sample is not missed.

    Args:
        store: Session store.
        sid: head_arm session id.

    Returns:
        datetime: UTC time of the earliest stamp.

    Raises:
        ValueError: the session has no still pose.
    """
    stamps = [
        int(stamp)
        for pid in store.pose_ids(sid)
        for stamp in json.loads((store.pose_dir(sid, pid) / POSE_FILE).read_text())["stamps"].values()
    ]
    if not stamps:
        raise ValueError(NO_STILLS_ERROR.format(sid=sid))
    return datetime.fromtimestamp(min(stamps) / NS_PER_S, tz=UTC)


def zero_refusal(cfg: NodeConfig, rid: str, since: datetime) -> str | None:
    """Why the offsets of a result captured from `since` may not hold in the current arm zero (the follower bridge's
    stale marker and zero change record, ros2_common `zero_change_refusal`).

    Args:
        cfg: Node config (arm_stale_marker).
        rid: Result id (for the message).
        since: The session's first still.

    Returns:
        str | None: The refusal; None without a marker check (`arm_stale_marker` null) or without a zero change at or
        after `since`.
    """
    if cfg.arm_stale_marker is None:
        return None
    reason = zero_change_refusal(Path(cfg.arm_stale_marker), since)
    return None if reason is None else ZERO_CHANGED_ERROR.format(rid=rid, reason=reason)


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
        fit_cfg, applied_source = effective_fit(cfg)
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
            applied_offsets_rad=fit_cfg.joint_offsets_rad,
            applied_base=fit_cfg.base_in_base_link,
            applied_source=applied_source,
            tool_offset_m=fit_cfg.tool_offset_m,
            arm_camera=fit_cfg.arm_camera,
            camera_frame=cfg.camera_frame,
        )
        fit = fit_arm_camera(
            views,
            intrinsics.matrix(),
            np.asarray(intrinsics.distortion or np.zeros(PLUMB_BOB_COEFFS), dtype=float),
            floor_results.base_urdf(cfg),
            Path(cfg.arm_urdf_file).read_text(),
            fit_cfg,
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
        report = arm_camera_report(fit, fit_cfg, applied_summary(cfg))
        log_checks(rid, sid, report)
        snippet = calibration_snippet_yaml(fit, sid, rid, now.strftime(floor_results.NOTE_DATE_FORMAT))
        result.update(
            state=STATE_DONE,
            report=report,
            calibration={
                # json_float: a non-finite fit (refused as non_finite) keeps result.json strict JSON (null)
                "joint_offsets_rad": {name: json_float(fit.offsets[name]) for name in ARM_JOINT_NAMES},
                "base_in_base_link": {key: json_float(value) for key, value in fit.base.items()},
                "tool_offset_m": [json_float(v) for v in fit_cfg.tool_offset_m],
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
    """Accept a done, plausible result: write applied/arm_calibration.json and the applied.json record (mcp_server
    applies it at its next idle point).

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
        NotApplicable: checked in this order (as the ground board accept): the result is not done, was declined or
            is not plausible; it was captured before the last arm re-zero (or the re-zero time is unknown) or does
            not validate as an ArmCalibrationFile; it was fitted against a head chain that is no longer applied.
        InvalidId, NotFound: unknown or malformed ids.
    """
    with DECISION_LOCK:
        return accept_locked(cfg, store, sid, rid, now, run)


def accept_locked(
    cfg: NodeConfig, store: SessionStore, sid: str, rid: str, now: datetime, run: floor_results.RunFn
) -> dict[str, Any]:
    """`accept_arm_camera` under DECISION_LOCK (same arguments, return value and errors)."""
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
    else:
        try:
            since = captured_from(store, sid)
        except ValueError as exc:
            refusal = str(exc)
        else:
            refusal = zero_refusal(cfg, rid, since)
    if refusal is not None:
        calib_event(LOGGER, FLOW_ACCEPT, rid, "accept", "refused", logging.WARNING, refusal, session_id=sid)
        raise NotApplicable(refusal)
    report = result["report"]
    record = {"session": sid, "result": rid, "accepted_at": now.isoformat(), "score": report["score"]}
    calibration = {
        **record,
        **result["calibration"],
        "captured_from": since.isoformat(),
        "head_chain": result["head_chain"],
        "accuracy_examples": report["accuracy_examples"],
    }
    try:
        ArmCalibrationFile.model_validate(calibration)  # the format mcp_server applies
    except ValidationError as exc:
        reason = f"result {rid} does not give a valid arm calibration: {exc}"
        calib_event(LOGGER, FLOW_ACCEPT, rid, "accept", "refused", logging.WARNING, reason, session_id=sid)
        raise NotApplicable(reason) from exc
    text = json.dumps(calibration, indent=2).encode()

    def add_record(info: dict[str, Any]) -> dict[str, Any]:
        # Under the applied lock: the head chain the fit held fixed is still the applied one when this one lands.
        if result["head_chain"] != (applied_chain := head_chain_source(cfg)):
            raise NotApplicable(
                HEAD_CHAIN_CHANGED_ERROR.format(rid=rid, fitted=result["head_chain"], applied=applied_chain)
            )
        return {**info, ARM_CALIBRATION_KEY: record}

    try:
        update_applied(Path(cfg.state_dir) / APPLIED_DIR, add_record, [(ARM_CALIBRATION_FILE, text)])
    except NotApplicable as exc:
        calib_event(LOGGER, FLOW_ACCEPT, rid, "accept", "refused", logging.WARNING, str(exc), session_id=sid)
        raise
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
        captured_from=calibration["captured_from"],
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
    with DECISION_LOCK:
        return decline_locked(store, sid, rid, now)


def decline_locked(store: SessionStore, sid: str, rid: str, now: datetime) -> dict[str, Any]:
    """`decline_arm_camera` under DECISION_LOCK (same arguments, return value and errors)."""
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
        {applied: {session, result, accepted_at, score, accuracy_examples, head_chain_changed} | None,
        head_chain_source, ready, reason}.
    """
    ready, reason = readiness(cfg)
    return {
        "applied": applied_summary(cfg),
        "head_chain_source": head_chain_source(cfg)["source"],
        "ready": ready,
        "reason": reason,
    }
