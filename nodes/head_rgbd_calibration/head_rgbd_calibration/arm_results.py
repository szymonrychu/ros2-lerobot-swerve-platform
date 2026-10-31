"""Compute and apply the arm offsets fit of a head_arm session (design D9, D17).

The fit (`arm_offsets_fit.fit_head_arm`) solves the arm joint offsets, the head chain and the tool point from the
session's plate views (the board on the gripper, all seven joints per still) and the floor planes of a floor session.
A result lives in the session's results/<rid>/ next to the head-from-arm results and records `kind: "arm"`:
result.json and report.md always, plus head_chain.json, head.urdf and joint_offsets_snippet.yaml when it passed. Apply
writes only the head chain, through the floor fit's writer with `source: "arm"`; the offsets snippet is committed to
client.yml by hand after review and never goes to applied/.
"""

import json
import logging
import os
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
from ros2_common.camera_geometry import PLUMB_BOB_COEFFS
from ros2_common.head_rgbd_calibration.arm_offsets_fit import (
    OFFSET_SUFFIX,
    HeadArmFit,
    arm_report_markdown,
    fit_head_arm,
    offset_param,
    offsets_snippet_yaml,
)
from ros2_common.head_rgbd_calibration.chain import UrdfChain
from ros2_common.head_rgbd_calibration.config import ARM_JOINT_NAMES
from ros2_common.head_rgbd_calibration.floor_fit import json_float
from ros2_common.head_rgbd_calibration.head_arm_fit import D435_COLOR_FRAME, camera_mount_joint, start_params
from ros2_common.head_rgbd_calibration.urdf_text import HEAD_CHAIN_FILE, head_chain_json

from head_rgbd_calibration import floor_results
from head_rgbd_calibration.apply import APPLIED_DIR, SNIPPET_FILE, NotApplicable
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.floor_sweep import OBSERVATIONS_FILE
from head_rgbd_calibration.head_arm_results import session_intrinsics
from head_rgbd_calibration.store import KIND_HEAD_ARM, RESULT_FILE, TEMP_PREFIX, NotFound, SessionStore

LOGGER = logging.getLogger(__name__)

KIND = "arm"
SOURCE_ARM = "arm"
REPORT_FILE = "report.md"
DATE_TIMESPEC = "seconds"
NO_HEAD_CALIBRATION_ERROR = "apply a head calibration first"
NO_PLATE_MOUNT_ERROR = "fit.plate_mount is not set: the arm offsets fit needs the plate start pose (plate_mount)"
NO_WRIST_ROLL_ERROR = (
    "fit.wrist_roll_offset_rad is not set: the arm offsets fit needs the recorded wrist_roll neutral "
    "(wrist_roll_offset_rad) while plate_mount.rpy is null"
)


def config_error(cfg: NodeConfig) -> str | None:
    """The error of a fit config the arm offsets fit cannot run with, None when it can.

    Args:
        cfg: Node config (`fit.plate_mount`, `fit.wrist_roll_offset_rad`).

    Returns:
        str | None: The message naming the missing key.
    """
    mount = cfg.fit.plate_mount
    if mount is None:
        return NO_PLATE_MOUNT_ERROR
    if mount.rpy is None and cfg.fit.wrist_roll_offset_rad is None:
        return NO_WRIST_ROLL_ERROR
    return None


def floor_reference(cfg: NodeConfig, floor_session: str | None) -> dict[str, str | None] | None:
    """The floor session the fit reads, with the floor result it is recorded under.

    Args:
        cfg: Node config (state_dir).
        floor_session: The floor session named in the request, None to use the applied head chain's floor reference.

    Returns:
        {session, result}: the named session with its newest result (None when it has none), else
        `floor_results.current_floor_ref`; None when neither exists.
    """
    if floor_session is None:
        return floor_results.current_floor_ref(cfg)
    try:
        results = floor_results.result_ids(cfg, floor_session)
    except NotFound:
        results = []
    return {"session": floor_session, "result": results[0] if results else None}


def result_body(base: dict[str, Any], fit: HeadArmFit, urdf_params: dict[str, float], snippet: str) -> dict[str, Any]:
    """The result.json content of a computed fit.

    Args:
        base: id, session, kind, floor_session, floor and created.
        fit: The fit.
        urdf_params: Value of every head parameter in the head URDF the fit started from.
        snippet: The offsets snippet text.

    Returns:
        The result.json content (see `compute_arm`).
    """
    offsets = {
        joint: {
            "value": float(fit.offsets[joint]),
            "sigma": json_float(fit.sigma[offset_param(joint)]) if offset_param(joint) in fit.sigma else None,
            "current": float(fit.current_offsets[joint]),
        }
        for joint in ARM_JOINT_NAMES
    }
    head = {
        name: {"value": float(value), "sigma": json_float(fit.sigma[name]), "urdf": json_float(urdf_params[name])}
        for name, value in fit.params.items()
        if not name.endswith(OFFSET_SUFFIX)
    }
    return {
        **base,
        "passed": fit.passed,
        "failed_limits": list(fit.failed_limits),
        "error": None,
        "rms_px": json_float(fit.rms_px),
        "holdout_rms_px": json_float(fit.holdout_rms_px),
        "holdout_tool_mm": json_float(fit.holdout_tool_mm),
        "wrist_roll_fitted": fit.wrist_roll_fitted,
        "tool_fitted": fit.tool_fitted,
        "tool_offset_m": [float(v) for v in fit.tool_offset_m],
        "offsets": offsets,
        "head": head,
        "limits": [
            {"name": name, "value": json_float(float(value)), "limit": json_float(float(limit)), "ok": bool(ok)}
            for name, value, limit, ok in fit.limits
        ],
        "snippet": snippet,
    }


def compute_arm(
    cfg: NodeConfig, store: SessionStore, sid: str, now: datetime, floor_session: str | None
) -> dict[str, Any]:
    """Fit the arm offsets to a head_arm session and store the result.

    Args:
        cfg: Node config (state_dir, head_urdf_file, arm_urdf_file, fit).
        store: Session store.
        sid: head_arm session id (the plate views).
        now: Result time (names the result and dates the note and the snippet).
        floor_session: Floor session of the floor views; None uses the floor reference of the applied head chain.

    Returns:
        The result.json content {id, session, kind: "arm", floor_session, floor, created, passed, failed_limits, error,
        rms_px, holdout_rms_px, holdout_tool_mm, wrist_roll_fitted, tool_fitted, tool_offset_m, offsets: {joint:
        {value, sigma, current}}, head: {name: {value, sigma, urdf}}, limits, snippet}. An input error (no floor
        session, no plate_mount, no wrist_roll neutral, a still without camera_info, too few views) gives passed false
        and `error` with only result.json written; a run over a limit gives passed false, report.md and result.json.

    Raises:
        InvalidId, NotFound: unknown or malformed session id.
        ValueError: the session is not a head_arm session.
    """
    store.require_kind(sid, KIND_HEAD_ARM, "the arm offsets fit")
    rid = SessionStore.new_result_id(now)
    final = store.result_dir(sid, rid)
    tmp = final.with_name(f"{TEMP_PREFIX}{rid}")
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.mkdir(parents=True)
    floor = floor_reference(cfg, floor_session)
    base: dict[str, Any] = {
        "id": rid,
        "session": sid,
        "kind": KIND,
        "floor_session": None if floor is None else floor["session"],
        "floor": floor if floor is not None and floor["result"] else None,
        "created": now.isoformat(timespec=DATE_TIMESPEC),
    }
    try:
        if floor is None:
            raise ValueError(NO_HEAD_CALIBRATION_ERROR)
        if (error := config_error(cfg)) is not None:
            raise ValueError(error)
        session_dir = floor_results.existing_floor_session_dir(cfg, floor["session"])
        floor_views, _ = floor_results.read_observations(session_dir / OBSERVATIONS_FILE)
        intrinsics = session_intrinsics(store, sid)
        head_urdf = floor_results.base_urdf(cfg)
        fit = fit_head_arm(
            floor_views,
            store.load_arm_board_views(sid),
            intrinsics.matrix(),
            np.asarray(intrinsics.distortion or np.zeros(PLUMB_BOB_COEFFS), dtype=float),
            head_urdf,
            Path(cfg.arm_urdf_file).read_text(),
            cfg.fit,
            dict(cfg.fit.joint_offsets_rad),
        )
        chain = UrdfChain.from_text(head_urdf)
        names = tuple(name for name in fit.params if not name.endswith(OFFSET_SUFFIX))
        urdf_params = start_params(chain, names, camera_mount_joint(chain, D435_COLOR_FRAME))
        date = now.strftime(floor_results.NOTE_DATE_FORMAT)
        snippet = offsets_snippet_yaml(fit, sid, date)
        result = result_body(base, fit, urdf_params, snippet)
        (tmp / REPORT_FILE).write_text(arm_report_markdown(fit, cfg.fit, sid, date))
        if fit.passed:
            note = f"arm offsets fit {date}, RMS {fit.rms_px:.2f} px, held-out tool error {fit.holdout_tool_mm:.2f} mm"
            origins = {**floor_results.applied_chain_origins(Path(cfg.state_dir) / APPLIED_DIR), **fit.origins}
            (tmp / HEAD_CHAIN_FILE).write_text(head_chain_json(origins, note, sid, rid))
            (tmp / floor_results.HEAD_URDF_FILE).write_text(
                floor_results.apply_head_chain(head_urdf, fit.origins, note)
            )
            (tmp / SNIPPET_FILE).write_text(snippet)
    except (ValueError, OSError, NotFound) as exc:
        LOGGER.warning("arm offsets fit of %s failed: %s", sid, exc)
        for path in tmp.iterdir():
            path.unlink()
        result = {**base, "passed": False, "failed_limits": [], "error": str(exc)}
    (tmp / RESULT_FILE).write_text(json.dumps(result, indent=2))
    shutil.rmtree(final, ignore_errors=True)
    os.replace(tmp, final)
    return result


def apply_arm(
    cfg: NodeConfig, store: SessionStore, sid: str, rid: str, now: datetime, run: floor_results.RunFn
) -> dict[str, Any]:
    """Apply the head chain of a passed arm result through the floor fit's writer (the snippet is never applied).

    Args:
        cfg: Node config (state_dir, restart_units).
        store: Session store.
        sid: head_arm session id.
        rid: Result id of a `kind: "arm"` result.
        now: Apply time.
        run: subprocess.run-compatible callable for the unit state.

    Returns:
        {head_chain: {session, result, applied_at, source: "arm", floor}, units: [...]}; floor is the result's
        {session, result} floor reference. applied.json's top-level intrinsics record is left as it is.

    Raises:
        NotApplicable: the result is not an arm result, did not pass, or no D435 intrinsics check is accepted.
        InvalidId, NotFound: unknown or malformed ids.
    """
    directory = store.result_dir(sid, rid)
    result_path = directory / RESULT_FILE
    if not result_path.is_file():
        raise NotFound(f"no such result: {rid}")
    result = json.loads(result_path.read_text())
    if result.get("kind") != KIND:
        raise NotApplicable(f"result {rid} is not an arm offsets result")
    source = directory / HEAD_CHAIN_FILE
    if result.get("passed") is not True or not source.is_file():
        raise NotApplicable(f"result {rid} did not pass and cannot be applied")
    record = {
        "session": sid,
        "result": rid,
        "applied_at": now.isoformat(),
        "source": SOURCE_ARM,
        "floor": result.get("floor"),
    }
    return floor_results.apply_head_chain_file(cfg, source, record, now, run)
