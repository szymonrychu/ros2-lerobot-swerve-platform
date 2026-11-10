"""Compute and apply a head_arm session (design D31, D34, D36, D10).

The fit solves the head chain, d435_joint included, in `cfg.camera_frame` (the D435 colour optical frame) with the
D435 factory colour intrinsics: the camera_info recorded with each still, which must be the same for every still. A
result lives in the session's results/<rid>/: result.json, report.md and, only when it passed, head_chain.json (no
tof_joint.json). Apply goes through the floor fit's writer (`floor_results.apply_head_chain_file`), so both fits share
one applied path.

Telemetry: the compute logs calib_event lines of flow `head_arm_compute` (run id: the result id): `inputs` (views per
tier and head offset, corner counts, intrinsics, floor reference) and `result` (passed or failed with every parameter
value and sigma, the RMS and held-out errors, each limit check with its value and threshold, the worst views; or the
input error). The apply is logged by floor_results.apply_head_chain_file.
"""

import json
import logging
import math
import os
import shutil
import time
from collections import Counter
from collections.abc import Sequence
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
from ros2_common.camera_geometry import PLUMB_BOB_COEFFS, CameraIntrinsics
from ros2_common.head_rgbd_calibration.floor_fit import json_float
from ros2_common.head_rgbd_calibration.frames import CameraInfoData
from ros2_common.head_rgbd_calibration.head_arm_fit import (
    FLOOR_PARAMS,
    HeadArmFit,
    fit_head_from_arm,
    head_arm_report_markdown,
    limit_checks,
)
from ros2_common.head_rgbd_calibration.urdf_text import HEAD_CHAIN_FILE, ChainResidual, head_chain_json
from ros2_common.head_rgbd_calibration.views import ArmBoardView

from head_rgbd_calibration import floor_results
from head_rgbd_calibration.apply import APPLIED_DIR, NotApplicable
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.store import (
    RESULT_FILE,
    TEMP_PREFIX,
    NotFound,
    SessionStore,
)
from head_rgbd_calibration.telemetry import calib_event

LOGGER = logging.getLogger(__name__)

KIND = "head_arm"
REPORT_FILE = "report.md"
NO_STILLS_ERROR = "the session has no stills"
DATE_TIMESPEC = "seconds"
SOURCE_HEAD_ARM = "head_arm"
FLOW = "head_arm_compute"
WORST_VIEWS = 5


def views_summary(views: Sequence[ArmBoardView]) -> dict[str, Any]:
    """What a fit gets from a head_arm session, for the log.

    Args:
        views (Sequence[ArmBoardView]): The session's board views.

    Returns:
        dict[str, Any]: {views, arm_poses, per_tier, per_head_offset, corners} (corners per view, summarised when long).
    """
    return {
        "views": len(views),
        "arm_poses": len({view.arm_pose for view in views}),
        "per_tier": Counter(view.tier for view in views),
        "per_head_offset": Counter(view.head_offset for view in views),
        "corners": [len(view.detection.ids) for view in views],
    }


def worst_views(per_view: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """The WORST_VIEWS views with the largest reprojection RMS (a NaN RMS counts as the largest).

    Args:
        per_view (list[dict[str, Any]]): `per_view` of a fit result ({view_id, rms_px, ...}).

    Returns:
        list[dict[str, Any]]: Those entries, worst first.
    """

    def badness(entry: dict[str, Any]) -> float:
        value = entry.get("rms_px")
        return math.inf if value is None else float(value)

    return sorted(per_view, key=badness, reverse=True)[:WORST_VIEWS]


def same_camera_info(a: CameraInfoData, b: CameraInfoData) -> bool:
    """Whether two camera_infos hold the same image size, distortion model, K and D.

    Args:
        a: A camera_info.
        b: Another camera_info.

    Returns:
        bool: True when they are equal in those fields.
    """
    return (
        (a.width, a.height, a.distortion_model) == (b.width, b.height, b.distortion_model)
        and np.array_equal(a.k, b.k)
        and np.array_equal(a.d, b.d)
    )


def session_intrinsics(store: SessionStore, sid: str) -> CameraIntrinsics:
    """The D435 factory colour intrinsics of a head_arm session: the camera_info recorded with its stills.

    Args:
        store: Session store.
        sid: head_arm session id.

    Returns:
        CameraIntrinsics: From the camera_info every still carries (`CameraIntrinsics.from_camera_info`).

    Raises:
        ValueError: The session has no stills, a still has no camera_info, two stills carry different ones, or the
            camera_info is not a usable pinhole (plumb_bob) model.
    """
    infos = store.load_arm_camera_infos(sid)
    if not infos:
        raise ValueError(NO_STILLS_ERROR)
    missing = [pid for pid, info in infos.items() if info is None]
    if missing:
        raise ValueError(f"stills without a colour camera_info: {', '.join(missing)}")
    (first_id, first), *rest = infos.items()
    for pid, info in rest:
        if not same_camera_info(first, info):
            raise ValueError(f"the stills carry different colour camera_info: {first_id} and {pid} differ")
    return CameraIntrinsics.from_camera_info(first.width, first.height, first.k, first.d, first.distortion_model)


def read_floor_values(cfg: NodeConfig) -> tuple[dict[str, float] | None, dict[str, str] | None]:
    """The floor-fit values the applied head chain was checked against.

    Args:
        cfg: Node config (state_dir).

    Returns:
        ({floor parameter: value}, {session, result}); (None, None) when there is no usable floor result.
    """
    ref = floor_results.current_floor_ref(cfg)
    if ref is None:
        return None, None
    try:
        params = floor_results.read_result(cfg, ref["session"], ref["result"])["params"]
        return {name: float(params[name]["value"]) for name in FLOOR_PARAMS}, ref
    except (NotFound, ValueError, KeyError, TypeError, OSError) as exc:
        LOGGER.warning("floor result %s unusable as head-arm cross-check: %s", ref, exc)
        return None, None


def result_body(
    rid: str, sid: str, now: datetime, fit: HeadArmFit, cfg: NodeConfig, floor: dict[str, float] | None,
    floor_ref: dict[str, str] | None,
) -> dict[str, Any]:  # fmt: skip
    """The result.json content of a computed fit: one `params` entry per fitted parameter (the pan axis ones too, with
    no floor value), `offset_rms_px` the residual per head offset."""
    body = fit.to_json()
    params = {
        name: {
            "value": body["params"][name],
            "sigma": body["sigma"][name],
            "start": body["start_params"][name],
            "floor": json_float(floor[name]) if floor and name in floor else None,
            "floor_delta": body["floor_delta"].get(name),
        }
        for name in body["params"]
    }
    checks = [
        {"name": name, "value": json_float(value), "limit": json_float(limit), "ok": bool(ok)}
        for name, value, limit, ok in limit_checks(fit, cfg.fit.limits)
    ]
    return {
        "id": rid,
        "session": sid,
        "kind": KIND,
        "created": now.isoformat(timespec=DATE_TIMESPEC),
        "passed": body["passed"],
        "failed_limits": body["failed_limits"],
        "error": None,
        **{
            key: body[key]
            for key in (
                "rms_px",
                "holdout_rms_px",
                "holdout_board_mm",
                "views",
                "arm_poses",
                "pan_span_deg",
                "tilt_span_deg",
                "max_correlation",
            )
        },  # fmt: skip
        "params": params,
        "board_mount": body["board_mount"],
        "per_view": body["per_view"],
        "offset_rms_px": body["offset_rms_px"],
        "limits": checks,
        "floor": floor_ref,
    }


def head_arm_session_exists(store: SessionStore, sid: str | None) -> bool:
    """Whether `sid` is a stored head_arm session.

    Args:
        store: Session store.
        sid: Session id (None: no session).

    Returns:
        bool: True for an existing session of kind head_arm.
    """
    if sid is None:
        return False
    try:
        return store.kind(sid) == KIND
    except (NotFound, OSError, ValueError):
        return False


def latest_board_mount(store: SessionStore, sid: str | None) -> dict[str, list[float]] | None:
    """Board mount of the newest fitted result of a head_arm session.

    Args:
        store: Session store.
        sid: Session id (None: no session).

    Returns:
        dict[str, list[float]] | None: `{"xyz": [m] * 3, "rpy": [rad] * 3}` of the newest result that holds one, None
            without a session, a result or a stored session.
    """
    if sid is None:
        return None
    try:
        results = store.list_results(sid)
    except (NotFound, OSError, ValueError):
        return None
    for result in results:
        mount = result.get("board_mount")
        if isinstance(mount, dict) and len(mount.get("xyz", ())) == 3 and len(mount.get("rpy", ())) == 3:
            return {"xyz": [float(v) for v in mount["xyz"]], "rpy": [float(v) for v in mount["rpy"]]}
    return None


def error_body(rid: str, sid: str, now: datetime, error: str) -> dict[str, Any]:
    """The result.json content of a computation that could not run."""
    return {
        "id": rid,
        "session": sid,
        "kind": KIND,
        "created": now.isoformat(timespec=DATE_TIMESPEC),
        "passed": False,
        "failed_limits": [],
        "error": error,
        "floor": None,
    }


def compute_head_arm(cfg: NodeConfig, store: SessionStore, sid: str, now: datetime) -> dict[str, Any]:
    """Fit the head chain to a head_arm session and store the result.

    Args:
        cfg: Node config (state_dir, head_urdf_file, arm_urdf_file, camera_frame, fit).
        store: Session store.
        sid: head_arm session id.
        now: Result time (names the result and dates the note).

    Returns:
        The result.json content. An input error (a still without camera_info, stills with different camera_info,
        too few views, an unreadable URDF) gives passed false and `error`; a run over a limit gives passed false and
        `failed_limits`. head_chain.json is written only when the result passed.

    Raises:
        InvalidId, NotFound: unknown or malformed session id.
        ValueError: the session is not a head_arm session.
    """
    views = store.load_arm_board_views(sid)
    rid = SessionStore.new_result_id(now)
    final = store.result_dir(sid, rid)
    tmp = final.with_name(f"{TEMP_PREFIX}{rid}")
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.mkdir(parents=True)
    applied = Path(cfg.state_dir) / APPLIED_DIR
    began = time.monotonic()
    try:
        intrinsics = session_intrinsics(store, sid)
        floor, floor_ref = read_floor_values(cfg)
        calib_event(
            LOGGER,
            FLOW,
            rid,
            "inputs",
            "ok",
            session_id=sid,
            **views_summary(views),
            intrinsics=intrinsics,
            floor=floor_ref,
            camera_frame=cfg.camera_frame,
            limits=cfg.fit.limits,
        )
        fit = fit_head_from_arm(
            views,
            intrinsics.matrix(),
            np.asarray(intrinsics.distortion or np.zeros(PLUMB_BOB_COEFFS), dtype=float),
            floor_results.base_urdf(cfg),
            Path(cfg.arm_urdf_file).read_text(),
            cfg.fit,
            floor,
            camera_frame=cfg.camera_frame,
        )
        result = result_body(rid, sid, now, fit, cfg, floor, floor_ref)
        date = now.strftime(floor_results.NOTE_DATE_FORMAT)
        (tmp / REPORT_FILE).write_text(head_arm_report_markdown(fit, cfg.fit, sid, date))
        if fit.passed:
            origins = {**floor_results.applied_chain_origins(applied), **fit.origins}
            note = (
                f"head-from-arm fit {date}, RMS {fit.rms_px:.2f} px, held-out board error {fit.holdout_board_mm:.2f} mm"
            )
            measured = (fit.holdout_board_mm, fit.holdout_rms_px, fit.rms_px)
            residual = ChainResidual(KIND, *measured) if all(math.isfinite(v) for v in measured) else None
            (tmp / HEAD_CHAIN_FILE).write_text(head_chain_json(origins, note, sid, rid, residual))
        calib_event(
            LOGGER,
            FLOW,
            rid,
            "result",
            "passed" if fit.passed else "failed",
            logging.INFO if fit.passed else logging.WARNING,
            "; ".join(result["failed_limits"]) or None,
            time.monotonic() - began,
            session_id=sid,
            **{
                key: result[key]
                for key in ("rms_px", "holdout_rms_px", "holdout_board_mm", "views", "arm_poses", "max_correlation")
            },
            params={name: {"value": p["value"], "sigma": p["sigma"]} for name, p in result["params"].items()},
            limits=result["limits"],
            failed_limits=result["failed_limits"],
            board_mount=result["board_mount"],
            offset_rms_px=result["offset_rms_px"],
            worst_views=worst_views(result["per_view"]),
        )
    except (ValueError, OSError) as exc:
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
            error_type=type(exc).__name__,
        )
        for path in tmp.iterdir():
            path.unlink()
        result = error_body(rid, sid, now, str(exc))
    (tmp / RESULT_FILE).write_text(json.dumps(result, indent=2))
    shutil.rmtree(final, ignore_errors=True)
    os.replace(tmp, final)
    return result


def apply_head_arm(
    cfg: NodeConfig, store: SessionStore, sid: str, rid: str, now: datetime, run: floor_results.RunFn
) -> dict[str, Any]:
    """Apply a passed head_arm result through the floor fit's writer.

    Args:
        cfg: Node config (state_dir, restart_units).
        store: Session store.
        sid: head_arm session id.
        rid: Result id.
        now: Apply time.
        run: subprocess.run-compatible callable for the unit state.

    Returns:
        {head_chain: {session, result, applied_at, source: "head_arm", floor}, units: [...]}; floor is the record's
        current floor reference carried forward, read
        under the applied lock with the write.

    Raises:
        NotApplicable: the result did not pass, or no RGB-D calibration is applied.
        InvalidId, NotFound: unknown or malformed ids.
    """
    directory = store.result_dir(sid, rid)
    result_path = directory / RESULT_FILE
    if not result_path.is_file():
        raise NotFound(f"no such result: {rid}")
    source = directory / HEAD_CHAIN_FILE
    if json.loads(result_path.read_text()).get("passed") is not True or not source.is_file():
        raise NotApplicable(f"result {rid} did not pass and cannot be applied")

    def record_for(info: dict[str, Any]) -> dict[str, Any]:
        # The floor reference applied at the write (under the applied lock), never one read before it.
        floor = floor_results.floor_ref_in(info)
        return {"session": sid, "result": rid, "applied_at": now.isoformat(), "source": SOURCE_HEAD_ARM, "floor": floor}

    return floor_results.apply_head_chain_file(cfg, source, rid, record_for, now, run)
