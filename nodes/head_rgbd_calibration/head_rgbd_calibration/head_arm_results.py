"""Compute and apply a head_arm session (design D31, D34, D36).

A result lives in the session's results/<rid>/: result.json, report.md and, only when it passed, head_chain.json. Apply
goes through the floor fit's writer (`floor_results.apply_head_chain_file`), so both fits share one applied path.
"""

import json
import logging
import os
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from ros2_common.head_rgbd_calibration.floor_fit import json_float
from ros2_common.head_rgbd_calibration.head_arm_fit import (
    FLOOR_PARAMS,
    PARAM_NAMES,
    HeadArmFit,
    fit_head_from_arm,
    head_arm_report_markdown,
    limit_checks,
)
from ros2_common.head_rgbd_calibration.urdf_text import HEAD_CHAIN_FILE, head_chain_json

from head_rgbd_calibration import floor_results
from head_rgbd_calibration.apply import APPLIED_DIR, NotApplicable
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.store import (
    RESULT_FILE,
    TEMP_PREFIX,
    NotFound,
    SessionStore,
)

LOGGER = logging.getLogger(__name__)

KIND = "head_arm"
REPORT_FILE = "report.md"
INTRINSICS_FILE = "head_camera_calibration.yaml"
NO_INTRINSICS_ERROR = "no applied head camera calibration"
DATE_TIMESPEC = "seconds"
SOURCE_HEAD_ARM = "head_arm"


def read_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    """Camera matrix, distortion and image size of a ROS camera_calibration YAML.

    Args:
        path: The YAML file.

    Returns:
        (3x3 camera matrix, distortion coefficients, (width, height)).

    Raises:
        OSError, ValueError, KeyError: unreadable or malformed file.
    """
    data = yaml.safe_load(path.read_text())
    k = np.asarray(data["camera_matrix"]["data"], dtype=float).reshape(3, 3)
    d = np.asarray(data["distortion_coefficients"]["data"], dtype=float).reshape(-1)
    return k, d, (int(data["image_width"]), int(data["image_height"]))


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
    """The result.json content of a computed fit."""
    body = fit.to_json()
    params = {
        name: {
            "value": body["params"][name],
            "sigma": body["sigma"][name],
            "start": body["start_params"][name],
            "floor": json_float(floor[name]) if floor and name in floor else None,
            "floor_delta": body["floor_delta"].get(name),
        }
        for name in PARAM_NAMES
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
        "limits": checks,
        "floor": floor_ref,
    }


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
        cfg: Node config (state_dir, head_urdf_file, arm_urdf_file, fit).
        store: Session store.
        sid: head_arm session id.
        now: Result time (names the result and dates the note).

    Returns:
        The result.json content. An input error (no applied intrinsics, an image size mismatch, too few views, an
        unreadable URDF) gives passed false and `error`; a run over a limit gives passed false and `failed_limits`.
        head_chain.json is written only when the result passed.

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
    try:
        try:
            k, d, size = read_intrinsics(applied / INTRINSICS_FILE)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            LOGGER.warning("applied head camera calibration unreadable: %s", exc)
            raise ValueError(NO_INTRINSICS_ERROR) from exc
        rgb_size = store.read_session(sid)["rgb_size"]
        if rgb_size is not None and tuple(rgb_size) != size:
            raise ValueError(f"the session images are {tuple(rgb_size)} but the applied calibration is {size}")
        floor, floor_ref = read_floor_values(cfg)
        fit = fit_head_from_arm(
            views,
            k,
            d,
            floor_results.base_urdf(cfg),
            Path(cfg.arm_urdf_file).read_text(),
            cfg.fit,
            floor,
        )
        result = result_body(rid, sid, now, fit, cfg, floor, floor_ref)
        date = now.strftime(floor_results.NOTE_DATE_FORMAT)
        (tmp / REPORT_FILE).write_text(head_arm_report_markdown(fit, cfg.fit, sid, date))
        if fit.passed:
            origins = {**floor_results.applied_chain_origins(applied), **fit.origins}
            note = (
                f"head-from-arm fit {date}, RMS {fit.rms_px:.2f} px, held-out board error {fit.holdout_board_mm:.2f} mm"
            )
            (tmp / HEAD_CHAIN_FILE).write_text(head_chain_json(origins, note, sid, rid))
    except (ValueError, OSError) as exc:
        LOGGER.warning("head-arm fit of %s failed: %s", sid, exc)
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
        current floor reference carried forward.

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
    record = {
        "session": sid,
        "result": rid,
        "applied_at": now.isoformat(),
        "source": SOURCE_HEAD_ARM,
        "floor": floor_results.current_floor_ref(cfg),
    }
    return floor_results.apply_head_chain_file(cfg, source, record, now, run)
