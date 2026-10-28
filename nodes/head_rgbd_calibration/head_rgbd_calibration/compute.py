"""Compute the D435 intrinsics check of one session's stills (runs as a niced child process).

The colour intrinsics are fitted from the ChArUco board in the stored colour stills and compared with the factory
camera_info recorded with them (design D10); the board-plane depth bias is measured from the aligned depth against
the PnP board plane. The result passes or fails against the limits and writes no calibration file: the factory
intrinsics stay authoritative.
"""

import argparse
import dataclasses
import logging
import os
import shutil
from collections.abc import Sequence
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from ros2_common.head_rgbd_calibration.calibrate import (
    MIN_BOARD_PIXELS,
    board_plane_bias_mm,
    check_intrinsics,
    intrinsics_report_markdown,
)
from ros2_common.head_rgbd_calibration.charuco import Detection, board_corners_3d, board_outline, detect, make_board
from ros2_common.head_rgbd_calibration.config import BoardConfig, LimitsConfig
from ros2_common.head_rgbd_calibration.frames import CameraInfoData
from ros2_common.head_rgbd_calibration.intrinsics import IntrinsicsResult

from head_rgbd_calibration.config import NodeConfig, load_config
from head_rgbd_calibration.store import (
    KIND_INTRINSICS,
    RESULT_FILE,
    TEMP_PREFIX,
    SessionStore,
    StoredStill,
    write_json_atomic,
)

LOGGER = logging.getLogger(__name__)

REPORT_FILE = "report.md"
DATE_TIMESPEC = "seconds"
PLANE_BIAS_KEY = "limits.plane_max_bias_mm"
PLANE_SPREAD_KEY = "limits.plane_max_spread_mm"


def failed_result(base: dict[str, Any], error: str) -> dict[str, Any]:
    """The result.json content of a computation that could not run."""
    return {**base, "passed": False, "error": error, "failed_limits": []}


def same_camera_info(a: CameraInfoData, b: CameraInfoData) -> bool:
    """Both camera_infos describe the same calibration (size, model, D, K and P)."""
    return (
        (a.width, a.height, a.distortion_model) == (b.width, b.height, b.distortion_model)
        and np.array_equal(a.d, b.d)
        and np.array_equal(a.k, b.k)
        and np.array_equal(a.p, b.p)
    )


def session_factory_info(stills: list[StoredStill]) -> CameraInfoData:
    """The factory colour camera_info the session's stills were captured with.

    Args:
        stills: The session's stills.

    Returns:
        CameraInfoData: The camera_info every still carries.

    Raises:
        ValueError: no stills, a still without a camera_info, or stills with different camera_infos.
    """
    if not stills:
        raise ValueError("the session has no poses")
    missing = [still.pose_id for still in stills if still.camera_info is None]
    if missing:
        raise ValueError(f"no factory camera_info is recorded with pose {', '.join(missing)}")
    first = stills[0]
    for still in stills[1:]:
        if not same_camera_info(first.camera_info, still.camera_info):
            raise ValueError(
                f"the factory camera_info of pose {still.pose_id} differs from pose {first.pose_id}'s: "
                "start a new session"
            )
    return first.camera_info


def still_plane_bias(
    depth_mm: np.ndarray, detection: Detection, board_cfg: BoardConfig, factory: CameraInfoData
) -> tuple[float, float] | None:
    """Board-plane depth bias of one still: aligned depth against the board plane solved by PnP with the factory K, D.

    Args:
        depth_mm: The still's aligned depth, uint16 mm.
        detection: Board corners in the colour image.
        board_cfg: The printed board.
        factory: The factory colour camera_info (the intrinsics the driver aligns the depth with).

    Returns:
        (median_mm, spread_mm) over the board pixels, None when PnP fails or too few board pixels have depth.
    """
    k = np.asarray(factory.k, dtype=np.float64).reshape(3, 3)
    d = np.asarray(factory.d, dtype=np.float64)
    objects = board_corners_3d(board_cfg)[detection.ids]
    ok, rvec, tvec = cv2.solvePnP(objects, detection.points.astype(np.float64), k, d)
    if not ok:
        return None
    outline = cv2.projectPoints(board_outline(board_cfg, 0.0), rvec, tvec, k, d)[0].reshape(-1, 2)
    intrinsics = IntrinsicsResult(k, d, 0.0, np.zeros(0), 0, (factory.width, factory.height))
    return board_plane_bias_mm(depth_mm, outline, (cv2.Rodrigues(rvec)[0], tvec.reshape(3)), intrinsics)


def plane_failures(median: float | None, spread: float | None, limits: LimitsConfig) -> list[str]:
    """Plane bias limit failures, each naming its limits.* key.

    Args:
        median: Median of the per-still bias medians (mm), None when no still measured one.
        spread: Median of the per-still spreads (mm), None when no still measured one.
        limits: Acceptance limits (plane_max_bias_mm, plane_max_spread_mm).

    Returns:
        list[str]: One description per failed limit; an unmeasured bias fails plane_max_bias_mm.
    """
    if median is None or spread is None:
        return [f"{PLANE_BIAS_KEY}: board plane depth bias not measured (no still has {MIN_BOARD_PIXELS} board pixels)"]
    failures = []
    if abs(median) > limits.plane_max_bias_mm:
        failures.append(
            f"{PLANE_BIAS_KEY}: board plane depth bias {median:.1f} mm exceeds {limits.plane_max_bias_mm} mm"
        )
    if spread > limits.plane_max_spread_mm:
        failures.append(
            f"{PLANE_SPREAD_KEY}: board plane depth spread {spread:.1f} mm exceeds {limits.plane_max_spread_mm} mm"
        )
    return failures


def compute_result(store: SessionStore, sid: str, rid: str, cfg: NodeConfig, now: datetime) -> dict[str, Any]:
    """Compute the intrinsics check of a session and store it under results/<rid>/.

    The files are written into a temporary directory that is renamed once complete. A check (passed or not) writes
    result.json and report.md; an input error (not an intrinsics session, too few views, no or differing factory
    camera_info) writes result.json with `error` only.

    Args:
        store: Session store holding the stills.
        sid: Session id.
        rid: Result id (r-YYYYmmdd-HHMMSS).
        cfg: Node config (board, limits).
        now: Creation time.

    Returns:
        The result.json content: id, session, created, kind ("intrinsics"), passed, failed_limits, rms_px,
        focal_diff_pct, principal_diff_px, plane_bias_median_mm, plane_bias_spread_mm, views, pose_ids, poses
        ([{id, error_px, plane_bias_mm, plane_spread_mm}], null where a still has no detection or no bias), limits,
        plus `error` on failure.
    """
    final = store.result_dir(sid, rid)
    tmp = final.with_name(f"{TEMP_PREFIX}{rid}")
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.mkdir(parents=True)
    base: dict[str, Any] = {
        "id": rid,
        "session": sid,
        "created": now.isoformat(timespec=DATE_TIMESPEC),
        "kind": KIND_INTRINSICS,
        "passed": False,
        "failed_limits": [],
        "rms_px": None,
        "focal_diff_pct": None,
        "principal_diff_px": None,
        "plane_bias_median_mm": None,
        "plane_bias_spread_mm": None,
        "views": None,
        "pose_ids": store.pose_ids(sid),
        "poses": [],
        "limits": cfg.limits.model_dump(),
    }
    try:
        stills = store.load_stills(sid)
        factory = session_factory_info(stills)
        board = make_board(cfg.board)
        detections = {still.pose_id: detect(still.rgb, board) for still in stills}
        found = [(pid, d) for pid, d in detections.items() if d is not None]
        check = check_intrinsics([d for _, d in found], board, factory, cfg.limits)
        errors = dict(zip([pid for pid, _ in found], check.per_view_errors, strict=True))
        biases = {
            still.pose_id: still_plane_bias(still.depth, detections[still.pose_id], cfg.board, factory)
            for still in stills
            if detections[still.pose_id] is not None
        }
        measured = [bias for bias in biases.values() if bias is not None]
        median = float(np.median([m for m, _ in measured])) if measured else None
        spread = float(np.median([s for _, s in measured])) if measured else None
        failures = [*check.failed_limits, *plane_failures(median, spread, cfg.limits)]
        check = dataclasses.replace(check, failed_limits=tuple(failures), passed=not failures)
        poses = []
        for still in stills:
            bias = biases.get(still.pose_id)
            error = errors.get(still.pose_id)
            poses.append(
                {
                    "id": still.pose_id,
                    "error_px": None if error is None else float(error),
                    "plane_bias_mm": None if bias is None else float(bias[0]),
                    "plane_spread_mm": None if bias is None else float(bias[1]),
                }
            )
        result = {
            **base,
            "passed": check.passed,
            "failed_limits": failures,
            "rms_px": float(check.rms_px),
            "focal_diff_pct": float(check.focal_diff_pct),
            "principal_diff_px": float(check.principal_diff_px),
            "plane_bias_median_mm": median,
            "plane_bias_spread_mm": spread,
            "views": len(found),
            "poses": poses,
        }
        bias_pair = None if median is None or spread is None else (median, spread)
        (tmp / REPORT_FILE).write_text(intrinsics_report_markdown(check, bias_pair, sid, base["created"]))
    except (ValueError, OSError) as exc:
        LOGGER.warning("intrinsics check of %s failed: %s", sid, exc)
        for path in tmp.iterdir():
            path.unlink()
        result = failed_result(base, str(exc))
    write_json_atomic(tmp / RESULT_FILE, result)
    shutil.rmtree(final, ignore_errors=True)
    os.replace(tmp, final)
    return result


def main(argv: Sequence[str] | None = None) -> int:
    """Run one computation.

    Args:
        argv: --config <node config path> --session <sid> --result <rid>.

    Returns:
        0 when result.json was written (also for a failed check), 1 otherwise.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--session", required=True)
    parser.add_argument("--result", required=True)
    args = parser.parse_args(argv)
    cfg = load_config(args.config)
    store = SessionStore(Path(cfg.state_dir), cfg)
    compute_result(store, args.session, args.result, cfg, datetime.now().astimezone())
    return 0 if (store.result_dir(args.session, args.result) / RESULT_FILE).is_file() else 1


if __name__ == "__main__":
    raise SystemExit(main())
