"""Compute the head RGB-D calibration from one session's stored poses (runs as a niced child process)."""

import argparse
import logging
import os
import shutil
from collections.abc import Sequence
from datetime import datetime
from pathlib import Path
from typing import Any

from ros2_common.head_rgbd_calibration.calibrate import (
    calibrate_detections,
    failed_limits,
    outlier_pose_ids,
    report_markdown,
)
from ros2_common.head_rgbd_calibration.extrinsics import write_extrinsics_yaml
from ros2_common.head_rgbd_calibration.intrinsics import write_camera_calibration_yaml
from ros2_common.head_rgbd_calibration.urdf_handoff import read_chain, rewrite_tof_joint, tof_joint_origin

from head_rgbd_calibration.config import NodeConfig, load_config
from head_rgbd_calibration.store import RESULT_FILE, TEMP_PREFIX, SessionStore, write_json_atomic

LOGGER = logging.getLogger(__name__)

RGB_CAMERA_FILE = "head_camera_calibration.yaml"
TOF_CAMERA_FILE = "tof_calibration.yaml"
EXTRINSICS_FILE = "head_rgbd_extrinsics.yaml"
TOF_JOINT_FILE = "tof_joint.json"
URDF_FILE = "head.urdf"
REPORT_FILE = "report.md"
RGB_CAMERA_NAME = "head_camera"
TOF_CAMERA_NAME = "tof"
DATE_TIMESPEC = "seconds"


def failed_result(base: dict[str, Any], error: str) -> dict[str, Any]:
    """The result.json content of a computation that could not run."""
    return {**base, "passed": False, "error": error, "failed_limits": []}


def compute_result(store: SessionStore, sid: str, rid: str, cfg: NodeConfig, now: datetime) -> dict[str, Any]:
    """Compute one calibration result of a session and store it under results/<rid>/.

    The files are written into a temporary directory that is renamed once complete. A run over the limits keeps
    its report and result.json only; an input error (too few views, no ToF camera_info, unreadable URDF) writes
    result.json with `error` only.

    Args:
        store: Session store holding the poses.
        sid: Session id.
        rid: Result id (r-YYYYmmdd-HHMMSS).
        cfg: Node config (board, limits, head_urdf_file).
        now: Creation time.

    Returns:
        The result.json content (id, session, created, passed, failed_limits, rgb_rms_px, tof_rms_px,
        stereo_rms_px, plane_bias_mm, rgb_views, tof_views, stereo_pairs, lens_position, pose_ids, poses (per-pose
        errors and outlier fits), outlier_poses, limits (every
        acceptance limit used), plus `error`
        on failure).
    """
    final = store.result_dir(sid, rid)
    tmp = final.with_name(f"{TEMP_PREFIX}{rid}")
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.mkdir(parents=True)
    views = store.load_views(sid)
    base: dict[str, Any] = {
        "id": rid,
        "session": sid,
        "created": now.isoformat(timespec=DATE_TIMESPEC),
        "passed": False,
        "failed_limits": [],
        "rgb_rms_px": None,
        "tof_rms_px": None,
        "stereo_rms_px": None,
        "plane_bias_mm": None,
        "rgb_views": None,
        "tof_views": None,
        "stereo_pairs": None,
        "lens_position": views.lens_position,
        "pose_ids": views.pose_ids,
        "poses": [],
        "outlier_poses": [],
        "limits": cfg.limits.model_dump(),
    }
    try:
        if views.rgb_size is None or views.tof_size is None:
            raise ValueError("the session has no poses")
        run = calibrate_detections(
            views.views,
            views.rgb_size,
            views.tof_size,
            views.tof_info,
            cfg.board,
            cfg.limits,
            pose_ids=views.pose_ids,
        )
        date = base["created"]
        exceeded = failed_limits(run, cfg.limits)
        result = {
            **base,
            "passed": not exceeded,
            "failed_limits": exceeded,
            "rgb_rms_px": float(run.rgb.rms),
            "tof_rms_px": float(run.tof.rms),
            "stereo_rms_px": float(run.extrinsic.stereo_rms),
            "plane_bias_mm": float(run.plane.median_mm),
            "rgb_views": run.rgb.view_count,
            "tof_views": run.tof.view_count,
            "stereo_pairs": run.extrinsic.pair_count,
            "poses": [
                {
                    "id": p.pose_id,
                    "rgb_error_px": p.rgb_px,
                    "tof_error_px": p.tof_px,
                    "stereo_error_px": p.stereo_px,
                    "outlier": list(p.outlier),
                }
                for p in run.poses
            ],
            "outlier_poses": outlier_pose_ids(run),
        }
        if not exceeded:
            urdf_text = Path(cfg.head_urdf_file).read_text()
            origin = tof_joint_origin(read_chain(urdf_text), run.extrinsic)
            note = f"{date}, stereo RMS {run.extrinsic.stereo_rms:.3f} px"
            write_camera_calibration_yaml(
                tmp / RGB_CAMERA_FILE, run.rgb, RGB_CAMERA_NAME, run.rgb.image_size, cfg.limits
            )
            write_camera_calibration_yaml(
                tmp / TOF_CAMERA_FILE, run.tof, TOF_CAMERA_NAME, run.tof.image_size, cfg.limits
            )
            meta = {
                "session": sid,
                "lens_position": views.lens_position,
                "board": cfg.board.model_dump(),
                "date": date,
                "rgb_rms_px": float(run.rgb.rms),
                "tof_rms_px": float(run.tof.rms),
                "tof_sdk_diff_pct": run.sdk_diff_pct,
            }
            write_extrinsics_yaml(tmp / EXTRINSICS_FILE, run.extrinsic, run.plane, meta, cfg.limits)
            write_json_atomic(tmp / TOF_JOINT_FILE, {"xyz": list(origin.xyz), "rpy": list(origin.rpy), "note": note})
            (tmp / URDF_FILE).write_text(rewrite_tof_joint(urdf_text, origin, note))
        (tmp / REPORT_FILE).write_text(report_markdown(run, cfg.limits, sid, views.lens_position, date))
    except (ValueError, OSError) as exc:
        LOGGER.warning("calibration of %s failed: %s", sid, exc)
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
        0 when result.json was written (also for a failed calibration), 1 otherwise.
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
