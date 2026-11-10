"""GET /api/arm/calibration: the accepted arm calibration for the 3D arm view (Arm tab, map tab).

head_rgbd_calibration's arm to camera accept writes applied/arm_calibration.json and the `arm_calibration` record of
applied/applied.json. This route serves that calibration with the shared ros2_common pieces mcp_server and
filter_node use: `read_arm_calibration` reads it and `applied_marker_identity` (applied.json mtime_ns, size, inode) is
the reload key, so the file is re-read only after applied.json changed and a new Accept shows without a restart. The
frontend polls the route and draws the arm with the accepted mount and joint offsets; while `accepted` is null it keeps
the tab's configured `arm_offset` and no joint offsets (the configured client.yml values).

Body: {"accepted": {session, result, accepted_at, joint_offsets_rad, base_in_base_link} | null, "error": str | null}.
An unreadable or invalid calibration keeps the last good one (as mcp_server does) and reports `error`; a removed record
goes back to null. Without `arm_applied_calibration_dir` the route answers {"accepted": null, "error": null}.
"""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Any

import structlog
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from ros2_common.head_rgbd_calibration.applied_arm_calibration import MarkerIdentity, applied_marker_identity
from ros2_common.head_rgbd_calibration.arm_calibration_file import (
    ArmCalibrationError,
    ArmCalibrationFile,
    read_arm_calibration,
)

from .config import AppConfig

log = structlog.get_logger(__name__)

ROUTE_PATH = "/api/arm/calibration"
# The view's fields of the accepted calibration (the tool point is not fitted; capture time and score are not drawn).
PAYLOAD_FIELDS = {"session", "result", "accepted_at", "joint_offsets_rad", "base_in_base_link"}


def calibration_payload(calibration: ArmCalibrationFile | None) -> dict[str, Any] | None:
    """The accepted calibration as the frontend reads it.

    Args:
        calibration (ArmCalibrationFile | None): Accepted calibration, None when nothing is accepted.

    Returns:
        dict[str, Any] | None: {session, result, accepted_at (ISO 8601), joint_offsets_rad, base_in_base_link}.
    """
    if calibration is None:
        return None
    return calibration.model_dump(mode="json", include=PAYLOAD_FIELDS)


class ArmCalibrationCache:
    """The accepted arm calibration of one applied directory, re-read when applied.json's identity changes.

    Attributes:
        applied_dir: Applied calibration directory; None serves nothing accepted.
        stamp: applied.json identity of the last read (None before it, or without the file).
        calibration: Last good accepted calibration, None for none.
        error: Why the last read failed, None after a good read.
    """

    def __init__(self, applied_dir: Path | None) -> None:
        """Start empty: the first `get` reads when applied.json exists.

        Args:
            applied_dir (Path | None): Applied calibration directory; None serves nothing accepted.
        """
        self.applied_dir = applied_dir
        self.stamp: MarkerIdentity | None = None
        self.calibration: ArmCalibrationFile | None = None
        self.error: str | None = None
        self.lock = threading.Lock()

    def get(self) -> dict[str, Any]:
        """The route body, re-reading the calibration after an applied.json change.

        Returns:
            dict[str, Any]: {"accepted": calibration_payload | None, "error": str | None}.
        """
        with self.lock:
            stamp = applied_marker_identity(self.applied_dir)
            if self.applied_dir is not None and stamp != self.stamp:
                self.stamp = stamp
                try:
                    self.calibration = read_arm_calibration(self.applied_dir)
                    self.error = None
                except ArmCalibrationError as exc:
                    self.error = str(exc)
                    log.warning("arm_calibration_unreadable", applied_dir=str(self.applied_dir), error=self.error)
                else:
                    log.info(
                        "arm_calibration_loaded",
                        session=None if self.calibration is None else self.calibration.session,
                        result=None if self.calibration is None else self.calibration.result,
                    )
            return {"accepted": calibration_payload(self.calibration), "error": self.error}


def register_arm_calibration_routes(app: FastAPI, config: AppConfig) -> None:
    """Add GET /api/arm/calibration over config.arm_applied_calibration_dir.

    Args:
        app (FastAPI): Application.
        config (AppConfig): Validated configuration.
    """
    applied = config.arm_applied_calibration_dir
    cache = ArmCalibrationCache(None if applied is None else Path(applied))

    # Plain `def`: FastAPI runs it in its thread pool, so the stat and the small read never block the event loop.
    @app.get(ROUTE_PATH)
    def get_arm_calibration() -> JSONResponse:
        return JSONResponse(cache.get(), headers={"Cache-Control": "no-store"})
