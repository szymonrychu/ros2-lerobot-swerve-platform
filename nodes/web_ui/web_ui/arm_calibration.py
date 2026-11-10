"""GET /api/arm/calibration: the arm calibration the 3D arm views draw with (Arm tab, map tab).

The baseline is web_ui's `arm_calibration` (client.yml): the configured joint offsets, mount height and arm base pose,
the same values mcp_server's `arm` block and filter_node's `self_collision_guard.arm` hold (tests/
test_arm_guard_config.py pins them equal), so with nothing accepted the views draw the arm the other consumers model.
head_rgbd_calibration's arm to camera accept writes applied/arm_calibration.json and the `arm_calibration` record of
applied/applied.json; the shared ros2_common `AppliedArmCalibration` (the reload state mcp_server and filter_node build
on) merges it over the baseline, re-reading only after applied.json's identity (mtime_ns, size, inode) changed, so a
new Accept shows without a restart.

Body: {"calibration": {source, accepted, joint_offsets_rad, base_in_base_link} | null, "error": str | null}. `source`
is "accepted" (with `accepted` = {session, result, accepted_at}) or "configured" (`accepted` null). An unreadable or
invalid calibration, or one the baseline model refuses, keeps the values in use (as mcp_server does) and reports
`error`; a removed record goes back to the configured values. Without `arm_calibration` in the config the route answers
{"calibration": null, "error": null} and the views draw no arm.
"""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Any, Literal

import structlog
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration.applied_arm_calibration import AppliedArmCalibration, ArmCalibrationSettings
from ros2_common.head_rgbd_calibration.arm_calibration_file import ArmCalibrationError, ArmCalibrationFile

from .config import AppConfig

log = structlog.get_logger(__name__)

ROUTE_PATH = "/api/arm/calibration"
# The identity of an accepted calibration the view shows (capture time and score are not drawn).
ACCEPTED_FIELDS = {"session", "result", "accepted_at"}
# The calibrated values the view draws with (the mount height is the base z; the tool point is not fitted).
DRAWN_FIELDS = {"joint_offsets_rad", "base_in_base_link"}

Source = Literal["accepted", "configured"]


def calibration_source(calibration: ArmCalibrationFile | None) -> Source:
    """Where the values in use come from.

    Args:
        calibration (ArmCalibrationFile | None): Accepted calibration, None for the configured values.

    Returns:
        Source: "accepted" or "configured".
    """
    return "configured" if calibration is None else "accepted"


def calibration_payload(settings: ArmCalibrationSettings, calibration: ArmCalibrationFile | None) -> dict[str, Any]:
    """The calibration in use as the frontend reads it.

    Args:
        settings (ArmCalibrationSettings): The values in use (the baseline, or the accepted calibration merged over it).
        calibration (ArmCalibrationFile | None): Accepted calibration in use, None for the configured values.

    Returns:
        dict[str, Any]: {source, accepted: {session, result, accepted_at (ISO 8601)} | None, joint_offsets_rad,
            base_in_base_link}.
    """
    accepted = None if calibration is None else calibration.model_dump(mode="json", include=ACCEPTED_FIELDS)
    return {
        "source": calibration_source(calibration),
        "accepted": accepted,
        **settings.model_dump(mode="json", include=DRAWN_FIELDS),
    }


class ArmCalibrationCache:
    """The arm calibration in use: the accepted one of one applied directory over the configured values.

    Attributes:
        applied: Shared reload state (baseline, applied.json identity, the calibration and values in use).
        error: Why the last read or merge failed, None after a good one.
    """

    def __init__(self, baseline: ArmCalibrationSettings, applied_dir: Path | None) -> None:
        """Start on the configured values: the first `get` reads when applied.json exists.

        Args:
            baseline (ArmCalibrationSettings): The configured values.
            applied_dir (Path | None): Applied calibration directory; None serves the configured values only.
        """
        self.applied: AppliedArmCalibration[ArmCalibrationSettings] = AppliedArmCalibration(baseline, applied_dir)
        self.error: str | None = None
        self.lock = threading.Lock()

    def refresh(self) -> None:
        """Re-read and merge the accepted calibration after an applied.json change; a failure keeps the values in use."""
        if not self.applied.changed():
            return
        try:
            settings, calibration = self.applied.read()
        except (ArmCalibrationError, ValidationError) as exc:
            self.error = str(exc)
            log.warning(
                "arm_calibration_unusable",
                applied_dir=str(self.applied.applied_dir),
                keeping=calibration_source(self.applied.loaded),
                error=self.error,
            )
            return
        self.error = None
        if self.applied.same(calibration):
            return
        self.applied.use(settings, calibration)
        log.info(
            "arm_calibration_loaded",
            source=calibration_source(calibration),
            session=None if calibration is None else calibration.session,
            result=None if calibration is None else calibration.result,
        )

    def get(self) -> dict[str, Any]:
        """The route body, re-reading the calibration after an applied.json change.

        Returns:
            dict[str, Any]: {"calibration": calibration_payload, "error": str | None}.
        """
        with self.lock:
            self.refresh()
            return {"calibration": calibration_payload(self.applied.current, self.applied.loaded), "error": self.error}


def register_arm_calibration_routes(app: FastAPI, config: AppConfig) -> None:
    """Add GET /api/arm/calibration over config.arm_calibration.

    Args:
        app (FastAPI): Application.
        config (AppConfig): Validated configuration.
    """
    arm = config.arm_calibration
    cache = None if arm is None else ArmCalibrationCache(arm.baseline(), arm.applied_path())

    # Plain `def`: FastAPI runs it in its thread pool, so the stat and the small read never block the event loop.
    @app.get(ROUTE_PATH)
    def get_arm_calibration() -> JSONResponse:
        body = {"calibration": None, "error": None} if cache is None else cache.get()
        return JSONResponse(body, headers={"Cache-Control": "no-store"})
