"""Accepted arm to camera calibration at runtime: applied/arm_calibration.json overrides the configured `arm:` values.

The head_rgbd_calibration accept writes applied/arm_calibration.json and then the `arm_calibration` record of
applied/applied.json (ros2_common `arm_calibration_file`). The reload state is the shared ros2_common
`applied_arm_calibration.AppliedArmCalibration` (also used by filter_node's guard and web_ui): like the head chain
(head.AppliedHeadKinematics), the applied.json identity (mtime_ns, size, inode; `applied_marker_identity`) is the
reload trigger, checked without a thread of its own: `startup` loads it before the arm model is built, `tick` (the
monitor timer, every 0.25 s) re-reads it when applied.json changed.

The accepted calibration overrides exactly `ARM_SETTINGS_FIELDS` of the configured arm block (joint offsets, arm base
pose, mount height = base z); the configured client.yml values are the baseline, used when nothing is accepted (or the
record is removed). The tool point is not fitted (the fit reports the configured one) and stays configured. The merged
block is validated as ArmSettings; an unreadable or invalid calibration logs a failed calib_event (ERROR) and keeps
the values in use.

A reload is swapped into the running arm model only at an idle point: `ArmController.swap_calibration` takes the
motion lock without waiting, so no arm motion runs during the swap and none can start until it is done; while a
motion runs the swap waits for a later tick (one `apply` start event says so).

After an accepted calibration is applied (startup or reload), joint_offsets is cleared from the arm re-zero stale
marker when the servo zero did not change at or after the session's first still (`captured_from`; ros2_common
zero_change_refusal reads the marker's re-zero time and the bridge's zero change record): the offsets were measured in
the current zero. Otherwise (a later or unknown re-zero, a later zero restore) the marker is kept. The clear only
removes the item while the marker still names the re-zero that was checked. mcp_server is the one writer of that
clearing (calibration_marker.clear_stale, atomic).

Telemetry (calib_event, flow `arm_calibration`): `load` ok with {source, old, new} on every load (startup, each
changed calibration, `unchanged: true` when applied.json changed for something else), `load` failed for an invalid
calibration, `apply` start when a swap waits for a running motion (failed when the swap raised; the values in use
stay), `stale_marker` ok when joint_offsets was cleared
(failed when the marker could not be written).
"""

import logging
from collections.abc import Callable
from typing import Any

from pydantic import ValidationError
from ros2_common.head_rgbd_calibration.applied_arm_calibration import AppliedArmCalibration, assign_calibration
from ros2_common.head_rgbd_calibration.arm_calibration_file import (
    ARM_SETTINGS_FIELDS,
    ArmCalibrationError,
    ArmCalibrationFile,
    zero_change_refusal,
)

from .calib_event import emit_calib_event, new_run_id
from .calibration_marker import clear_stale, load_marker
from .config import ArmSettings
from .models import ArmCalibrationInUse

LOGGER = logging.getLogger(__name__)

FLOW = "arm_calibration"
CONFIG_SOURCE = "config"  # the `source` of a load without an accepted calibration (client.yml arm: values)
STALE_OFFSETS_ITEM = "joint_offsets"
DEFERRED_REASON = "an arm motion is running: the calibration is applied at the next idle point"

# Swaps the arm model to new arm settings at an idle point; False while a motion runs (ArmController.swap_calibration).
SwapFn = Callable[[ArmSettings, ArmCalibrationInUse | None], bool]


def arm_values(arm: ArmSettings) -> dict[str, Any]:
    """The calibrated `arm:` values (ARM_SETTINGS_FIELDS) for the telemetry.

    Args:
        arm (ArmSettings): Arm settings.

    Returns:
        dict[str, Any]: {joint_offsets_rad, arm_base_height_m, base_in_base_link (None when not configured)}.
    """
    return arm.model_dump(mode="json", include=set(ARM_SETTINGS_FIELDS))


def in_use(calibration: ArmCalibrationFile | None) -> ArmCalibrationInUse | None:
    """The identity of an accepted calibration as the arm state reports it.

    Args:
        calibration (ArmCalibrationFile | None): Accepted calibration, None for the configured values.

    Returns:
        ArmCalibrationInUse | None: session, result and accepted_at; None for the configured values.
    """
    if calibration is None:
        return None
    return ArmCalibrationInUse.model_validate(
        calibration.model_dump(mode="json", include={"session", "result", "accepted_at"})
    )


def source_name(calibration: ArmCalibrationFile | None) -> dict[str, Any] | str:
    """Telemetry name of a calibration source.

    Args:
        calibration (ArmCalibrationFile | None): Accepted calibration, None for the configured values.

    Returns:
        dict[str, Any] | str: {session, result, accepted_at}, or "config" for the configured values.
    """
    identity = in_use(calibration)
    return CONFIG_SOURCE if identity is None else identity.model_dump()


class ArmCalibrationWatcher(AppliedArmCalibration[ArmSettings]):
    """Loads the accepted arm calibration and hands changes to the arm at an idle point (see the module docstring).

    The reload state (baseline, applied.json identity, calibration and settings in use) is the shared
    AppliedArmCalibration; the telemetry, the deferred swap and the stale marker are mcp_server's.
    """

    def __init__(self, configured: ArmSettings) -> None:
        """Snapshot the configured arm settings (the baseline) before any override.

        Args:
            configured (ArmSettings): The arm settings as loaded from the config (applied_dir, calibration_stale_marker).
        """
        super().__init__(configured, configured.applied_dir)
        self.marker_path = configured.calibration_stale_marker
        # A validated change waiting for the idle point: (settings, calibration, telemetry run id); and whether its
        # wait was logged.
        self.pending: tuple[ArmSettings, ArmCalibrationFile | None, str] | None = None
        self.deferred_logged = False

    def read_logged(self, run_id: str) -> tuple[ArmSettings, ArmCalibrationFile | None] | None:
        """Read and validate the accepted calibration; a failure is logged (calib_event failed) and gives None.

        Args:
            run_id (str): calib_event run id.

        Returns:
            tuple[ArmSettings, ArmCalibrationFile | None] | None: (merged settings, calibration or None for the
                configured values), None when the calibration cannot be used.
        """
        try:
            return self.read()
        except (ArmCalibrationError, ValidationError) as exc:
            emit_calib_event(
                LOGGER,
                FLOW,
                run_id,
                "load",
                "failed",
                f"accepted arm calibration not applied, keeping the values in use: {exc}",
                {"applied_dir": str(self.applied_dir), "in_use": source_name(self.loaded)},
            )
            return None

    def startup(self, arm: ArmSettings) -> None:
        """Load the accepted calibration into the config's arm settings before the arm model is built.

        Args:
            arm (ArmSettings): The running config's arm settings (changed in place).
        """
        run_id = new_run_id()
        self.changed()
        loaded = self.read_logged(run_id)
        if loaded is None:
            return
        settings, calibration = loaded
        assign_calibration(arm, settings)
        self.commit(settings, calibration, run_id)

    def tick(self, swap: SwapFn) -> None:
        """Reload after an applied.json change and swap a changed calibration in at an idle point.

        Never raises for a bad calibration (logged, the values in use stay).

        Args:
            swap (SwapFn): ArmController.swap_calibration.
        """
        if self.changed():
            run_id = new_run_id()
            loaded = self.read_logged(run_id)
            if loaded is not None:
                settings, calibration = loaded
                if self.same(calibration):
                    self.pending = None
                    emit_calib_event(
                        LOGGER, FLOW, run_id, "load", "ok", data={"source": source_name(calibration), "unchanged": True}
                    )
                else:
                    self.pending = (settings, calibration, run_id)
                    self.deferred_logged = False
        if self.pending is None:
            return
        settings, calibration, run_id = self.pending
        try:
            swapped = swap(settings, in_use(calibration))
        except Exception as exc:  # the monitor timer must keep running: a failed swap keeps the values in use
            self.pending = None
            emit_calib_event(
                LOGGER, FLOW, run_id, "apply", "failed", f"swap failed, keeping the values in use: {exc}",
                {"source": source_name(calibration), "in_use": source_name(self.loaded)},
            )  # fmt: skip
            return
        if not swapped:
            if not self.deferred_logged:
                emit_calib_event(LOGGER, FLOW, run_id, "apply", "start", DEFERRED_REASON)
                self.deferred_logged = True
            return
        self.pending = None
        self.commit(settings, calibration, run_id)

    def commit(self, settings: ArmSettings, calibration: ArmCalibrationFile | None, run_id: str) -> None:
        """Record the calibration now in use, log old -> new and clear the stale marker's joint_offsets when due.

        Args:
            settings (ArmSettings): Settings now in use.
            calibration (ArmCalibrationFile | None): Accepted calibration now in use, None for the configured values.
            run_id (str): calib_event run id.
        """
        old = arm_values(self.use(settings, calibration))
        emit_calib_event(
            LOGGER,
            FLOW,
            run_id,
            "load",
            "ok",
            data={
                "source": source_name(calibration),
                "old": old,
                "new": arm_values(settings),
                "applied_dir": str(self.applied_dir),
            },
        )
        if calibration is not None:
            self.clear_stale_offsets(calibration, run_id)

    def clear_stale_offsets(self, calibration: ArmCalibrationFile, run_id: str) -> None:
        """Clear joint_offsets from the re-zero stale marker when the zero did not change at or after the calibration's
        capture (ros2_common zero_change_refusal: the marker's re-zero and the bridge's zero change record).

        Args:
            calibration (ArmCalibrationFile): The accepted calibration now in use.
            run_id (str): calib_event run id.
        """
        if self.marker_path is None:
            return
        marker = load_marker(self.marker_path)
        if marker is None or STALE_OFFSETS_ITEM not in marker.stale:
            return
        if zero_change_refusal(self.marker_path, calibration.captured_from) is not None:
            return
        data = {
            "marker": str(self.marker_path),
            "saved_at": marker.saved_at,
            "captured_from": calibration.captured_from.isoformat(),
        }
        try:
            cleared = clear_stale(self.marker_path, [STALE_OFFSETS_ITEM], only_if_saved_at=marker.saved_at)
        except OSError as exc:
            emit_calib_event(LOGGER, FLOW, run_id, "stale_marker", "failed", f"cannot clear joint_offsets: {exc}", data)
            return
        if cleared:
            emit_calib_event(LOGGER, FLOW, run_id, "stale_marker", "ok", data=data)
